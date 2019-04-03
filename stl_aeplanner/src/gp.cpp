#include <aeplanner/gp.h>

#include <eigen3/Eigen/Cholesky>

#include <omp.h>

#include <ros/ros.h>

namespace aeplanner
{
Eigen::MatrixXd sqExpKernel(const Eigen::Matrix<double, Eigen::Dynamic, 3>& x1,
                            const Eigen::Matrix<double, Eigen::Dynamic, 3>& x2,
                            double hyp_l, double hyp_sigma_f, double hyp_sigma_n)
{
  size_t n1 = x1.rows();
  size_t n2 = x2.rows();
  Eigen::MatrixXd k(n1, n2);

  // #pragma omp parallel for
  for (size_t i = 0; i < n2; ++i)
  {
    // FIXME: Optimize (find cool Eigen functions)
    for (size_t j = 0; j < n1; ++j)
    {
      double l = (x1.row(j) - x2.row(i)).norm();

      k(j, i) = std::pow(hyp_sigma_f, 2) * std::exp(-0.5 * std::pow(l / hyp_l, 2));
    }
  }

  return k;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
gp(const Eigen::VectorXd& y, const Eigen::Matrix<double, Eigen::Dynamic, 3>& x,
   const Eigen::Matrix<double, Eigen::Dynamic, 3>& x_star, double hyp_l,
   double hyp_sigma_f, double hyp_sigma_n)
{
  assert(y.size() != 0);
  assert(x.size() != 0);

  Eigen::MatrixXd k = sqExpKernel(x, x, hyp_l, hyp_sigma_f, hyp_sigma_n);
  Eigen::MatrixXd k_star = sqExpKernel(x, x_star, hyp_l, hyp_sigma_f, hyp_sigma_n);
  Eigen::MatrixXd k_star_star =
      sqExpKernel(x_star, x_star, hyp_l, hyp_sigma_f, hyp_sigma_n);

  // Algorithm 2.1 from Rasmussen & Williams
  // FIXME: Check that everything is correct and optmize (find cool Eigen functions)
  Eigen::LLT<Eigen::MatrixXd> l(k + std::pow(hyp_sigma_n, 2) *
                                        Eigen::MatrixXd::Identity(k.rows(), k.cols()));
  Eigen::MatrixXd alpha = l.matrixL().transpose().solve(l.matrixL().solve(y));

  Eigen::VectorXd posterior_mean = (k_star.transpose() * alpha);

  Eigen::MatrixXd v = l.matrixL().solve(k_star);
  Eigen::VectorXd posterior_variance = (k_star_star - (v.transpose() * v)).diagonal();

  return std::make_pair(posterior_mean, posterior_variance);
}
}  // namespace aeplanner