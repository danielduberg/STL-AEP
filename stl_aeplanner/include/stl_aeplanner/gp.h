#ifndef AEPLANNER_GP_H
#define AEPLANNER_GP_H

#include <eigen3/Eigen/Dense>
#include <vector>

namespace STL_aeplanner
{
Eigen::MatrixXd sqExpKernel(const Eigen::Matrix<double, Eigen::Dynamic, 3>& x1,
                            const Eigen::Matrix<double, Eigen::Dynamic, 3>& x2, double hyp_l, double hyp_sigma_f,
                            double hyp_sigma_n);

std::pair<Eigen::VectorXd, Eigen::VectorXd> gp(const Eigen::VectorXd& y,
                                               const Eigen::Matrix<double, Eigen::Dynamic, 3>& x,
                                               const Eigen::Matrix<double, Eigen::Dynamic, 3>& x_star, double hyp_l,
                                               double hyp_sigma_f, double hyp_sigma_n);
}  // namespace STL_aeplanner

#endif  // AEPLANNER_GP_H