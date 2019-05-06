#ifndef STLAEPLANNER_NODELET_H
#define STLAEPLANNER_NODELET_H

#include <nodelet/nodelet.h>

#include <stl_aeplanner/stl_aeplanner.h>

namespace stl_aeplanner
{
class STLAEPlannerNodelet : public nodelet::Nodelet
{
private:
  STLAEPlanner* stl_aeplanner_;

public:
  virtual void onInit();
  ~STLAEPlannerNodelet();
};
}  // namespace stl_aeplanner

#endif
