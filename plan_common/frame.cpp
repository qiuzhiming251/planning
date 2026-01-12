
#include "frame.h"

namespace ad_byd {
namespace planning {

Frame::Frame() {
  candidate_sequences_ =
      std::make_shared<std::vector<NavigableLaneSequencePtr>>();
  planning_datas_ = std::make_shared<std::vector<PlanningDataPtr>>();
  polyline_info_ptr_ = std::make_shared<PolylineInfo>();
}

void Frame::SetPrevPlanningData(const PlanningDataConstPtr& planning_data_ptr) {
  previous_planning_data_ = planning_data_ptr;
}

}  // namespace planning
}  // namespace ad_byd