

#ifndef AD_BYD_PLANNING_COMMON_FRAME_H
#define AD_BYD_PLANNING_COMMON_FRAME_H
#include <memory>
#include <vector>

#include "planning_data.h"
#include "plan_common/maps/polyline.h"
#include "plan_common/maps/lane_sequence.h"
namespace ad_byd {
namespace planning {
class Frame {
 public:
  Frame();
  ~Frame() = default;

  void SetPrevPlanningData(const PlanningDataConstPtr& planning_data_ptr);

  PlanningDataVecPtr mutable_planning_data() { return planning_datas_; }
  CandidateSequencesPtr mutable_candidate_sequences() {
    return candidate_sequences_;
  }
  PolylineInfoPtr mutable_polyline_info() { return polyline_info_ptr_; }
  const PlanningDataVecConstPtr planning_data() const {
    return planning_datas_;
  }
  const CandidateSequencesConstPtr candidate_sequences() const {
    return candidate_sequences_;
  }
  const PlanningDataConstPtr previous_planning_data() const {
    return previous_planning_data_;
  }
  const PlanningDataConstPtr target_planning_data() const {
    return target_planning_data_;
  }
  const PolylineInfoPtr polyline_info() const { return polyline_info_ptr_; }

 private:
  PlanningDataPtr target_planning_data_ = nullptr;
  PlanningDataConstPtr previous_planning_data_ = nullptr;
  PlanningDataVecPtr planning_datas_ = nullptr;
  CandidateSequencesPtr candidate_sequences_ = nullptr;
  PolylineInfoPtr polyline_info_ptr_ = nullptr;
};

using FramePtr = std::shared_ptr<Frame>;
using FrameConstPtr = std::shared_ptr<const Frame>;
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_INPUT_FRAME_H
