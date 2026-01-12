

#include "decider/scheduler/smooth_reference_line_builder.h"
#include "plan_common/log_data.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "absl/hash/hash.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"

#include "modules/cnoa_pnc/planning/proto/reference_line_qp_smoother.pb.h"

#include "plan_common/plan_common_defs.h"
#include "plan_common/container/strong_int.h"
#include "plan_common/maps/lane_path.h"
#include "plan_common/maps/lane_point.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/math/frenet_common.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/status_builder.h"
#include "plan_common/util/status_macros.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/planning_macros.h"

#include "router/route_sections_util.h"
#include "router/drive_passage_builder.h"

#include "decider/scheduler/path_boundary_builder.h"
#include "decider/scheduler/reference_line_qp_smoother.h"
#include "decider/scheduler/path_boundary_builder_helper.h"

namespace st::planning {
namespace {
// Reference line smoother weight.
// TODO: use planner param.
constexpr double kKappaWeight = 50.0;
constexpr double kBaseKappaGain = 0.8;
constexpr double kLengthWeight = 20.0;
constexpr double kBaseLengthGain = 0.2;
constexpr double kLateralOffsetWeight = 500.0;
constexpr double kDeltaWeight = 7.0;

absl::StatusOr<mapping::LanePath> GenerateLanePath(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids) {
  mapping::LanePath lane_path(psmm.map_ptr(), lane_ids,
                              /*start_fraction=*/0.0, /*end_fraction=*/1.0);

  // Check connectivity and validity.
  for (int i = 0; i < lane_path.size(); ++i) {
    SMM_ASSIGN_LANE_OR_ERROR_ISSUE(lane_info, psmm, lane_ids[i]);
    if (i + 2 >= lane_path.size()) break;
    const auto& outgoing_lanes = lane_info.next_lane_ids();
    if (std::find_if(outgoing_lanes.begin(), outgoing_lanes.end(),
                     [&lane_ids, i](mapping::ElementId id) {
                       return id == lane_ids[i + 1];
                     }) == outgoing_lanes.end()) {
      return absl::NotFoundError(
          absl::StrFormat("LanePath validation: lane %d does not have lane "
                          "%d as an outgoing lane",
                          lane_ids[i], lane_ids[i + 1]));
    }
  }

  return lane_path;
}

}  // namespace

absl::StatusOr<SmoothedReferenceCenterResult> SmoothLanePathByLaneIds(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width) {
  return SmoothLanePathBoundedByPathBoundary(psmm, lane_ids, half_av_width);
}

absl::StatusOr<SmoothedReferenceLineResultMap>
BuildSmoothedResultMapFromRouteSections(
    const PlannerSemanticMapManager& psmm, Vec2d ego_pos, double half_av_width,
    SmoothedReferenceLineResultMap results) {
  SCOPED_TRACE(__FUNCTION__);
  ASSIGN_OR_RETURN(const auto lane_ids_vec, FindLanesToSmoothFromRoute(psmm));

  constexpr double kSmoothSearchRadiusThreshold = 20.0;
  auto lane_ptrs = psmm.GetLanesInRadius(ego_pos, kSmoothSearchRadiusThreshold);
  if (lane_ptrs.empty()) {
    return absl::NotFoundError("");
    ;
  }

  absl::flat_hash_set<mapping::ElementId> neighbor_lane_id_set;
  for (const auto& lane_ptr : lane_ptrs) {
    neighbor_lane_id_set.insert(lane_ptr->id());
  }

  absl::flat_hash_set<mapping::ElementId> new_lane_id_set;
  int smooth_size = 0;
  for (const auto& lane_ids : lane_ids_vec) {
    bool is_neighbor_lanes = false;
    for (const auto& lane_id : lane_ids) {
      if (neighbor_lane_id_set.find(lane_id) != neighbor_lane_id_set.end()) {
        is_neighbor_lanes = true;
        break;
      }
    }

    if (!is_neighbor_lanes) {
      continue;
    }

    new_lane_id_set.insert(lane_ids.begin(), lane_ids.end());
    // if (!results.Contains(lane_ids))
    // optimze refline in each runonce

    SCOPED_TRACE(absl::StrCat("SmoothLanePathByLaneIds ", smooth_size + 1));
    auto smoothed_result =
        SmoothLanePathByLaneIds(psmm, lane_ids, half_av_width);
    if (smoothed_result.ok()) {
      results.AddResult(lane_ids, std::move(smoothed_result).value());
      smooth_size++;
    }
  }

  Log2DDS::LogDataV2("smooth_debug", smooth_size);

  std::vector<std::vector<mapping::ElementId>> lane_ids_to_delete;
  for (const auto& pair_it : results.smoothed_result_map()) {
    bool should_delete = true;
    for (const auto id : pair_it.first) {
      if (new_lane_id_set.find(id) != new_lane_id_set.end()) {
        should_delete = false;
        break;
      }
    }

    if (should_delete) {
      lane_ids_to_delete.push_back(pair_it.first);
    }
  }
  for (const auto& lane_ids : lane_ids_to_delete) {
    results.DeleteResult(lane_ids);
  }
  return results;
}

absl::StatusOr<std::vector<std::vector<mapping::ElementId>>>
FindLanesToSmoothFromRoute(const PlannerSemanticMapManager& psmm) {
  std::vector<std::vector<mapping::ElementId>> final_result;
  std::vector<std::vector<mapping::ElementId>> lane_ids_to_extend_vec;
  std::vector<mapping::ElementId> prev_lane_id_vec;
  const auto& sections = psmm.map_ptr()->route()->GetRouteInfo().sections;
  if (sections.empty()) return absl::NotFoundError("");
  const auto& front_section = sections.front();
  for (const auto lane_id : front_section.lane_ids) {
    prev_lane_id_vec.push_back(lane_id);
    if (IsTurningLanePath(psmm, lane_id)) {
      lane_ids_to_extend_vec.push_back({lane_id});
    }
  }

  for (int i = 1; i < sections.size(); ++i) {
    const auto& section_info = sections[i];
    std::vector<mapping::ElementId> new_lane_id_vec;
    std::vector<std::vector<mapping::ElementId>> new_lane_ids_to_extend_vec;
    for (const auto lane_id : section_info.lane_ids) {
      for (const auto& prev_lane_id : prev_lane_id_vec) {
        const auto prev_lane = psmm.map_ptr()->GetLaneById(prev_lane_id);
        if (!prev_lane) continue;
        if (IsOutgoingLane(psmm, *prev_lane, lane_id) &&
            !IsTurningLanePath(psmm, prev_lane_id) &&
            IsTurningLanePath(psmm, lane_id)) {
          new_lane_ids_to_extend_vec.push_back({lane_id});
        }
      }
      new_lane_id_vec.push_back(lane_id);
    }

    for (const auto& lane_id_vec : lane_ids_to_extend_vec) {
      const auto prev_lane = psmm.map_ptr()->GetLaneById(lane_id_vec.back());
      if (!prev_lane) continue;
      bool has_outgoing_to_smooth = false;
      for (const auto lane_id : new_lane_id_vec) {
        if (IsOutgoingLane(psmm, *prev_lane, lane_id) &&
            IsTurningLanePath(psmm, lane_id)) {
          has_outgoing_to_smooth = true;
          std::vector<mapping::ElementId> lane_ids = lane_id_vec;
          lane_ids.push_back(lane_id);
          new_lane_ids_to_extend_vec.emplace_back(std::move(lane_ids));
        }
      }

      if (!has_outgoing_to_smooth) {
        final_result.push_back(lane_id_vec);
      }
    }

    prev_lane_id_vec = std::move(new_lane_id_vec);
    lane_ids_to_extend_vec = std::move(new_lane_ids_to_extend_vec);
  }

  for (auto& lane_ids : lane_ids_to_extend_vec) {
    final_result.emplace_back(std::move(lane_ids));
  }

  if (final_result.empty()) {
    return absl::NotFoundError("");
  }

  return final_result;
}

absl::StatusOr<SmoothedReferenceCenterResult>
SmoothLanePathBoundedByPathBoundary(
    const PlannerSemanticMapManager& psmm,
    const std::vector<mapping::ElementId>& lane_ids, double half_av_width) {
  constexpr double kSmootherStepHint = 1.0;
  constexpr double maxSmootherDistance = 500.0;
  constexpr int kPre = ReferenceLineQpSmoother::kNumPreOptimizationPoints;
  constexpr int kPost = ReferenceLineQpSmoother::kNumPostOptimizationPoints;
  absl::flat_hash_map<mapping::ElementId,
                      PiecewiseLinearFunction<double, double>>
      smoothed_lane_path_map;
  ASSIGN_OR_RETURN(const auto lane_path, GenerateLanePath(psmm, lane_ids));

  // ==== Sampling ====
  const double length = lane_path.length();
  CHECK_GE(length, 0);
  if (length > maxSmootherDistance) {
    // Too long, unable to smooth.
    return absl::NotFoundError("Lane path is too long.");
  }

  const int n = static_cast<int>(length / kSmootherStepHint + 1.0) + 1;
  if (n < std::max(kPre + kPost + 1, 2)) {
    // Too short, no need to smooth.
    return absl::NotFoundError("Lane path is too short.");
  }
  const int opt_n = n - kPre - kPost;
  const double step = length / (n - 1);

  std::vector<double> sample_s;
  std::vector<mapping::LanePoint> lane_points;
  sample_s.reserve(n);
  lane_points.reserve(n);
  for (int i = 0; i < n; ++i) {
    // O(log(num of lanes in lane path)) aciton, cached beforehand.
    sample_s.push_back(i * step);
    lane_points.push_back(lane_path.ArclengthToLanePoint(i * step));
  }

  // ===== Build problem input ====
  ReferenceLineQpSmoother::Input input;
  input.base_poses.reserve(n);
  input.base_tangents.reserve(n);
  input.l_lower_bound.reserve(opt_n);
  input.l_upper_bound.reserve(opt_n);
  input.pre_fixed_lateral_offsets.fill(0.0);
  input.post_fixed_lateral_offsets.fill(0.0);

  std::optional<std::pair<Vec2d, Vec2d>> pre_pose_tangent;
  int current_lane_index = 0;
  auto lanes = lane_path.lane_seq()->lanes();
  const int lane_num = lanes.size();
  constexpr double kLaneSeqSmoothDistanceH = 15.0;
  constexpr double kLaneSeqSmoothDistanceC = 8.0;

  for (int i = 0; i < n; ++i) {
    const mapping::LanePoint& lane_point = lane_points[i];
    const Vec2d base_pose = ComputeLanePointPos(psmm, lane_point);
    const Vec2d base_tangent = ComputeLanePointTangent(psmm, lane_point);

    input.base_poses.push_back(base_pose);
    input.base_tangents.push_back(base_tangent);

    if (i >= kPre && i < n - kPost) {
      while (current_lane_index < lane_num - 1 &&
             lane_point.lane_id() != lane_ids[current_lane_index]) {
        current_lane_index++;
      }

      auto lane_segment = lane_path.lane_segment(current_lane_index);
      double valid_lane_fraction =
          (psmm.IsOnHighway() ? kLaneSeqSmoothDistanceH
                              : kLaneSeqSmoothDistanceC) /
          (lane_segment.end_s - lane_segment.start_s);
      bool is_turn_type = lanes[current_lane_index]->turn_type() ==
                              ad_byd::planning::LEFT_TURN ||
                          lanes[current_lane_index]->turn_type() ==
                              ad_byd::planning::RIGHT_TURN;
      bool is_split_type = lanes[current_lane_index]->split_topology() ==
                               SplitTopology::TOPOLOGY_SPLIT_LEFT ||
                           lanes[current_lane_index]->split_topology() ==
                               SplitTopology::TOPOLOGY_SPLIT_RIGHT;
      bool is_merge_type = lanes[current_lane_index]->merge_topology() ==
                               MergeTopology::TOPOLOGY_MERGE_LEFT ||
                           lanes[current_lane_index]->merge_topology() ==
                               MergeTopology::TOPOLOGY_MERGE_RIGHT;

      if (is_turn_type || is_split_type || is_merge_type ||
          lane_point.fraction() < valid_lane_fraction ||
          1 - valid_lane_fraction < lane_point.fraction()) {
        // Limited by boundary.
        double l_min = -half_av_width;
        double l_max = half_av_width;
        bool is_last_split = false;
        if (current_lane_index > 0) {
          is_last_split = lanes[current_lane_index - 1]->split_topology() ==
                              SplitTopology::TOPOLOGY_SPLIT_LEFT ||
                          lanes[current_lane_index - 1]->split_topology() ==
                              SplitTopology::TOPOLOGY_SPLIT_RIGHT;
        }

        bool is_next_merge = false;
        if (current_lane_index < lane_num - 1) {
          is_next_merge = lanes[current_lane_index + 1]->merge_topology() ==
                              MergeTopology::TOPOLOGY_MERGE_LEFT ||
                          lanes[current_lane_index + 1]->merge_topology() ==
                              MergeTopology::TOPOLOGY_MERGE_RIGHT;
        }

        bool is_highway_enhance_case = false;
        if (psmm.IsOnHighway()) {
          if ((is_split_type || is_merge_type || is_last_split ||
               is_next_merge) &&
              ((lane_point.fraction() < valid_lane_fraction ||
                1 - valid_lane_fraction < lane_point.fraction()))) {
            is_highway_enhance_case = true;
          }
        }

        if (is_highway_enhance_case) {
          l_min = std::max(l_min, -0.50);
          l_max = std::min(l_max, 0.50);
        } else {
          l_min = std::max(l_min, -0.25);
          l_max = std::min(l_max, 0.25);
        }

        // Limited by lane center radius.
        if (pre_pose_tangent) {
          const double d_theta = NormalizeAngle(
              base_tangent.FastAngle() - pre_pose_tangent->second.FastAngle());
          if (std::abs(d_theta) * kMaxLateralOffset >= step) {
            const double turn_radius = step / d_theta;
            if (turn_radius < 0.0) {
              l_min = std::max(l_min, turn_radius);
            } else {
              l_max = std::min(l_max, turn_radius);
            }
          }
        }

        constexpr double kEpsilon = 0.1;  // m.
        if (l_max - l_min < kEpsilon) {
          return absl::NotFoundError(
              "Implausible lower and upper bound for smoother.");
        }

        input.l_lower_bound.push_back(l_min);
        input.l_upper_bound.push_back(l_max);
      } else {
        input.l_lower_bound.push_back(0.0);
        input.l_upper_bound.push_back(0.0);
      }
    }

    pre_pose_tangent = std::make_pair(base_pose, base_tangent);
  }

  // ====Try smooth the lane path====
  planning::ReferenceLineQpSmootherParamProto param;
  param.set_lateral_offset_weight(kLateralOffsetWeight);
  param.set_delta_weight(kDeltaWeight);
  param.set_kappa_weight(kKappaWeight);
  param.set_lambda_weight(0.0);
  param.set_base_kappa_gain(kBaseKappaGain);
  param.set_length_weight(kLengthWeight);
  param.set_base_length_gain(kBaseLengthGain);

  const ReferenceLineQpSmoother smoother(&param, std::move(input));
  const std::optional<std::vector<double>> result = smoother.Solve();
  if (!result) {
    const auto debug_string =
        absl::StrCat("Reference line qp smoother failed on lane path<",
                     lane_path.DebugString(), ">");
    return absl::InternalError(debug_string);
  }
  CHECK_EQ(result->size(), opt_n);
  std::vector<std::string> s;
  for (int i = 0; i < lane_ids.size(); ++i) {
    s.push_back(absl::StrCat(lane_ids[i]));
  }

  std::vector<double> smoothed_l;
  smoothed_l.reserve(n);
  for (int i = 0; i < n; ++i) {
    if (i < kPre || i >= n - kPost) {
      smoothed_l.push_back(0.0);
    } else {
      smoothed_l.push_back((*result)[i - kPre]);
    }

    s.push_back(absl::StrCat(smoothed_l.back()));
  }

  Log2DDS::LogDataV2("smooth_debug", s);
  // ==== Append the result ====
  int sample_index = 0;
  for (int i = 0; i < lane_path.size(); ++i) {
    const double start_s = lane_path.start_s(i);
    const double end_s = lane_path.end_s(i);
    if (std::abs(end_s - start_s) < 1e-8) {
      continue;
    };

    while (sample_index < n && sample_s[sample_index] <= start_s) {
      ++sample_index;
    }
    const int last_sample_leq_start_s = std::max(sample_index - 1, 0);
    while (sample_index < n && sample_s[sample_index] < end_s) {
      ++sample_index;
    }
    const int first_sample_geq_end_s =
        std::max(sample_index, last_sample_leq_start_s + 1);

    if (smoothed_lane_path_map.contains(lane_path.lane_id(i))) {
      // Will NOT override "first loop" smoothed result if same lane get
      // smoothed twice.
      continue;
    }

    const int num_sub_sample =
        first_sample_geq_end_s - last_sample_leq_start_s + 1;
    std::vector<double> lane_fraction;
    std::vector<double> smoothed_offset;
    lane_fraction.reserve(num_sub_sample);
    smoothed_offset.reserve(num_sub_sample);

    for (int j = last_sample_leq_start_s; j <= first_sample_geq_end_s; ++j) {
      lane_fraction.push_back((sample_s[j] - start_s) / (end_s - start_s));
      smoothed_offset.push_back(smoothed_l[j]);
    }

    smoothed_lane_path_map[lane_path.lane_id(i)] = PiecewiseLinearFunction(
        std::move(lane_fraction), std::move(smoothed_offset));
  }

  return SmoothedReferenceCenterResult{.lane_id_to_smoothed_lateral_offset =
                                           std::move(smoothed_lane_path_map)};
}

}  // namespace st::planning
