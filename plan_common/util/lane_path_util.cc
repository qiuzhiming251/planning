

#include <algorithm>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "plan_common/base/macros.h"
#include "plan_common/log_data.h"
#include "plan_common/maps/lane_path_data.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_util.h"
#include "plan_common/util/lane_path_util.h"
#include "plan_common/util/lane_point_util.h"
#include "plan_common/util/planner_semantic_map_util.h"
#include "plan_common/util/spatial_search_util.h"
#include "plan_common/util/status_macros.h"

namespace st::planning {
namespace {
// TODO: consider to use macros to replace this.
mapping::LanePath BuildLanePathFromDataOrReportIssue(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm) {
  auto lane_path_or = BuildLanePathFromData(data, psmm);
  if (UNLIKELY(!lane_path_or.ok())) {
    return mapping::LanePath(psmm.map_ptr(), data);
  }
  return *lane_path_or;
}

mapping::LanePath ConnectLanePathOrReportIssue(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, bool fail_return_this,
    double distance_threshold = 0.01) {
  auto lane_path_or =
      ConnectLanePath(lane_path, other, psmm, distance_threshold);
  if (UNLIKELY(!lane_path_or.ok())) {
    return fail_return_this ? lane_path : other;
  }
  return *lane_path_or;
}
}  // namespace

absl::StatusOr<mapping::LanePath> BuildLanePathFromData(
    const mapping::LanePathData& data, const PlannerSemanticMapManager& psmm) {
  const int n = data.size();

  std::vector<double> lane_lengths;
  lane_lengths.reserve(n);
  std::vector<double> lane_end_s;
  lane_end_s.reserve(n + 1);
  lane_end_s.push_back(0.0);
  if (n == 1) {
    const auto& lane_info_ptr =
        psmm.FindCurveLaneByIdOrNull(data.lane_ids().front());
    if (lane_info_ptr == nullptr) {
      return absl::NotFoundError(
          absl::StrCat("lane ", data.lane_ids().front(), " not found."));
    }
    lane_lengths.push_back(lane_info_ptr->curve_length());
    lane_end_s.push_back(lane_info_ptr->curve_length() *
                         (data.end_fraction() - data.start_fraction()));
    return mapping::LanePath(data, std::move(lane_end_s),
                             std::move(lane_lengths));
  }
  for (int i = 0; i < n; ++i) {
    const auto id = data.lane_ids()[i];
    const auto& lane_info_ptr = psmm.FindCurveLaneByIdOrNull(id);
    if (lane_info_ptr == nullptr) {
      return absl::NotFoundError(absl::StrCat("lane ", id, " not found."));
    }
    lane_lengths.push_back(lane_info_ptr->curve_length());

    double fraction = 1.0;
    if (i == 0) {
      fraction = data.lane_path_in_forward_direction()
                     ? (1.0 - data.start_fraction())
                     : data.start_fraction();
    } else if (i + 1 == n) {
      fraction = data.lane_path_in_forward_direction()
                     ? data.end_fraction()
                     : (1.0 - data.end_fraction());
    }
    lane_end_s.push_back(lane_end_s.back() +
                         lane_info_ptr->curve_length() * fraction);
  }

  return mapping::LanePath(data, std::move(lane_end_s),
                           std::move(lane_lengths));
}

bool IsLanePathConnectedTo(const mapping::LanePath& lane_path,
                           const mapping::LanePath& other,
                           const PlannerSemanticMapManager& psmm,
                           double distance_threshold) {
  // NOTE: below are copied from lane_path.cc and slightly refactored.
  if (lane_path.end_fraction() == 1.0 && other.start_fraction() == 0.0) {
    // TODO: call smm util later.
    const auto& lane_info =
        psmm.FindCurveLaneByIdOrNull(lane_path.lane_ids().back());
    if (lane_info == nullptr) {
      return false;
    }
    const auto& outgoing_lanes_ids = lane_info->next_lane_ids();
    const mapping::ElementId front_id = other.lane_ids().front();
    return std::find_if(outgoing_lanes_ids.begin(), outgoing_lanes_ids.end(),
                        [front_id](mapping::ElementId id) {
                          return id == front_id;
                        }) != outgoing_lanes_ids.end();
  }
  if (lane_path.lane_ids().back() != other.lane_ids().front()) {
    return false;
  }

  const auto& last_lane_info =
      psmm.FindCurveLaneByIdOrNull(lane_path.lane_ids().back());
  if (last_lane_info == nullptr) {
    return false;
  }
  return std::abs(lane_path.end_fraction() - other.start_fraction()) *
             last_lane_info->curve_length() <
         distance_threshold;
}

absl::StatusOr<mapping::LanePath> ConnectLanePath(
    const mapping::LanePath& lane_path, const mapping::LanePath& other,
    const PlannerSemanticMapManager& psmm, double distance_threshold) {
  if (!IsLanePathConnectedTo(lane_path, other, psmm, distance_threshold)) {
    return absl::FailedPreconditionError("lane path not connected.");
  }

  std::vector<mapping::ElementId> ids = lane_path.lane_ids();
  if (lane_path.lane_ids().back() == other.lane_ids().front()) {
    ids.pop_back();
  }
  ids.insert(ids.end(), other.lane_ids().begin(), other.lane_ids().end());

  return BuildLanePathFromData(
      mapping::LanePathData(lane_path.start_fraction(), other.end_fraction(),
                            std::move(ids)),
      psmm);
}

mapping::LanePath BackwardExtendTargetAlignedRouteLanePath(
    const PlannerSemanticMapManager& psmm, bool left,
    const mapping::LanePoint start_point, const mapping::LanePath& target) {
  SMM_ASSIGN_LANE_OR_RETURN_ISSUE(start_lane, psmm, start_point.lane_id(),
                                  mapping::LanePath());
  const auto start_range = mapping::GetNeighborRange(left, target, start_lane);
  const double min_fraction_error = 1.0 / start_lane.curve_length();

  if (start_point.fraction() > start_range.second + min_fraction_error) {
    return mapping::LanePath();
  }

  mapping::LanePath extended_path = BuildLanePathFromDataOrReportIssue(
      mapping::LanePathData(start_range.first, start_point.fraction(),
                            {start_lane.id()}),
      psmm);

  while (extended_path.start_fraction() == 0.0) {
    bool extended = false;
    auto lane_id = extended_path.front().lane_id();
    auto lane_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
    if (lane_ptr == nullptr) {
      // QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
      //                   QIssueSubType::QIST_SEMANTIC_MAP,
      //                   "Cannot find the map ", absl::StrCat(lane_id));
      break;
    }
    for (const auto& id : lane_ptr->pre_lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(in_lane, psmm, id);
      const auto range = mapping::GetNeighborRange(left, target, in_lane);
      if (range.second == 1.0) {
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(range.first, 1.0, {in_lane.id()}), psmm);
        extended_path = ConnectLanePathOrReportIssue(
            tmp_lane_path, extended_path, psmm, /*fail_return_this=*/false);
        extended = true;
        break;
      }
    }
    if (!extended) break;
    lane_id = extended_path.front().lane_id();
    lane_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
    if (lane_ptr == nullptr) {
      // QISSUEX_WITH_ARGS(QIssueSeverity::QIS_FATAL, QIssueType::QIT_BUSINESS,
      //                   QIssueSubType::QIST_SEMANTIC_MAP,
      //                   "Cannot find the map ", absl::StrCat(lane_id));
      continue;
    }
  }
  return extended_path;
}

mapping::LanePath BackwardExtendLanePath(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len,
    const std::function<bool(const ad_byd::planning::Lane&)>*
        nullable_should_stop_and_avoid_extend) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.front();
  mapping::LanePath backward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);

  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, start_lp.lane_id());
    if (lane_info.center_line().points().size() < 1) {
      break;
    }
    if (nullable_should_stop_and_avoid_extend != nullptr &&
        (*nullable_should_stop_and_avoid_extend)(lane_info)) {
      break;
    }
    if (start_lp.fraction() == 0.0) {
      if (lane_info.pre_lane_ids().empty()) break;
      if (lane_info.pre_lane_ids().size() == 1) {
        start_lp = mapping::LanePoint(lane_info.pre_lane_ids().front(), 1.0);
      } else {
        constexpr double kSampleLen = 4.0;  // m.
        const Vec2d origin_pt = lane_info.points().front();
        const Vec2d next_pt = lane_info.LerpPointFromFraction(
            std::min(1.0, kSampleLen / lane_info.curve_length()));
        const Vec2d heading = (next_pt - origin_pt).normalized();

        double max_projection = std::numeric_limits<double>::lowest();
        mapping::ElementId opt_incoming_id = lane_info.pre_lane_ids().front();
        for (int i = 0; i < lane_info.pre_lane_ids().size(); ++i) {
          const mapping::ElementId tmp_lane_id = lane_info.pre_lane_ids()[i];
          SMM_ASSIGN_LANE_OR_BREAK_ISSUE(tmp_lane_info, psmm, tmp_lane_id);
          if (tmp_lane_info.center_line().points().size() < 1) {
            break;
          }
          const Vec2d prev_pt = tmp_lane_info.LerpPointFromFraction(
              std::max(0.0, 1.0 - kSampleLen / tmp_lane_info.curve_length()));

          const Vec2d tmp_heading = (origin_pt - prev_pt).normalized();

          const double proj = heading.Dot(tmp_heading);

          if (proj > max_projection) {
            max_projection = proj;
            opt_incoming_id = tmp_lane_id;
          }
        }
        start_lp = mapping::LanePoint(opt_incoming_id, 1.0);
      }

    } else {
      const double len = lane_info.curve_length();
      if (len * start_lp.fraction() > extend_len) {
        const double fraction = start_lp.fraction() - extend_len / len;
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(fraction, start_lp.fraction(),
                                  {lane_info.id()}),
            psmm);
        auto tmp_lane_path_ext =
            ConnectLanePath(tmp_lane_path, backward_path, psmm);
        if (tmp_lane_path_ext.ok()) {
          backward_path = std::move(tmp_lane_path_ext).value();
        }

        break;
      } else {
        extend_len -= len * start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(0.0, start_lp.fraction(), {lane_info.id()}),
            psmm);
        auto tmp_lane_path_ext =
            ConnectLanePath(tmp_lane_path, backward_path, psmm);
        if (tmp_lane_path_ext.ok()) {
          backward_path = std::move(tmp_lane_path_ext).value();
        } else {
          break;
        }

        start_lp = mapping::LanePoint(start_lp.lane_id(), 0.0);
      }
    }
  }

  auto tmp_lane_path_ext = ConnectLanePath(backward_path, raw_lane_path, psmm);

  return tmp_lane_path_ext.ok() ? *tmp_lane_path_ext : raw_lane_path;
}

absl::StatusOr<mapping::LanePath> ForwardExtendLanePathWithMinimumHeadingDiff(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len) {
  if (extend_len <= 0.0) return raw_lane_path;
  mapping::LanePoint start_lp = raw_lane_path.back();
  ASSIGN_OR_RETURN(
      auto forward_path,
      BuildLanePathFromData(mapping::LanePathData(start_lp), psmm));
  while (extend_len > 0.0) {
    if (start_lp.fraction() == 1.0) {
      ASSIGN_OR_BREAK(start_lp, FindOutgoingLanePointWithMinimumHeadingDiff(
                                    psmm, start_lp.lane_id()));
    } else {
      SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, start_lp.lane_id());
      const double len = lane_info.curve_length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        ASSIGN_OR_CONTINUE(
            const auto tmp_lane_path,
            BuildLanePathFromData(
                mapping::LanePathData(start_lp.fraction(), fraction,
                                      {lane_info.id()}),
                psmm));
        ASSIGN_OR_CONTINUE(forward_path,
                           ConnectLanePath(forward_path, tmp_lane_path, psmm));
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        ASSIGN_OR_CONTINUE(
            const auto tmp_lane_path,
            BuildLanePathFromData(mapping::LanePathData(start_lp.fraction(),
                                                        1.0, {lane_info.id()}),
                                  psmm));
        ASSIGN_OR_CONTINUE(forward_path,
                           ConnectLanePath(forward_path, tmp_lane_path, psmm));
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }
  return ConnectLanePath(raw_lane_path, forward_path, psmm);
}

mapping::LanePath ForwardExtendLanePathWithoutFork(
    const PlannerSemanticMapManager& psmm,
    const mapping::LanePath& raw_lane_path, double extend_len) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);
  while (extend_len > 0.0) {
    SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
    if (start_lp.fraction() == 1.0) {
      if (lane_info.next_lane_ids().size() != 1) break;
      const auto out_lane_id = lane_info.next_lane_ids().front();
      start_lp = mapping::LanePoint(out_lane_id, 0.0);

    } else {
      const double len = lane_info.curve_length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), fraction,
                                  {lane_info.id()}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), 1.0, {lane_info.id()}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }
  return ConnectLanePathOrReportIssue(raw_lane_path, forward_path, psmm,
                                      /*fail_return_this=*/true);
}

absl::StatusOr<mapping::LanePath> FindNearestLanePathFromEgoPose(
    const PoseProto& pose, const PlannerSemanticMapManager& psmm,
    double required_min_length) {
  const auto close_points =
      FindCloseLanePointsToSmoothPointWithHeadingBoundAmongLanesAtLevel(
          psmm, Vec2d(pose.pos_smooth().x(), pose.pos_smooth().y()), pose.yaw(),
          /*heading_penalty_weight=*/0.0, /*spatial_distance_threshold=*/5.0,
          /*angle_error_threshold=*/M_PI_2);
  if (close_points.empty()) {
    return absl::NotFoundError(
        absl::StrCat("Can not find lane point around pose [",
                     pose.pos_smooth().x(), ",", pose.pos_smooth().y(), "]."));
  }

  return ForwardExtendLanePathWithMinimumHeadingDiff(
      psmm,
      BuildLanePathFromDataOrReportIssue(
          mapping::LanePathData(close_points.front()), psmm),
      required_min_length);
}

Vec2d ArclengthToPos(const PlannerSemanticMapManager& psmm,
                     const mapping::LanePath& lane_path, double s) {
  const auto lane_point = lane_path.ArclengthToLanePoint(s);
  return ComputeLanePointPos(psmm, lane_point);
}

double ArclengthToLerpTheta(const PlannerSemanticMapManager& psmm,
                            const mapping::LanePath& lane_path, double s) {
  return LaneIndexPointToLerpTheta(psmm, lane_path,
                                   lane_path.ArclengthToLaneIndexPoint(s));
}

double LaneIndexPointToLerpTheta(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    const mapping::LanePath::LaneIndexPoint& lane_index_point) {
  const int lane_index = lane_index_point.first;
  const double lane_fraction = lane_index_point.second;
  CHECK_GE(lane_index, 0);
  CHECK_LT(lane_index, lane_path.size());
  CHECK_GE(lane_fraction, 0.0);
  CHECK_LE(lane_fraction, 1.0);

  mapping::LanePoint lane_point(lane_path.lane_ids()[lane_index],
                                lane_fraction);
  double tangent;
  std::string math_point =
      absl::StrCat("index: ", lane_index, " fraation: ", lane_fraction);
  if (lane_index == lane_path.size() - 1) {
    // No successor lane.
    math_point += " No successor ";
    tangent = ComputeLanePointLerpTheta(psmm, lane_point);
    math_point += absl::StrCat("tangent: ", tangent);
    Log2DDS::LogDataV2("math_point", math_point);
  } else {
    const auto succ_lane_id = lane_path.lane_ids()[lane_index + 1];
    math_point += absl::StrCat(" succ_lane_id : ", succ_lane_id);
    tangent = ComputeLanePointLerpThetaWithSuccessorLane(psmm, succ_lane_id,
                                                         lane_point);
    math_point += absl::StrCat("tangent: ", tangent);
    Log2DDS::LogDataV2("math_point", math_point);
  }

  return tangent;
}

std::vector<ad_byd::planning::LaneConstPtr> GetLanesInfoBreakIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<ad_byd::planning::LaneConstPtr> lanes_info;
  lanes_info.reserve(lane_path.size());
  for (const auto& lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, lane_id);
    lanes_info.push_back(
        std::make_shared<const ad_byd::planning::Lane>(lane_info));
  }
  return lanes_info;
}

std::vector<ad_byd::planning::LaneConstPtr> GetLanesInfoContinueIfNotFound(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  std::vector<ad_byd::planning::LaneConstPtr> lanes_info;
  lanes_info.reserve(lane_path.size());
  for (const auto& lane_id : lane_path.lane_ids()) {
    SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(lane_info, psmm, lane_id);
    lanes_info.push_back(
        std::make_shared<const ad_byd::planning::Lane>(lane_info));
  }
  return lanes_info;
}

absl::StatusOr<mapping::LanePath> TrimTrailingNotFoundLanes(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path) {
  if (lane_path.IsEmpty()) {
    return absl::InvalidArgumentError("Input lane path is empty.");
  }

  if (const auto& last_lane_ptr =
          psmm.FindCurveLaneByIdOrNull(lane_path.lane_ids().back());
      last_lane_ptr != nullptr) {
    return lane_path;
  }

  if (const auto& first_lane_ptr =
          psmm.FindCurveLaneByIdOrNull(lane_path.lane_ids().front());
      first_lane_ptr == nullptr) {
    return absl::NotFoundError(absl::StrFormat(
        "Current lane path is not loaded entirely, the lane path is: %s",
        lane_path.DebugString()));
  }

  const auto iter = std::find_if_not(
      lane_path.lane_ids().begin(), lane_path.lane_ids().end(),
      [&psmm](mapping::ElementId lane_id) {
        const auto& lane_ptr = psmm.FindCurveLaneByIdOrNull(lane_id);
        return lane_ptr != nullptr;
      });
  std::vector<mapping::ElementId> new_lane_ids(lane_path.lane_ids().begin(),
                                               iter);
  return BuildLanePathFromData(
      mapping::LanePathData(lane_path.start_fraction(),
                            /*end_fraction=*/1.0, std::move(new_lane_ids)),
      psmm);
}

mapping::LanePath ForwardExtendLanePath(const PlannerSemanticMapManager& psmm,
                                        const mapping::LanePath& raw_lane_path,
                                        double extend_len, bool report_issue) {
  if (extend_len <= 0.0) {
    return raw_lane_path;
  }

  mapping::LanePoint start_lp = raw_lane_path.back();
  mapping::LanePath forward_path =
      BuildLanePathFromDataOrReportIssue(mapping::LanePathData(start_lp), psmm);
  while (extend_len > 0.0) {
    ad_byd::planning::LanePtr lane_info_ptr = nullptr;
    if (report_issue) {
      SMM_ASSIGN_LANE_OR_BREAK_ISSUE(lane_info, psmm, start_lp.lane_id());
      lane_info_ptr = std::make_shared<ad_byd::planning::Lane>(lane_info);
    } else {
      SMM_ASSIGN_LANE_OR_BREAK(lane_info, psmm, start_lp.lane_id());
      lane_info_ptr = std::make_shared<ad_byd::planning::Lane>(lane_info);
    }
    CHECK_NOTNULL(lane_info_ptr);
    if (start_lp.fraction() == 1.0) {
      if (lane_info_ptr->next_lane_ids().empty()) break;
      bool valid_next_lane_found = false;
      for (const auto& out_lane_id : lane_info_ptr->next_lane_ids()) {
        const ad_byd::planning::LaneConstPtr out_lane_ptr =
            psmm.map_ptr()->GetLaneById(out_lane_id);
        if (out_lane_ptr && out_lane_ptr->center_line().IsValid()) {
          start_lp = mapping::LanePoint(out_lane_id, 0.0);
          valid_next_lane_found = true;
          break;
        }
      }
      if (!valid_next_lane_found) break;
    } else {
      const double len = lane_info_ptr->curve_length();
      if (len * (1.0 - start_lp.fraction()) > extend_len) {
        const double fraction = extend_len / len + start_lp.fraction();
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), fraction,
                                  {lane_info_ptr->id()}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        break;
      } else {
        extend_len -= len * (1.0 - start_lp.fraction());
        const auto tmp_lane_path = BuildLanePathFromDataOrReportIssue(
            mapping::LanePathData(start_lp.fraction(), 1.0,
                                  {lane_info_ptr->id()}),
            psmm);
        forward_path = ConnectLanePathOrReportIssue(
            forward_path, tmp_lane_path, psmm, /*fail_return_this=*/true);
        start_lp = mapping::LanePoint(start_lp.lane_id(), 1.0);
      }
    }
  }

  return ConnectLanePathOrReportIssue(raw_lane_path, forward_path, psmm,
                                      /*fail_return_this=*/true);
}

std::vector<mapping::LanePath> CollectAllLanePathFromStartLane(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& start_lane,
    double max_lane_length) {
  if (start_lane.length() >= max_lane_length) {
    return {start_lane};
  }

  std::vector<mapping::LanePath> results;
  std::queue<mapping::LanePath> search_queue;
  search_queue.push(start_lane);
  while (!search_queue.empty()) {
    auto lane_path = search_queue.front();
    search_queue.pop();
    SMM_ASSIGN_LANE_OR_CONTINUE(lane_info, psmm, lane_path.lane_ids().back());

    if (lane_info.next_lane_ids().empty()) {
      results.emplace_back(std::move(lane_path));
      continue;
    }

    for (const auto& out_lane_id : lane_info.next_lane_ids()) {
      SMM_ASSIGN_LANE_OR_CONTINUE(out_lane_info, psmm, out_lane_id);
      std::vector<mapping::ElementId> new_lane_ids = lane_path.lane_ids();
      new_lane_ids.push_back(out_lane_id);
      if (lane_path.length() + out_lane_info.curve_length() >=
          max_lane_length) {
        const double end_fraction =
            std::clamp((max_lane_length - lane_path.length()) /
                           out_lane_info.curve_length(),
                       0.0, 1.0);
        results.emplace_back(psmm.map_ptr(), std::move(new_lane_ids),
                             lane_path.front().fraction(), end_fraction);
      } else {
        search_queue.emplace(psmm.map_ptr(), std::move(new_lane_ids),
                             lane_path.front().fraction(),
                             /*end_fraction=*/1.0);
      }
    }
  }

  return results;
}

absl::StatusOr<std::vector<Vec2d>> SampleLanePathByStep(
    const PlannerSemanticMapManager& psmm, const mapping::LanePath& lane_path,
    double step) {
  if (step <= 0.0) {
    return absl::InvalidArgumentError(absl::StrFormat("Invalid step %f", step));
  }

  const double lane_len = lane_path.length();
  const int n = CeilToInt(lane_len / step) + 1;
  std::vector<Vec2d> sample_points;
  sample_points.reserve(n);

  double sample_s = 0.0;
  for (; sample_s <= lane_len; sample_s += step) {
    sample_points.emplace_back(
        ComputeLanePointPos(psmm, lane_path.ArclengthToLanePoint(sample_s)));
  }

  return sample_points;
}

}  // namespace st::planning
