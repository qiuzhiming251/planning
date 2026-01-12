

#include "decider/decision_manager/inferred_object_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "plan_common/maps/map_or_die_macros.h"
#include "plan_common/maps/semantic_map_defs.h"
#include "modules/cnoa_pnc/planning/proto/lane_point.pb.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "modules/cnoa_pnc/planning/proto/piecewise_linear_function.pb.h"
#include "plan_common/plan_common_defs.h"

namespace st::planning {

namespace {

constexpr double kDefaultBraking = -1.0;  // m/s^2
constexpr double kMinSpeedFraction = 0.75;

struct InferredObjectInfo {
  mapping::ElementId lane_id;
  double lane_frac, dist_to_interaction;
  std::string object_id;
};

PiecewiseLinearFunctionDoubleProto CreateSpeedProfile(double init_v, double a,
                                                      double min_v) {
  constexpr double kTimeInterval = 1.0;  // seconds.
  const int n = static_cast<int>(kTrajectoryTimeHorizon / kTimeInterval) + 1;

  PiecewiseLinearFunctionDoubleProto speed_profile;
  speed_profile.mutable_x()->Reserve(n);
  speed_profile.mutable_y()->Reserve(n);
  for (double t = 0.0; t < kTrajectoryTimeHorizon; t += kTimeInterval) {
    speed_profile.add_x(t);
    speed_profile.add_y(std::max(init_v + a * t, min_v));
  }

  return speed_profile;
}

}  // namespace

absl::StatusOr<ConstraintProto::SpeedProfileProto>
BuildInferredObjectConstraint(const PlannerSemanticMapManager& psmm,
                              const SceneOutputProto& scene_reasoning,
                              const mapping::LanePath& lane_path_from_start,
                              double ego_init_v) {
  std::vector<InferredObjectInfo> object_infos;
  for (const auto& inferred_object : scene_reasoning.inferred_objects()) {
    if (inferred_object.infer_source().type_case() !=
        InferredObjectProto::InferSource::TypeCase::kInteractionLane) {
      continue;
    }
    const auto& interaction_info =
        inferred_object.infer_source().interaction_lane();
    object_infos.push_back(InferredObjectInfo{
        .lane_id =
            mapping::ElementId(interaction_info.interaction_point().lane_id()),
        .lane_frac = interaction_info.interaction_point().fraction(),
        .dist_to_interaction = interaction_info.dist_to_interaction(),
        .object_id = inferred_object.object_info().id()});
  }

  for (const auto& seg : lane_path_from_start) {
    const auto lane = psmm.map_ptr()->GetLaneById(seg.lane_id);
    if (!lane) continue;

    const auto& interactions = lane->interactions();
    for (const auto& interaction : interactions) {
      const auto other_lane_id = interaction.other_lane_id;
      const double other_lane_frac = interaction.other_lane_fraction;

      const auto other_lane = psmm.map_ptr()->GetLaneById(other_lane_id);
      if (!other_lane) continue;

      for (const auto& object_info : object_infos) {
        if (object_info.lane_id != other_lane_id ||
            (other_lane_frac - object_info.lane_frac) *
                    other_lane->curve_length() >
                object_info.dist_to_interaction) {
          continue;
        }

        // Use original speed limit.
        const double min_speed = kMinSpeedFraction * lane->speed_limit();
        auto speed_profile =
            CreateSpeedProfile(ego_init_v, kDefaultBraking, min_speed);
        ConstraintProto::SpeedProfileProto speed_profile_constraint;
        *speed_profile_constraint.mutable_vt_upper_constraint() =
            std::move(speed_profile);
        speed_profile_constraint.mutable_source()
            ->mutable_occluded_object()
            ->set_id(object_info.object_id);
        return speed_profile_constraint;
      }
    }
  }

  return absl::NotFoundError("no valid inferred object");
}

}  // namespace st::planning
