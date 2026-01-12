

#include "planner/planner_manager/planner_params_builder.h"

#include <cmath>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/message.h"
#include "plan_common/math/fast_math.h"
#include "plan_common/math/util.h"
#include "planner/planner_manager/planner_flags.h"
#include "plan_common/util/file_util.h"
#include "plan_common/util/proto_util.h"
#include "plan_common/util/status_macros.h"
#include "modules/cnoa_pnc/planning/proto/aabox3d.pb.h"
#include "modules/cnoa_pnc/planning/proto/speed_finder_params.pb.h"

namespace st {
namespace planning {

namespace {

std::string GetDefaultParamsFile() {
  return absl::StrCat("planner_default_params.pb.txt");
}

// Protests if any field is missing.
absl::Status ValidateParams(const google::protobuf::Message& params) {
  const google::protobuf::Descriptor* descriptor = params.GetDescriptor();
  const google::protobuf::Reflection* reflection = params.GetReflection();
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (!field->is_optional()) continue;
    if (!reflection->HasField(params, field)) {
      return absl::NotFoundError(
          absl::StrCat("Missing field: ", field->full_name()));
    }
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      RETURN_IF_ERROR(ValidateParams(reflection->GetMessage(params, field)));
    }
  }
  return absl::OkStatus();
}

void ComputeVehicleModelParamsOfPlanner(
    const VehicleGeometryParamsProto& vehicle_geo_params,
    PlannerVehicleModelParamsProto* vehicle_models_params) {
  const double half_width = 0.5 * vehicle_geo_params.width();
  /*----------------------trajectory optimizer model----------------------*/
  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_circles();
  vehicle_models_params->mutable_trajectory_optimizer_vehicle_model_params()
      ->clear_mirror_circles();
  {
    constexpr double kFrontCircleDefualtRadius = 0.3;  // m.
    // Rear axis center.
    const auto rac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    rac->set_dist_to_rac(half_width - vehicle_geo_params.back_edge_to_center());
    rac->set_angle_to_axis(0.0);
    rac->set_radius(half_width);
    rac->set_name("Rear axis center");
    // Front axis center.
    const auto fac = vehicle_models_params
                         ->mutable_trajectory_optimizer_vehicle_model_params()
                         ->add_circles();
    fac->set_dist_to_rac(vehicle_geo_params.front_edge_to_center() -
                         half_width);
    fac->set_angle_to_axis(0.0);
    fac->set_radius(half_width);
    fac->set_name("Front axis center");
    // Middle axis center.
    constexpr double kLengthWidthRateThreshold = 3.0;
    // Add one more circle if vehicle is long.
    if (vehicle_geo_params.length() / vehicle_geo_params.width() >
        kLengthWidthRateThreshold) {
      const auto mfac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac1 =
          2.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          1.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mfac->set_dist_to_rac(dist_to_rac1);
      mfac->set_angle_to_axis(0.0);
      mfac->set_radius(half_width);
      mfac->set_name("Middle Front axis center");
      const auto mrac =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_circles();
      const double dist_to_rac2 =
          1.0 / 3.0 * (vehicle_geo_params.front_edge_to_center() - half_width) +
          2.0 / 3.0 * (half_width - vehicle_geo_params.back_edge_to_center());
      mrac->set_dist_to_rac(dist_to_rac2);
      mrac->set_angle_to_axis(0.0);
      mrac->set_radius(half_width);
      mrac->set_name("Middle Rear axis center");
    } else {
      if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
               .use_less_circles()) {
        const auto mac =
            vehicle_models_params
                ->mutable_trajectory_optimizer_vehicle_model_params()
                ->add_circles();
        mac->set_dist_to_rac(0.5 * (vehicle_geo_params.front_edge_to_center() -
                                    vehicle_geo_params.back_edge_to_center()));
        mac->set_angle_to_axis(0.0);
        mac->set_radius(half_width);
        mac->set_name("Middle axis center");
      }
    }
    // Corners.
    if (!vehicle_models_params->trajectory_optimizer_vehicle_model_params()
             .use_less_circles()) {
      const double dist_to_front_corner =
          Hypot(half_width - kFrontCircleDefualtRadius,
                vehicle_geo_params.front_edge_to_center() -
                    kFrontCircleDefualtRadius);
      const double front_angle =
          st::fast_math::Atan2(half_width - kFrontCircleDefualtRadius,
                               vehicle_geo_params.front_edge_to_center() -
                                   kFrontCircleDefualtRadius);
      // Front left corner.
      const auto flc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      flc->set_dist_to_rac(dist_to_front_corner);
      flc->set_angle_to_axis(front_angle);
      flc->set_radius(kFrontCircleDefualtRadius * 1.414);
      flc->set_name("Front left corner");
      // Front right corner.
      const auto frc = vehicle_models_params
                           ->mutable_trajectory_optimizer_vehicle_model_params()
                           ->add_circles();
      frc->set_dist_to_rac(dist_to_front_corner);
      frc->set_angle_to_axis(-front_angle);
      frc->set_radius(kFrontCircleDefualtRadius * 1.414);
      frc->set_name("Front right corner");
    }
    // Add circles for mirror.
    if (vehicle_geo_params.has_left_mirror() &&
        vehicle_geo_params.has_right_mirror()) {
      // Left mirror.
      const auto left_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      left_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.left_mirror().x(),
                vehicle_geo_params.left_mirror().y()));
      left_mirror_circle->set_angle_to_axis(
          st::fast_math::Atan2(vehicle_geo_params.left_mirror().y(),
                               vehicle_geo_params.left_mirror().x()));
      left_mirror_circle->set_radius(vehicle_geo_params.left_mirror().length() *
                                     0.5);
      left_mirror_circle->set_name("Left mirror");
      // Right mirror.
      const auto right_mirror_circle =
          vehicle_models_params
              ->mutable_trajectory_optimizer_vehicle_model_params()
              ->add_mirror_circles();
      right_mirror_circle->set_dist_to_rac(
          Hypot(vehicle_geo_params.right_mirror().x(),
                vehicle_geo_params.right_mirror().y()));
      right_mirror_circle->set_angle_to_axis(
          st::fast_math::Atan2(vehicle_geo_params.right_mirror().y(),
                               vehicle_geo_params.right_mirror().x()));
      right_mirror_circle->set_radius(
          vehicle_geo_params.right_mirror().length() * 0.5);
      right_mirror_circle->set_name("Right mirror");
    }
  }

  /*--------------------freespace vehicle octagon model--------------------*/
  const auto vehicle_octagon_params =
      vehicle_models_params->mutable_freespace_vehicle_octagon_model_params();
  if (vehicle_geo_params.has_left_mirror() &&
      vehicle_geo_params.has_right_mirror()) {
    vehicle_octagon_params->set_mirror_offset_x(
        vehicle_geo_params.left_mirror().x());
    vehicle_octagon_params->set_mirror_offset_y(
        vehicle_geo_params.left_mirror().y());
    vehicle_octagon_params->set_mirror_radius(
        vehicle_geo_params.left_mirror().length() * 0.5);
    vehicle_octagon_params->set_mirror_height(
        vehicle_geo_params.left_mirror().z() -
        vehicle_geo_params.left_mirror().height() * 0.5);
  } else {
    vehicle_octagon_params->set_consider_mirror(false);
  }
  // Specify vehicle octagon model params from CAD model.
  //   switch (vehicle_model) {
  //     case VEHICLE_LINCOLN_MKZ:
  //     case VEHICLE_LINCOLN_MKZ_AS_PACMOD: {
  //       vehicle_octagon_params->set_front_corner_side_length(0.41);
  //       vehicle_octagon_params->set_rear_corner_side_length(0.35);
  //     } break;
  //     case VEHICLE_MARVELR:
  //     case VEHICLE_MARVELX:
  //     case VEHICLE_MARVELR_NEW: {
  //       vehicle_octagon_params->set_front_corner_side_length(0.44);
  //       vehicle_octagon_params->set_rear_corner_side_length(0.32);
  //     } break;
  //     default:
  //       break;
  //   }

  /*----------------------freespace local smoother model----------------------*/
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->clear_circles();
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->clear_mirror_circles();
  // Copy mirror params from trajectory optimizer.
  vehicle_models_params->mutable_freespace_local_smoother_vehicle_model_params()
      ->mutable_mirror_circles()
      ->CopyFrom(
          vehicle_models_params->trajectory_optimizer_vehicle_model_params()
              .mirror_circles());
  {
    // Corners.
    constexpr double kFrontCircleDefualtRadius = 0.6;  // m.
    constexpr double kRearCircleDefualtRadius = 0.5;   // m.
    double front_circle_radius = kFrontCircleDefualtRadius;
    double rear_circle_radius = kRearCircleDefualtRadius;
    // Specify vehicle circle model params from CAD model.
    // switch (vehicle_model) {
    //   case VEHICLE_LINCOLN_MKZ:
    //   case VEHICLE_LINCOLN_MKZ_AS_PACMOD: {
    //     front_circle_radius = 0.68;
    //     rear_circle_radius = 0.59;
    //   } break;
    //   case VEHICLE_MARVELR:
    //   case VEHICLE_MARVELX:
    //   case VEHICLE_MARVELR_NEW: {
    //     front_circle_radius = 0.73;
    //     rear_circle_radius = 0.54;
    //   } break;
    //   default:
    //     break;
    // }
    // Front corners.
    const double dist_to_front_corner =
        Hypot(half_width - front_circle_radius,
              vehicle_geo_params.front_edge_to_center() - front_circle_radius);
    const double front_angle = st::fast_math::Atan2(
        half_width - front_circle_radius,
        vehicle_geo_params.front_edge_to_center() - front_circle_radius);
    // Front left corner.
    const auto flc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    flc->set_dist_to_rac(dist_to_front_corner);
    flc->set_angle_to_axis(front_angle);
    flc->set_radius(front_circle_radius);
    flc->set_name("Front left corner");
    // Front right corner.
    const auto frc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    frc->set_dist_to_rac(dist_to_front_corner);
    frc->set_angle_to_axis(-front_angle);
    frc->set_radius(front_circle_radius);
    frc->set_name("Front right corner");
    // Rear corners.
    const double dist_to_rear_corner =
        Hypot(half_width - rear_circle_radius,
              vehicle_geo_params.back_edge_to_center() - rear_circle_radius);
    const double rear_angle = st::fast_math::Atan2(
        half_width - rear_circle_radius,
        vehicle_geo_params.back_edge_to_center() - rear_circle_radius);
    // Rear left corner.
    const auto rlc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rlc->set_dist_to_rac(dist_to_rear_corner);
    rlc->set_angle_to_axis(M_PI - rear_angle);
    rlc->set_radius(rear_circle_radius);
    rlc->set_name("Rear left corner");
    // Rear right corner.
    const auto rrc =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rrc->set_dist_to_rac(dist_to_rear_corner);
    rrc->set_angle_to_axis(rear_angle - M_PI);
    rrc->set_radius(rear_circle_radius);
    rrc->set_name("Rear right corner");
    // Rear axis center.
    const auto rac =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    rac->set_dist_to_rac(half_width + rear_circle_radius -
                         vehicle_geo_params.back_edge_to_center());
    rac->set_angle_to_axis(0.0);
    rac->set_radius(half_width);
    rac->set_name("Rear axis center");
    // Front axis center.
    const auto fac =
        vehicle_models_params
            ->mutable_freespace_local_smoother_vehicle_model_params()
            ->add_circles();
    fac->set_dist_to_rac(vehicle_geo_params.front_edge_to_center() -
                         half_width - front_circle_radius);
    fac->set_angle_to_axis(0.0);
    fac->set_radius(half_width);
    fac->set_name("Front axis center");
    // Add one more circle if vehicle is long.
    if (vehicle_geo_params.length() > 2.0 * vehicle_geo_params.width() +
                                          rear_circle_radius +
                                          front_circle_radius) {
      const auto mac =
          vehicle_models_params
              ->mutable_freespace_local_smoother_vehicle_model_params()
              ->add_circles();
      const double dist_to_mac =
          0.5 * (vehicle_geo_params.front_edge_to_center() -
                 vehicle_geo_params.back_edge_to_center() + rear_circle_radius -
                 front_circle_radius);
      mac->set_dist_to_rac(dist_to_mac);
      mac->set_angle_to_axis(0.0);
      mac->set_radius(half_width);
      mac->set_name("Middle axis center");
    }
  }
  vehicle_models_params->set_is_vehicle_bus_model(false);
}

// void FillPlannerFunctionsParams(
//     VehicleModel vehicle_model,
//     PlannerFunctionsParamsProto* planner_functions_params) {
//   switch (vehicle_model) {
//     case VEHICLE_LINCOLN_MKZ:
//     case VEHICLE_LINCOLN_MKZ_AS_PACMOD:
//     case VEHICLE_MARVELR:
//     case VEHICLE_MARVELR_NEW:
//     case VEHICLE_JINLV_MINIBUS:
//       planner_functions_params->set_enable_three_point_turn(true);
//       break;
//     default:
//       planner_functions_params->set_enable_three_point_turn(false);
//       break;
//   }
// }

void FillAlccParamsMissingFieldsWithDefault(
    const PlannerParamsProto& default_planner_params,
    AlccTaskParamsProto* alcc_params) {
  // Fill est planner params.
  FillInMissingFieldsWithDefault(default_planner_params.speed_finder_params(),
                                 alcc_params->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_params(),
      alcc_params->mutable_trajectory_optimizer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.decision_constraint_config(),
      alcc_params->mutable_decision_constraint_config());
  FillInMissingFieldsWithDefault(default_planner_params.initializer_params(),
                                 alcc_params->mutable_initializer_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      alcc_params->mutable_motion_constraint_params());
  FillInMissingFieldsWithDefault(default_planner_params.vehicle_models_params(),
                                 alcc_params->mutable_vehicle_models_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.planner_functions_params(),
      alcc_params->mutable_planner_functions_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.spacetime_planner_object_trajectories_params(),
      alcc_params->mutable_spacetime_planner_object_trajectories_params());

  // Fill lane change style params.
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_radical_params(),
      alcc_params->mutable_speed_finder_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_conservative_params(),
      alcc_params->mutable_speed_finder_lc_conservative_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.speed_finder_lc_normal_params(),
      alcc_params->mutable_speed_finder_lc_normal_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_radical_params(),
      alcc_params->mutable_trajectory_optimizer_lc_radical_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_normal_params(),
      alcc_params->mutable_trajectory_optimizer_lc_normal_params());
  FillInMissingFieldsWithDefault(
      default_planner_params.trajectory_optimizer_lc_conservative_params(),
      alcc_params->mutable_trajectory_optimizer_lc_conservative_params());
}

PlannerParamsProto CreateDefaultParam(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  PlannerParamsProto default_planner_params;
  CHECK(file_util::FileToProto(params_dir + "/planner_default_params.pb.txt",
                               &default_planner_params));

  // Fill default speed finder params into default planner params.
  SpeedFinderParamsProto default_speed_finder_params;
  CHECK(
      file_util::FileToProto(params_dir + "/speed_finder_default_params.pb.txt",
                             &default_speed_finder_params));
  // Fill default trajectory optimizer params into default planner params.
  TrajectoryOptimizerParamsProto default_trajectory_optimizer_params;
  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_default_params.pb.txt",
      &default_trajectory_optimizer_params));
  // Fill est planner.
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_params());
  // Fill fallback planner.
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_fallback_planner_params()
          ->mutable_speed_finder_params());
  // Fill freespace planner.
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_speed_finder_params());
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_speed_finder_params());
  // Fill acc params.
  FillInMissingFieldsWithDefault(default_speed_finder_params,
                                 default_planner_params.mutable_acc_params()
                                     ->mutable_speed_finder_params());
  //   CHECK(file_util::FileToProto(
  //       params_dir + "/acc_req_params.pb.txt",
  //       default_planner_params.mutable_acc_params()->mutable_acc_req_params()));
  // Fill noa params.
  CHECK(file_util::FileToProto(
      params_dir + "/noa_req_params.pb.txt",
      default_planner_params.mutable_noa_params()->mutable_noa_req_params()));
  // Fill default path_finder params into default planner params.
  FreespacePathFinderParamsProto default_path_finder_params;
  CHECK(
      file_util::FileToProto(params_dir + "/path_finder_default_params.pb.txt",
                             &default_path_finder_params));
  FillInMissingFieldsWithDefault(
      default_path_finder_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_path_finder_params());
  FillInMissingFieldsWithDefault(
      default_path_finder_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_path_finder_params());

  // Fill default local_smoother params into default planner params.
  FreespaceLocalSmootherParamsProto default_local_smoother_params;
  CHECK(file_util::FileToProto(
      params_dir + "/local_smoother_default_params.pb.txt",
      &default_local_smoother_params));

  FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_local_smoother_params());
  FillInMissingFieldsWithDefault(
      default_local_smoother_params,
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_local_smoother_params());

  // Copy
  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_parking()
          ->mutable_motion_constraint_params());

  FillInMissingFieldsWithDefault(
      default_planner_params.motion_constraint_params(),
      default_planner_params.mutable_freespace_params_for_driving()
          ->mutable_motion_constraint_params());

  // Fill style params.
  CHECK(file_util::FileToProto(
      params_dir + "/speed_finder_lc_radical_params.pb.txt",
      default_planner_params.mutable_speed_finder_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_lc_radical_params());

  CHECK(file_util::FileToProto(
      params_dir + "/speed_finder_lc_conservative_params.pb.txt",
      default_planner_params.mutable_speed_finder_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_lc_conservative_params());

  CHECK(file_util::FileToProto(
      params_dir + "/speed_finder_lc_normal_params.pb.txt",
      default_planner_params.mutable_speed_finder_lc_normal_params()));
  FillInMissingFieldsWithDefault(
      default_speed_finder_params,
      default_planner_params.mutable_speed_finder_lc_normal_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_radical_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_radical_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_normal_params.pb.txt",
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params.mutable_trajectory_optimizer_lc_normal_params());

  CHECK(file_util::FileToProto(
      params_dir + "/trajectory_optimizer_lc_conservative_params.pb.txt",
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params()));
  FillInMissingFieldsWithDefault(
      default_trajectory_optimizer_params,
      default_planner_params
          .mutable_trajectory_optimizer_lc_conservative_params());

  // Fill vehicle model params.
  ComputeVehicleModelParamsOfPlanner(
      vehicle_geo_params,
      default_planner_params.mutable_vehicle_models_params());

  // Fill planner functions params.This should determined by params or HMI in
  // the future.
  //   FillPlannerFunctionsParams(
  //       vehicle_model,
  //       default_planner_params.mutable_planner_functions_params());

  // Fill alcc params.
  FillAlccParamsMissingFieldsWithDefault(
      default_planner_params, default_planner_params.mutable_alcc_params());
  return default_planner_params;
}

}  // namespace

absl::StatusOr<PlannerParamsProto> BuildPlannerParams(
    const std::string& params_dir,
    const VehicleGeometryParamsProto& vehicle_geo_params) {
  PlannerParamsProto run_planner_params;
  const bool load_file_success = file_util::FileToProto(
      params_dir + "/planner_default_params.pb.txt", &run_planner_params);

  if (!load_file_success) {
    return absl::NotFoundError(
        absl::StrCat("Cannot find param file in dir " + params_dir));
  }

  PlannerParamsProto planner_params;
  FillInMissingFieldsWithDefault(run_planner_params, &planner_params);

  // Fill with default planner params.
  const auto default_planner_params =
      CreateDefaultParam(params_dir, vehicle_geo_params);

  FillInMissingFieldsWithDefault(default_planner_params, &planner_params);

  // Check that all fields are set.
  RETURN_IF_ERROR(ValidateParams(planner_params));
  return planner_params;
}

}  // namespace planning
}  // namespace st
