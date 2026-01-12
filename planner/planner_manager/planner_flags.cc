

#include "planner/planner_manager/planner_flags.h"

DEFINE_int32(planner_debug, 0, "How much do you want to debug me?");

DEFINE_int32(planner_thread_pool_size, 3, "Planner thread pool size.");

DEFINE_double(planner_lateral_reset_error, 0.7,
              "Lateral reset error for EstPlanner.");

DEFINE_double(planner_max_allowed_iteration_time, 0.45,
              "The maximum allowed planner iteration time in seconds.");

DEFINE_double(planner_max_localization_transform_delay, 5.0,
              "The max delay allowed to use a localization transform message.");
DEFINE_double(planner_max_perception_delay, 3.0,
              "The max delay allowed for perception message.");
DEFINE_double(planner_max_pose_delay, 0.1,
              "The max delay allowed for pose message in seconds.");

DEFINE_bool(planner_allow_async_in_main_thread, true,
            "Whether to allow async operations in planner\'s main thread.");

DEFINE_bool(planner_allow_multi_threads_in_est, true,
            "Whether to allow multiple threads in est-planner.");

DEFINE_bool(planner_multi_est_in_parallel, true,
            "Whether to run multiple est-planners in parallel.");

DEFINE_bool(
    planner_consider_objects, true,
    "Planner consider objects in perception and prediction. This is "
    "useful for testing mode that does not have perception or prediction.");

DEFINE_double(
    planner_check_trajectory_engage_condition_duration, 1.0,
    "Validate if we can engage in the trajectory's first this amount of time.");

DEFINE_double(
    planner_filter_reflected_object_distance, 0.3,
    "Filter reflected objects that in AV's current position's proximity and "
    "the distance to SDC is less than this value (in meters). Reflected "
    "objects are fake unknown stationary perception objects "
    "that are near SDC body. When this value is less than zero, we "
    "will not filter reflected object.");

DEFINE_int32(planner_lookforward_time_ms, 100,
             "The planner look forward time when computing trajectory.");

DEFINE_bool(planner_open_door_at_route_end, false,
            "Open door when AV is at end of route and fully stopped.");

DEFINE_double(
    planner_door_state_override_waiting_time, 40.0,
    "Don't change door state for so many seconds after door state override. "
    "This value should be less than the duration for the AV to travel from one "
    "station to another. This is used to prevent software open door "
    "immediately after driver closed the door.");

DEFINE_int32(
    planner_task_init_type, 1,
    "0: NOA only; 1:Dynamic switch; 2: ACC only; 3:LCC only; 4:Mapless NOA");

DEFINE_bool(planner_export_all_prediction_to_speed_considered, false,
            "Whether to send all time-aligned predictions to speed considered "
            "prediction. Set it true only for simulation debugging purpose.");

DEFINE_bool(planner_check_aeb, true, "Whether to check emergency stop.");

DEFINE_bool(planner_simplify_debug_proto, false,
            "Only return limited fields in planenr debug proto if true.");

DEFINE_int32(planner_running_platform, 0, "0: IPC; 1: Orin; 2: X9");

// Spacetime flags.
DEFINE_bool(planner_st_traj_mgr_use_all, false,
            "Spacetime trajectory manager uses all objects and trajectories.");

// Selector auto tuning.
DEFINE_bool(planner_use_tuned_selector_params, false,
            "Whether to use the auto tuned selector cost weight params.");
DEFINE_string(
    planner_selector_params_file_address,
    "offboard/planner/ml/models/selector_auto_tuning/selector_params.pb.txt",
    "The address of the auto tuned selector params proto file.");

DEFINE_double(planner_path_start_point_time_diff_limit, 0.5,
              "If relative time of closest point on prev traj from plan "
              "start point larger than this time, set path plan start point to "
              "current close point.");
DEFINE_bool(
    planner_enable_path_start_point_look_ahead, false,
    "Whether to use logic about planner_path_start_point_time_diff_limit.");

DEFINE_int32(planner_runtime_uturn_level, 1,
             "0: Disable three point turn by force; 1: Enable by params; 2: "
             "Enable by force.");

// Planner ml inference.
DEFINE_bool(planner_enable_selector_scoring_net, false,
            "Whether to enable the inference of selector scoring model.");
DEFINE_bool(planner_enable_captain_net, false,
            "Whether to enable the inference of CaptainNet model.");
DEFINE_bool(planner_enable_captain_net_onnx_trt, true,
            "Whether to enable the onnx & trt inference of CaptainNet model.");
DEFINE_bool(planner_use_ml_trajectory_end_to_end, false,
            "Whether to use model generated trajectory as the final output.");
DEFINE_bool(planner_use_ml_trajectory_as_initializer_ref_traj, true,
            "Whether to use model generated trajectory as the reference "
            "trajectory of initializer.");
DEFINE_bool(planner_use_ml_trajectory_as_optimizer_ref_traj, false,
            "Whether to use model generated trajectory as the reference "
            "trajectory of optimizer.");
DEFINE_bool(planner_captain_net_align_traj_based_on_time_for_all_points, true,
            "Whether to align captain net trajectory based on time for "
            "all point.");
DEFINE_bool(
    planner_captain_net_align_traj_based_on_time_for_first_point, false,
    "Whether to align captain net trajectory based on time for first point.");
DEFINE_bool(planner_captain_net_post_process_movability_issue, true,
            "Whether to post process the captain net trajectory if it has "
            "movability issue.");
DEFINE_bool(
    planner_capnet_ref_traj_use_mahalanobis_distance, true,
    "Whether to use mahalanobis distance when using captain net trajectory as "
    "initializer reference trajectory.");
DEFINE_bool(planner_captain_net_use_dkm, true,
            "Whether to post process captain net output as dkm or not.");
DEFINE_bool(
    planner_enable_act_net_speed, true,
    "Whether to use act net speed predictor in planner speed decision.");

DEFINE_bool(planner_rebuild_route_navi_info, false,
            "alway rebuild route navi info in snapshot");

// Planner selector
DEFINE_bool(planner_enable_turn_light_when_open_gap, false,
            "whether to allow turn light when open gap");
DEFINE_bool(planner_enable_lane_change_in_intersection, false,
            "whether to allow lane change in intersection");
DEFINE_bool(planner_enable_cross_solid_boundary, true,
            "whether to allow cross solid boundary");
DEFINE_int32(planner_begin_lane_change_frame, 3,
             "Can not change until lane change trajectory is better than "
             "lane keep trajectory in successive frames");

DEFINE_int32(planner_begin_signal_frame, 5,
             "Turn pre lane change when lane change trajectory is better than "
             "lane keep trajectory in successive frames");
DEFINE_int32(planner_begin_signal_frame_city_noa, 2,
             "Turn pre lane change when lane change trajectory is better than "
             "lane keep trajectory in successive frames");
DEFINE_int32(
    planner_begin_lane_change_frame_progress, 10,
    "Can not change until lane change trajectory is better than "
    "lane keep trajectory in successive frames when lc reason is progress");
DEFINE_int32(
    planner_begin_lane_change_frame_progress_city_noa, 5,
    "Can not change until lane change trajectory is better than "
    "lane keep trajectory in successive frames when lc reason is progress");
DEFINE_int32(planner_begin_change_best_lk_trajectory_frame_city_noa, 2.0,
             "Can not change lane keep trajectory until it better than"
             "last lane keep trajectory in successive frames at city noa");
DEFINE_int32(planner_begin_change_best_lk_trajectory_frame_highway_noa, 3.0,
             "Can not change lane keep trajectory until it better than"
             "last lane keep trajectory in successive frames at highway noa");
DEFINE_double(planner_allow_lc_time_after_activate_selector, 2.0,
              "After activating selector, how long lane change is allowed");
DEFINE_double(planner_allow_lc_time_after_give_up_lc, 15.0,
              "After lc giving up,how long lane change is allowed");

DEFINE_double(planner_allow_opposite_lc_time_after_paddle_lc, 30.0,
              "After paddle lc, how long opposite lane change is allowed");
DEFINE_int32(planner_begin_radical_lane_change_frame, 3,
             "In certain situations, accelerate lane change decision.");
DEFINE_double(planner_max_allow_lc_time_before_give_up, 15.0,
              "How long is maximum allowed duration for a lane change");
DEFINE_double(planner_miss_navi_length, 50.0,
              "length along route less than this, need notice lc miss navi");
DEFINE_bool(
    planner_enable_safe_invariance_supervisor_debug, true,
    "If enabled, safe invariance supervisor will run with rich debug info, "
    "which may slow down the overall performance.");

DEFINE_bool(
    planner_safe_invariance_supervisor_consider_curb, true,
    "If enabled, safe invariance supervisor will consider curb in general.");
DEFINE_bool(planner_safe_invariance_supervisor_path_follow_lane, true,
            "If enabled,  safe invariance supervisor reshape path according to "
            "lane info.");

DEFINE_bool(
    planner_safe_invariance_supervisor_skip_m, false,
    "If enabled, safe invariance supervisor will not run harmony result.");

DEFINE_bool(planner_safe_invariance_problem_consider_kgr, false,
            "If enabled, safe invariance problem respect adv's KGR.");

DEFINE_bool(
    planner_enable_cross_iteration_tf, true,
    "Whether to enable cross-interation smooth coordinate transformation to "
    "compensate for smooth coordinate origin drift.");

// L2 related flag to simulate different situation
DEFINE_bool(planner_force_no_map, false, "Force to not use map.");

DEFINE_double(planner_paddle_lane_change_max_prepare_time, 6.0,
              "Max allowed duration for staying in ALC_PREPARE state after "
              "having triggered paddle lane change.");

// Use previous optimization result as init solution.
DEFINE_bool(
    planner_st_path_planner_lookahead_for_trajectory_optimizer_synchronization,
    true,
    "When enabled, st path planner start point will be further prolonged.");

DEFINE_bool(planner_traj_opt_init_traj_uses_last_optimized_trajectory, true,
            "If enabled, optimizer will try to "
            "use last optimized trajectory as its init solution.");

// Map preprocessing.
DEFINE_bool(planner_force_route_filtered_smm, false,
            "if enabled, filter elements in smm with route.");

DEFINE_bool(lon_decision_enable_use_game_theory, false,
            "if enable, use game theory");

// acc related flags
DEFINE_bool(planner_acc_use_dead_zone, true,
            "Use the steer deadzone to avoid path swing. default true.");
