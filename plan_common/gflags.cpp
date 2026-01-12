

#include "plan_common/gflags.h"

DEFINE_int32(ad_byd_planning_log_level, 0,
             "log level, 0 = DEBUG, 1 = INFO, 2 = WARN, 3 = ERROR, 4 = FATAL");
DEFINE_double(planner_max_drop_cruising_speed_limit, 3.0,
              "m/s, delete it later.");
// thread pool numbers
DEFINE_int32(ad_byd_prediction_pool_size, 10,
             "The number of threads in the default thread pool for prediction");
DEFINE_int32(
    ad_byd_city_planner_pool_size, 15,
    "The number of threads in the default thread pool for city planner");

DEFINE_int32(ad_byd_planning_future_wait_timeout, 60,
             "time for future get timeout in second");
DEFINE_double(ad_byd_planning_zero_threshold, 1e-6,
              "zero threshold for ad_byd_planning");

// map
DEFINE_double(ad_byd_planning_map_search_heading_limit, 1.04719755,
              "heading limit for search lane by point");
DEFINE_double(ad_byd_planning_map_minimum_boundary_search_radius, 3.5,
              "minimun search radius for searching lane");
DEFINE_double(ad_byd_planning_map_point_distance_threshold, 0.01,
              "skip map point by distance (between two map points)");
DEFINE_double(ad_byd_planning_map_boundary_gap_threshold, 1.0,
              "distance of gap between two boundaries");
DEFINE_double(ad_byd_planning_boundary_interpolate_dist, 1.0,
              "The distance for boundary interpolate");

DEFINE_double(
    ad_byd_planning_lane_polyline_vector_dist, 2.0,
    "The distance between start point with end point of lane/boundary vector");
DEFINE_uint32(ad_byd_planning_lane_polyline_vector_num, 32,
              "Number of lane/boundary polyline vector size");
DEFINE_uint32(ad_byd_planning_lane_vector_dim, 24, "dimension of lane vector");

DEFINE_double(
    ad_byd_planning_road_boundary_polyline_vector_dist, 2.0,
    "The distance between start point with end point of road_boundary vector");
DEFINE_uint32(ad_byd_planning_road_boundary_polyline_vector_num, 32,
              "Number of road_boundary polyline vector size");
DEFINE_uint32(ad_byd_planning_road_boundary_vector_dim, 16,
              "dimension of road boundary vector");

DEFINE_uint32(ad_byd_planning_polygon_polyline_vector_num, 32,
              "Number of polygon(crosswalk/junction) polyline vector size");
DEFINE_uint32(ad_byd_planning_polygon_polyline_vector_dim, 8,
              "dimension of polygon polyline vector");
DEFINE_double(ad_byd_junction_search_radius, 50,
              "Distance for obstacle junction search radius");
DEFINE_double(ad_byd_converged_entry_speed, 3.0,
              "Converge speed for obs entering junction");
DEFINE_double(ad_byd_lane_sequence_unique_distance, 1.0,
              "Distance between lane sequence");
DEFINE_double(ad_byd_min_juction_lane_length, 2.9, "Min juction lane length");
// map scene
DEFINE_double(ad_byd_planning_forward_lane_sequence_length, 150, "m");
DEFINE_double(ad_byd_planning_backward_lane_sequence_length, 60, "m");

// obstacles
DEFINE_double(ad_byd_caution_obstacle_max_distance, 100,
              "Maximum distance for caution obstacle ranking");
DEFINE_int32(ad_byd_planning_obstacle_lru_cache_num, 100,
             "max number of obstacle lru cache size");
DEFINE_int32(ad_byd_obstacle_lru_cache_num, 100,
             "max number of obstacle lru cache size");
DEFINE_double(ad_byd_current_lane_search_dist, 20,
              "lane search distance for osbtacle current lane");
DEFINE_int32(ad_byd_max_dynamic_obstacle_num, 64,
             "max number of dynamic obstacle to predictive");
DEFINE_string(ad_byd_adc_id, "-1", "autonomous dring car's id");
DEFINE_uint32(ad_byd_planning_max_obstacle_vector_size, 20,
              "max number of obstacle vector size");
DEFINE_uint32(ad_byd_planning_dynamic_obstacle_vector_dim, 24,
              "dynamic obstacle vector dimension");
DEFINE_uint32(ad_byd_planning_static_obstacle_vector_dim, 8,
              "static obstacle vector dimension");

// TODO:Multi-vehicle compatibility
// car
DEFINE_double(ad_byd_adc_L9_length, 5.2,
              "autonomous dring car's length in meters");
DEFINE_double(ad_byd_adc_L9_width, 1.998,
              "autonomous dring car's width in meters");
DEFINE_double(ad_byd_adc_L9_height, 1.8,
              "autonomous dring car's height in meters");
DEFINE_double(ad_byd_adc_L9_front_to_rear, 4.07,
              "autonomous dring car's front to rear axle");

// nn planner network

// tensorrt
DEFINE_string(ad_byd_planning_platform, "x86", "platform");
DEFINE_string(ad_byd_planning_cache_data, "models/", "cache data path");
DEFINE_bool(ad_byd_planning_set_cuda_graph, true, "enable cuda graph");
DEFINE_bool(ad_byd_planning_set_stream_priority, true,
            "enable stream high priority");

// pinned memory
DEFINE_bool(ad_byd_planning_is_pinned_memory, true, "use pinned memory or not");

// gtest
DEFINE_string(ad_byd_planning_test_data, "test/", "test data path");

// prediction reasion
DEFINE_string(ad_byd_prediction_vehicle_reasoning, "PredictionVehicleReasoning",
              "prediction reasoning for vehicle");
DEFINE_string(ad_byd_prediction_cyclist_reasoning, "PredictionCyclistReasoning",
              "prediction reasoning for vehicle");
DEFINE_string(ad_byd_prediction_pedestrian_reasoning,
              "PredictionPedestrianReasoning",
              "prediction reasoning for vehicle");
DEFINE_string(ad_byd_prediction_vehicle_reasoning_config,
              "conf/prediction_vehicle_rules_config.pb.txt",
              "prediction vehicle reasoning config pb");
DEFINE_string(ad_byd_prediction_pedestrian_reasoning_config,
              "conf/prediction_pedestrian_rules_config.pb.txt",
              "prediction vehicle reasoning config pb");
DEFINE_string(ad_byd_prediction_cyclist_reasoning_config,
              "conf/prediction_cyclist_rules_config.pb.txt",
              "prediction vehicle reasoning config pb");
DEFINE_string(ad_byd_prediction_config, "conf/prediction_config.pb.txt",
              "config for prediction module");

DEFINE_bool(ad_byd_prediction_enable_multimodal_prediction, false,
            "enable multimodal prediction");
DEFINE_double(ad_byd_prediction_deterministic_probability, 0.8,
              "deterministic probability for multi modal prediction");

// prediction trajectory
DEFINE_double(ad_byd_prediction_predict_period, 6.0,
              "prediction trajectory prediction period");
DEFINE_double(ad_byd_prediction_trajectory_time_step, 0.1,
              "prediction trajectory time step");

DEFINE_string(ad_byd_city_config, "conf/", "config for city plannner");
DEFINE_string(ad_byd_vehicle_params_path, "vehicle_params.pb.txt",
              "vehicle params file suffix");

DEFINE_double(ad_byd_prediction_predict_extend_period, 8.0,
              "extend prediction trajectory prediction period");
DEFINE_bool(ad_byd_prediction_enbale_trajectory_smooth, true,
            "switch for smooth prediction trajectory");

// lane former
DEFINE_double(ad_byd_lane_former_priori_distance, 350.0,
              "original distance for add priori probability");
DEFINE_double(ad_byd_lane_former_priori_probability, 0.6,
              "original probability for add priori probability");
DEFINE_bool(ad_byd_lane_former_inhibit_eff_lc, false,
            "whether restrain effeciency lane change");
DEFINE_double(ad_byd_lane_former_inhibit_eff_lc_dist_to_junction, 100.0,
              "distance to junction for inhibiting effeciency lc");
DEFINE_double(ad_byd_lane_former_junction_navigation_reset_distance, 200.0,
              "junction navigation reset distance");
DEFINE_double(ad_byd_lane_former_navigation_reset_diff_dis, 100.0,
              "navigation reset diff dis");
DEFINE_int32(ad_byd_lane_former_max_roundabout_exit_range, 120,
             "max exit distance range in roundabout");
DEFINE_int32(ad_byd_lane_former_max_roundabout_enter_range, 250,
             "max enter distance range in roundabout");
DEFINE_int32(ad_byd_lane_former_min_roundabout_exit_range, 20,
             "min exit distance range in roundabout");
DEFINE_int32(ad_byd_lane_former_min_roundabout_enter_range, 50,
             "min enter distance range in roundabout");
DEFINE_bool(ad_byd_handle_special_split, true, "handle_special_split");
DEFINE_bool(planner_enable_u_turn_speed_limit, false,
            "Whether to enable u turn speedlimit");
DEFINE_bool(planner_enable_dynamic_lane_speed_limit, true,
            "Whether to enable dynamic lane speedlimit");
DEFINE_double(planner_override_lane_speed_limit_proportion, 0.0,
              "Modify lane speed limit by a given proportion");
DEFINE_double(planner_prediction_probability_threshold, 0.1,
              "The threshold on probability below which a predicted behavior / "
              "trajectory will be ignored (use at your risk)");
DEFINE_bool(planner_only_use_most_likely_trajectory, false,
            "Only use the most likely trajectory.");

// Lane change style setting.
DEFINE_bool(planner_enable_lc_style_params, true,
            "Whether to enable stylistic planner params in lc.");
DEFINE_bool(planner_enable_occluded_objects_inference, true,
            "Enable reasoning occluded objects according sensor fov.");
DEFINE_bool(
    planner_enable_crosswalk_occluded_objects_inference, false,
    "Enable reasoning occluded objects according sensor fov on crosswalks.");

DEFINE_bool(
    planner_ignore_stalled_objects_on_tl_controlled_leftmost_lane, false,
    " Don't make stalled object decision for  objects on left most lane");

DEFINE_bool(
    planner_enable_un_tl_controlled_intersection_reasoning, true,
    "Enable scene reasoning on un traffic light controlled intersection.");
DEFINE_bool(planner_mapless_status, false, "check whether mapless status");

// Initializer
DEFINE_int32(planner_initializer_debug_level, 0,
             "Initializer debug level: 0: Nothing; 1: Debug info for world "
             "renderer; 2: Debug info for terminal.");
DEFINE_bool(planner_initializer_only_activate_nodes_near_capnet_traj, false,
            "Only activate geometry nodes near the reference traj provided by "
            "captain net for motion search.");
DEFINE_bool(
    planner_initializer_only_activate_nodes_near_refline, false,
    "Only activate geometry nodes near the reference line for motion search.");
DEFINE_int32(
    planner_initializer_max_multi_traj_num, 3,
    "The maximal amount of leading groups that should be considered for "
    "initializer multiple trajectory selection in certain situations.");
DEFINE_bool(planner_initializer_enable_post_evaluation, false,
            "Whether to re-evaluate the top k trajectories from the hand-tuned "
            "initializer params with learned params and select the best as the "
            "final output.");
DEFINE_bool(planner_initializer_enable_clip, true,
            "Whether to enable to clip initializer for faster calculation.");
DEFINE_double(planner_lc_safety_radical_factor, 0.7,
              "factor on radical lane change mode.");
DEFINE_double(planner_lc_safety_normal_factor, 0.8,
              "factor on normal lane change mode.");
DEFINE_double(planner_lc_safety_conservative_factor, 1.0,
              "factor on conservative lane change mode.");

DEFINE_double(planner_initializer_translate_center_line_buffer, 1.5,
              "translate_center_line_buffer");
DEFINE_double(planner_initializer_dist_for_inadvance_avoid, 10.0,
              "dist_for_inadvance_avoid");
DEFINE_double(planner_initializer_dist_for_behind_avoid, 10.0,
              "dist_for_behind_avoid");
DEFINE_double(planner_initializer_hysteresis_control_value, 0.3,
              "hysteresis_control_value");
DEFINE_double(planner_initializer_consider_distance, 100.0,
              "consider_distance");
DEFINE_bool(planner_enable_large_vehicle_avoid, true,
            "planner_enable_large_vehicle_avoid");
DEFINE_bool(planner_enable_push, true, "planner_enable_push");
DEFINE_double(planner_pred_traj_horizon_for_dp_and_ddp, 4.0,
              "planner_pred_traj_horizon_for_dp_and_ddp");
DEFINE_bool(planner_enable_pause_gap_cal, true,
            "Whether to enable gap calculation when in pause state");
// Scheduler.
DEFINE_int32(
    planner_est_parallel_branch_num, 2,
    "The maximal amount of target lane paths to be generated for multi-task "
    "est planner. Note that a borrow-lane branch will be created if this flag "
    "is set to 1, so if only one branch is desired, you should also set the "
    "next flag to false to prohibit lane borrowing.");
DEFINE_bool(planner_est_scheduler_seperate_lc_pause, false,
            "Whether to create a seperate lc pause scheduler branch along with "
            "a lc executing branch.");
DEFINE_bool(planner_est_scheduler_allow_borrow, true,
            "Whether to allow a borrow path boundary if only one target lane "
            "is chosen.");
DEFINE_bool(planner_consider_all_lanes_virtual, false,
            "Whether to consider all lanes\' type as VIRTUAL, for scenarios "
            "with one single lane for mixed use.");
DEFINE_bool(planner_enable_split_tasks, true,
            "Enable split tasks of scheduler or not");
DEFINE_double(planner_extend_traj_behind_ego_by_time, 0.5,
              "Extend trajectory behind car by time");

// Decision
DEFINE_bool(planner_regard_green_flashing_as_green, true,
            "Regard green flashing light as green light.");
DEFINE_bool(planner_enable_miss_navi_stop_line, false,
            "Enable stop vehilce use miss navi stop line");
// Offline DataDumping
DEFINE_bool(
    planner_dumping_initializer_features, false,
    "Whether reading and dumping initializer features' cost of manual driving "
    "trajectory for offline learning");
DEFINE_bool(
    planner_dumping_selector_features, false,
    "Whether reading and dumping selector features' cost of manual driving "
    "trajectory for offline learning");

DEFINE_bool(planner_filter_selector_intention, false,
            "Whether filtering the selector features dumping based on if their "
            "intentions are same as expert. Only work when "
            "planner_dumping_selector_features is true.");

DEFINE_bool(
    planner_dumping_ml_data_in_simulation, false,
    "Whether to use oracle trajectory and dump data for ml to planner_debug");

// Dopt auto tuning.
DEFINE_bool(planner_auto_tuning_mode, false,
            "When auto tuning mode is on, the optimizer will generate and save "
            "one more output which are the accumulated discounted costs for "
            "different cost type.");
DEFINE_bool(planner_optimizer_data_cleaning, false,
            "Whether to do data cleaning/filtering for optimizer and run "
            "snapshot in optimizer data cleaning mode.");
DEFINE_bool(
    planner_update_learned_alphas, false,
    "Whether to use the cost weight alphas learned in auto tuning mode.");
DEFINE_bool(
    planner_update_learned_alphas_except_lane_change, true,
    "Whether to use the cost weight alphas learned in auto tuning mode when "
    "lane change. Only worked when planner_update_learned_alphas is true. So "
    "this will "
    "not influence training process but will influence "
    "evaluation/validation/testing.");
DEFINE_string(planner_traj_opt_params_file_address,
              "./config/compared_trajectory_optimizer_params.pb.txt",
              "The address of the trajectory optimizer params proto file.");
DEFINE_bool(
    planner_compare_different_weight, false,
    "Whether to compare different cost weight learned in auto tuning mode.");
DEFINE_bool(planner_compare_based_on_original_weight, false,
            "Whether to compare the cost weight based on original cost "
            "weight(true) or auto tuned cost weight(false), only works when "
            "planner_compare_different_weight is true.");
DEFINE_string(planner_thread_pool_bind_cores, "2,3", "cores num.");
DEFINE_bool(planner_enable_acc, false, "Enable acc funciton or not");
DEFINE_bool(planner_enable_use_exp_traj, false, "Enable use exp_traj or not");
DEFINE_bool(planner_enable_selector_cost_history, false,
            " Enable use selector cost history");
DEFINE_bool(planner_enable_ld_lite_map, true, "enable ld lite map");
DEFINE_bool(planner_enable_bev_lite_match, false,
            "enable bev and ld lite match");

DEFINE_double(planner_curb_cutoff_buffer, 8.0,
              "dist_for_cutoff_drive_passage_by_curb_buffer");
DEFINE_double(planner_obstacle_cutoff_buffer, 10.0,
              "dist_for_cutoff_drive_passage_by_obstacle_buffer");
