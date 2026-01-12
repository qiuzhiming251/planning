

#include "planner/speed_optimizer/speed_finder_flags.h"

DEFINE_bool(planner_print_speed_finder_time_stats, false,
            "Whether to print speed finder time stats.");

DEFINE_bool(planner_send_speed_optimizer_debug, false,
            "Whether to send speed optimizer debug.");

DEFINE_bool(planner_enable_mapping_close_traj, true,
            "Whether to enable mapping moving close st-traj.");

DEFINE_bool(planner_enable_moving_close_traj_speed_limit, false,
            "Whether to enable moving close object speed_limit.");

DEFINE_bool(planner_enable_near_parallel_vehicle_speed_limit, false,
            "Whether to enable near parallel vehicle speed limit.");

DEFINE_bool(planner_enable_right_turn_close_speed_limit, true,
            "Whether to enable right turn close close object speed_limit.");

DEFINE_bool(planner_enable_cross_blind_close_decider, true,
            "Whether to enable planner_enable_cross_blind_close_decider.");

DEFINE_bool(planner_enable_curvature_speed_limit_level, false,
            "Whether to enable planner_enable_curvature_speed_limit_level.");

DEFINE_bool(planner_enable_small_angle_cutin_protective, false,
            "Whether to enable small angle cutin protective st_boundary.");

DEFINE_bool(planner_ignore_curb_out_of_real_range, true,
            "Whether to ignore the curb out of map real range.");

DEFINE_bool(planner_enable_static_object_close_speed_limit, false,
            "Whether to enable close static object speed limit.");

DEFINE_bool(planner_enable_acc_curvature_speed_limit, true,
            "Whether to enable curvature speed limit in acc function.");

DEFINE_bool(planner_enable_acc_pre_st_boundary_modifier, false,
            "Whether to enable pre st-boundary modifier in acc.");

DEFINE_bool(planner_enable_dense_traffic_flow_stop_line, false,
            "Whether to enable dense traffic flow stop line");

DEFINE_bool(planner_use_hybrid_dijkstra_to_search_dp_speed, true,
            "Whether to use hybrid dijkstra to search dp speed.");
