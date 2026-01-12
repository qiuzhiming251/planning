

#ifndef ADBYD_PLANNING_MAP_MAP_DEF_H
#define ADBYD_PLANNING_MAP_MAP_DEF_H

#include <string>
#include <vector>
#include <optional>

#include "plan_common/type_def.h"

namespace ad_byd {
namespace planning {
enum FeatureType {
  FEATURETYPE_LANE = 0,
  FEATURETYPE_LANEBOUNDARY = 1,
  FEATURETYPE_ROADBOUNDARY = 2,
  FEATURETYPE_STOPLINE = 3,
  FEATURETYPE_JUNCTION = 4,
  FEATURETYPE_CROSSWALK = 5,
  FEATURETYPE_SPEEDBUMP = 6,
  FEATURETYPE_CLEARAREA = 7,
};

enum LineColor {
  COLOR_UNKNOWN = 0,
  COLOR_WHITE = 1,
  COLOR_YELLOW = 2,
  COLOR_ORANGE = 3,
  COLOR_BLUE = 4,
  COLOR_GREEN = 5,
  COLOR_GRAY = 6,
  LEFT_GRAY_RIGHT_YELLOW = 7,
  LEFT_YELLOW_RIGHT_WHITE = 8,
  LEFT_WHITE_RIGHT_YELLOW = 9,
  OTHER = 10
};

enum LineType {
  UNKNOWN = 0,
  SOLID = 1,
  DASHED = 2,
  SOLID_SOLID = 3,
  DASHED_DASHED = 4,
  SOLID_DASHED = 5,
  DASHED_SOLID = 6,
  VIRTUAL_LANE = 7,
  SHADED_AREA = 8,
  VIRTUAL_JUNCTION = 9,
  RAMP = 10,        // mapping BEV static : RAMP =15, //  短粗线
  FISH_LINE = 11,   // mapping BEV static : FISH_LINE = 16 //鱼骨线
  CURB_LINE = 12,   // 地图收费站通道专用车道线
  FISH_SOLID = 13,  // 鱼骨线，非鱼骨部分为实线
  FISH_DASH = 14    // 鱼骨线，非鱼骨部分为虚线
};

enum BoundaryType {
  UNKNOWN_BOUNDARY = 0,
  LANELINE = 1,
  CURB = 2,
  CENTER = 3,
  GUARDRAIL = 4,
  CONCRETE_BARRIER = 5,
  FENCE = 6,
  WALL = 7,
  CANOPY = 8,
  PAVE = 9,
  DITCH = 10,
  PUNCHEON = 11,
  VIRTUAL = 12,
  LIDAR_UU = 13,
  OCC_UU = 14,
  OCC_CURB = 15,
  OCC_VEGETATION = 16,
  OCC_CONJECTURE = 17
};

enum LaneType {
  LANE_UNKNOWN = 0,
  LANE_NORMAL = 1,
  LANE_ACC = 2,
  LANE_DEC = 3,
  LANE_RAMP = 4,
  LANE_EMERGENCY = 5,
  LANE_ACC_DCC = 6,
  LANE_BUS_NORMAL = 7,
  LANE_HOV_NORMAL = 8,
  LANE_NON_MOTOR = 9,
  LANE_LEFT_WAIT = 10,
  LANE_VIRTUAL_COMMON = 11,
  LANE_VIRTUAL_JUNCTION = 12,
  LANE_ROUND_ABOUT = 13,
  LANE_REVERSIBLE = 16,  // 可变车道
  LANE_VARIABLE_TURN = 17,
  LANE_HARBOR_STOP = 18,                 // 港湾停靠站
  LANE_ENTRY = 19,                       // 入口
  LANE_EXIT = 20,                        // 出口
  LANE_DIVERSION = 21,                   // 导流区车道
  LANE_U_TURN_LANE = 22,                 // 掉头车道
  LANE_RIGHT_TURN_LANE = 23,             // 右转专用车道
  LANE_RIGHT_TURN_AREA = 24,             // 右转等待车道
  LANE_U_TURN_AREA = 25,                 // 掉头等待车道
  LANE_NO_TURN_AREA = 26,                // 直行等待车道
  LANE_VIRTUAL_CONNECTED_LANE = 27,      // 虚拟连接车道
  LANE_PARKING = 28,                     // 停车车道
  LANE_TOLLBOOTH = 29,                   // 收费站车道
  LANE_OBSTACLE = 30,                    // 障碍物
  LANE_MIXED_TOLL = 31,                  // 混合收费车道
  LANE_REVERSIBLE_CONNECTION_LANE = 32,  //潮汐连接路
  LANE_ENTRANCE_OR_EXIT_LANE = 33,       //主辅路出入口
  LANE_LEFT_TURN_LANE = 34,              //左转专用道
  LANE_CHECKPOINT_LANE = 35,             //检查站车道
  LANE_USE_THE_LEFT_TURN_LANE = 36,      //借道左转车道
  LANE_BRT = 37                          //公交专用道
};

enum class CompositeTurnType {
  NORMAL_TURN = 0,
  LEFT_STRAIGHT = 1,
  STRAIGHT_RIGHT = 2,
  LEFT_RIGHT = 3,
  LEFT_STRAIGHT_RIGHT = 4
};

enum LightStatus {
  NONE_LIGHT = 0,
  GREEN_LIGHT = 1,
  YELLOW_LIGHT = 2,
  RED_LIGHT = 3,
  UNKNOWN_LIGHT = 4,
  YELLOW_BLINKING = 5,
  FAIL_DETECTION = 6,
  BLOCK_FAIL = 7,
  GREEN_BLINKING = 8,
  Blurring_Mode = 9
};

enum TrafficSetReason {
  UNKNOWN_STATE = 0,
  STAY_PREV = 1,
  SET_DEFAULT_OBJ = 2,
  NEW_ARROW_LIGHT = 3,
  NEW_CIRCLE_LIGHT = 4,
  NEW_UNKNOWN_GREEN = 5,
  NEW_UNKNOWN_OTHER = 6,
  NEW_BLURRING_COLOR = 7,
  NEW_BLURRING_SHAPE = 8
};

enum StopLineReason {
  REASON_NONE = 0,
  REASON_LIGHT_RED = 1,
  REASON_LIGHT_YELLOW = 2,
  REASON_CONFIRM = 3,
  REASON_GREENBLINK = 4,
  REASON_Blurring_Mode = 5
};

enum StopLineInterface {
  STOP_LINE_NONE = 0,
  STOP_LINE_RED = 1,
  STOP_LINE_YELLOW = 2,
  STOP_LINE_CONFIRM = 3,
  STOP_LINE_UNKNOWN = 4,
  STOP_LINE_FAIL_DETECT = 5,
  STOP_LINE_LCC_TURN = 6,
  STOP_LINE_T_JUNCTION = 10
};

struct FsdTrafficLightDeciderInfo {
  double dist_to_stopline = std::numeric_limits<double>::max();
  double dist_to_leftwait_stopline = std::numeric_limits<double>::max();
  bool ego_is_in_leftwait = false;
  bool first_virtual_lane_is_leftwait = false;
  uint64_t first_virtual_lane = 0;
  uint64_t focus_lane = 0;
  uint64_t left_wait_lane = 0;
  LightStatus first_virtual_lane_light = NONE_LIGHT;
  LightStatus focus_lane_light = NONE_LIGHT;
  LightStatus left_wait_lane_light = NONE_LIGHT;
  uint32_t light_countdown = 0;
  uint32_t stopline_angle_flag = 0;
  TurnType light_turn_type = NO_TURN;
};

struct TrafficLightStatus {
  uint64_t lane_id;
  std::optional<uint64_t> junction_id = std::nullopt;
  LightStatus light_status = LightStatus::NONE_LIGHT;
  bool stop_line = false;
  bool is_left_wait_lane = false;
  uint32_t light_countdown = 0;
  uint32_t stopline_angle_flag = 0;
  TurnType light_turn_type = NO_TURN;
  TrafficSetReason traffic_set_reason = TrafficSetReason::UNKNOWN_STATE;
  bool pre_has_light = false;
};

using TrafficLightStatusMap = std::unordered_map<uint64_t, TrafficLightStatus>;

enum MergeTopology {
  TOPOLOGY_MERGE_NONE = 0,
  TOPOLOGY_MERGE_LEFT = 1,   // merge to left, ego is on right of target lane
  TOPOLOGY_MERGE_RIGHT = 2,  // merge to right, ego is on left of target lane
  TOPOLOGY_TO_BE_MERGED = 3,
  TOPOLOGY_MERGE_UNKNOWN = 4
};

enum SplitTopology {
  TOPOLOGY_SPLIT_NONE = 0,
  TOPOLOGY_SPLIT_LEFT = 1,   // split to left
  TOPOLOGY_SPLIT_RIGHT = 2,  // split to right
  TOPOLOGY_SPLIT_UNKNOWN = 3
};

enum NoneOddType { TYPE_NONE = 0, TYPE_TOLL = 1, TYPE_CONSTRUCTION = 2 };

enum MapRoadClass {
  RC_UNKNOWN = 0,
  RC_EXPRESSWAY = 1,
  RC_URBAN_EXPRESSWAY = 2,
  RC_NATION_ROAD = 3,
  RC_PROVINCE_ROAD = 4,
  RC_RESERVE = 5,
  RC_COUNTRY_ROAD = 6,
  RC_TOWN_ROAD = 7,
  RC_SPECIAL_ROAD = 8,  // 特服道路
  RC_WALK_ROAD = 9,
  RC_PEOPLE_FERRY = 10,  // 人渡
  RC_FERRY = 11,         // 轮渡
  RC_OTHERS = 99
};

enum PoiType {
  Poi_NaviEnd = 0,
  Poi_Split = 1,
  Poi_To_Be_Merged = 2,
  Poi_Merge = 3
};

enum XRoadType {
  XRoadType_Ramp = 0,
  XRoadType_NoneOdd = 1,
  XRoadType_Tunnel = 2
};

enum ImpassableAeraType {
  UNKNOWN_KIND = 0,          // 未调查
  FLOWERBED = 1,             // 花坛
  SENTRY_BOX = 2,            // 岗亭
  PHYSICAL_SAFE_ISLAND = 3,  // 物理边界安全岛
  LINEAR_SAFE_ISLAND = 4     // 线型安全岛
};
enum StopLineType {
  STOPLINETYPE_UNKNOWN = 0,
  STOPLINETYPE_STRAIGHT = 1,  // 直行停止线
  STOPLINETYPE_LEFT_WAIT = 2  // 左转待转停止线
};

enum MergeSource {
  MERGE_SOURCE_UNKNOWN = 0,
  MERGE_SOURCE_BEV = 1,
  MERGE_SOURCE_LD = 2,
  MERGE_SOURCE_SD = 3
};

enum RestrictedType {
  RT_NONE = 0, // 无
  RT_IDAL = 1, // 潮汐车道
  RT_BRT = 2, // 公交专用道
  RT_HOV = 3, // HOV车道
  RT_TAXI = 4, // 出租车专用道
  RT_OTHER = 5, // 其他车道

  RT_HVL = 7, // 危化品车辆专用道
  RT_ADL = 8, // 救护车专用道
  RT_SHARED_LEFT_TURN = 9 //借道左转车道
};

struct RestrictedInfo {
    RestrictedType restricted_type;
    bool is_passable;
    uint32_t passable_env_state; //车道的通行状态，0：未知 ；1：可通行； 2：不可通行
};

struct RoadBoundaryInfo {
  uint64_t id;
  double width = 0.0;
  std::vector<Point2d> points;
  BoundaryType boundary_type = UNKNOWN_BOUNDARY;
};

struct RoadBoundaryType {
  double s = 0.0;
  double width = 0.0;
  BoundaryType boundary_type = UNKNOWN_BOUNDARY;
};

struct LaneBoundarySegmentInfo {
  std::string id;
  std::vector<Point2d> points;
  LineType line_type = UNKNOWN;
  LineColor line_color = COLOR_UNKNOWN;
};

struct LaneBoundaryType {
  double s = 0.0;  // end_s
  LineType line_type = UNKNOWN;
  LineColor line_color = COLOR_UNKNOWN;
};

struct LaneBoundaryInfo {
  uint64_t id;
  std::vector<Point2d> points;
  LaneBoundaryType boundary_type;
};

struct LaneMergeInfo {
  bool valid = false;
  MergeSource merge_source = MERGE_SOURCE_UNKNOWN;
  double dist_to_merge = 0.0;
};

struct LaneInfo {
  uint64_t id;
  uint64_t section_id;
  uint64_t junction_id;
  uint64_t left_lane_id;
  uint64_t right_lane_id;
  std::vector<uint64_t> next_lane_ids;
  std::vector<uint64_t> left_lane_boundary_ids;
  std::vector<uint64_t> right_lane_boundary_ids;
  std::vector<uint64_t> left_road_boundary_ids;
  std::vector<uint64_t> right_road_boundary_ids;
  LaneType type = LANE_NORMAL;
  NoneOddType none_odd_type = TYPE_NONE;
  TurnType turn_type = NO_TURN;
  LightStatus light_status = NONE_LIGHT;
  TrafficSetReason traffic_set_reason = TrafficSetReason::UNKNOWN_STATE;
  bool pre_has_light = false;
  SplitTopology split_topology = TOPOLOGY_SPLIT_NONE;
  MergeTopology merge_topology = TOPOLOGY_MERGE_NONE;
  LaneMergeInfo lane_merge_info;
  RestrictedInfo restricted_info;
  std::vector<Point2d> points;
  // std::vector<double> points_mse;
  double length = 0.0;
  double speed_limit = 135 / 3.6;
  bool is_virtual = false;
  bool is_navigation = false;
  bool is_virtual_navigation = false;
  bool stop_line = false;
  std::vector<uint64_t> cross_walks;
  std::vector<uint64_t> speed_bumps;
  std::vector<uint64_t> parking_spaces;
  std::vector<uint64_t> clear_areas;
  std::vector<uint64_t> traffic_stop_lines;
  int32_t lane_operation_type = 0;
  std::vector<double> coeffs;
  int32_t arrow_type = 0;
  uint32_t light_countdown = 0;
  bool is_exp_traj{false};
  bool is_split_topo_modify_{false};
  bool is_merge_topo_modify_{false};
  uint32_t stopline_angle_flag;
  bool is_acc_adj_lane = {false};
  double merge_start_dis = 0.0;
  double merge_end_dis = 0.0;
};
struct SectionInfo {
  uint64_t id;
  double length = 0.0;
  NoneOddType none_odd_type = TYPE_NONE;
  std::vector<uint64_t> lane_ids;
  uint64_t navi_priority_lane_id;
  MapRoadClass road_class;
  bool is_neighbor_extend = false;
  std::vector<int> extend_sections_index;
};
struct PathExtend {
  int64_t from_section_idx = -1;
  uint64_t from_section_id = 0;
  uint64_t to_section_idx = -1;
  std::vector<SectionInfo> extend_sections;
};
struct NaviPosition {
  uint64_t section_id;
  double s_offset;
};
struct RouteInfo {
  uint64_t id;
  NaviPosition navi_start;
  NaviPosition navi_end;
  std::vector<SectionInfo> sections;
  std::vector<PathExtend> extend_sections_vec;
};
struct StopLineInfo {
  uint64_t id;
  std::vector<Point2d> points;
  StopLineType type;
  LightStatus light_type;
  int8_t sub_type;
  int8_t virtual_type;
};
struct JunctionInfo {
  uint64_t id;
  std::vector<Point2d> points;
};
struct CrossWalkInfo {
  uint64_t id;
  std::vector<Point2d> points;
};
struct SpeedBumpInfo {
  uint64_t id;
  std::vector<Point2d> points;
};
struct ParkingSpaceInfo {
  std::string id;
  std::vector<Point2d> points;
};
struct ClearAreaInfo {
  uint64_t id;
  ImpassableAeraType type;
  std::vector<Point2d> points;
};

struct V2RoadClass {
  double start_s = 0.0;
  double end_s = 0.0;
  enum V2RoadClassType {
    UNKNOWN_ROAD = 0,
    HIGH_WAY_ROAD = 1,
    EXPRESS_WAY_ROAD = 2,
    NATIOANL_ROAD = 3,
    PROVINCIAL_ROAD = 4,
    MAIN_ROAD = 5,
    SUB_ROAD = 6
  };
  V2RoadClassType type = UNKNOWN_ROAD;
};

struct V2TrafficFlow {
  double start_s = 0.0;
  double end_s = 0.0;
  enum V2TrafficFlowType {
    UNKNOWN_FLOW = 0,
    SMOOTH_FLOW = 1,
    SLOW_FLOW = 2,
    JAMMED_FLOW = 3,
    SERVERE_JAMMED_FLOW = 4,
    NO_FLOW = 5
  };
  V2TrafficFlowType type = UNKNOWN_FLOW;
};

struct RoadInfo {
  int road_class = 0;
  int lane_num = 0;
};

struct PassInfo {
  int curr_index = -1;
  int index_num = -1;
};

struct V2TurnInfo {
  uint64_t id = 0;
  bool is_valid = false;
  enum V2TurnType {
    UNKNOWN = 0,
    LEFT = 1,
    RIGHT = 2,
    STRAIGHT = 3,
    U_TURN_LEFT = 4,
    U_TURN_RIGHT = 5,
    MERGE_LEFT = 6,
    MERGE_RIGHT = 7,
    RAMP_LEFT = 11,
    RAMP_RIGHT = 12,
    RAMP_STRAIGHT = 13,
    RAMP_U_TURN_LEFT = 14,
    RAMP_U_TURN_RIGHT = 15,
  };
  V2TurnType turn_type = UNKNOWN;
  enum V2DetailTurnType {
    NONE = 0,
    TURN_LEFT = 2,         // 左转图标
    TURN_RIGHT = 3,        // 右转图标
    SLIGHT_LEFT = 4,       // 左前方图标
    SLIGHT_RIGHT = 5,      // 右前方图标
    TURN_HARD_LEFT = 6,    // 左后方图标
    TURN_HARD_RIGHT = 7,   // 右后方图标
    UTURN = 8,             // 左转掉头图标 (无在线图标)
    CONTINUE = 9,          // 直行图标
    TURN_RIGHT_ONLY = 10,  // 右转专用道
    UTURN_RIGHT = 19,      // 右转掉头图标，左侧通行地区的掉头
    LEFT_MERGE = 65,       // 靠左图标，1076b新增
    RIGHT_MERGE = 66,      // 靠右图标，1076b新增
  };
  V2DetailTurnType detail_turn_type = NONE;
  double dist = 0.0;
  uint32_t original_dist = 0;
  RoadInfo before_turn;
  RoadInfo after_turn;
  std::vector<std::string> infos;
  PassInfo straight_pass_info;
};

struct V2Curvature {
  double curvature;  // 曲率半径
  double distance;   // 距离
};

struct NonODDInfo {
  std::string id;
  uint64_t reason;
  double dist;
};

struct EHPV2Info {
  bool is_valid = false;
  bool has_navigation = false;
  double dist_to_ramp = 10000.0;
  double dist_to_toll = 10000.0;
  double dist_to_tunnel = 10000.0;
  double dist_to_subpath = 10000.0;
  double dist_to_route_split = 10000.0;
  std::vector<V2RoadClass> road_class;
  std::vector<V2TrafficFlow> traffic_flow;
  std::vector<V2TurnInfo> turn_info;
  std::vector<V2TurnInfo> pnp_turn_info;
  std::vector<V2Curvature> v2_curvatures;
  std::vector<NonODDInfo> v2_nodd_info;
};

struct ExpTrajectoryInfo {
  uint64_t id;
  uint64_t lane_id;
  uint64_t start_lane_id;
  uint64_t end_lane_id;
  std::vector<uint64_t> relative_lane_id;
  std::vector<Point2d> points;
};

struct MapInfo {
  MapType type = BEV_MAP;  // 没有真正赋值，慎用
  MapSubType sub_type;
  double timestamp = 0.0;
  int64_t seq = 0;
  bool is_on_highway = false;
  std::vector<LaneInfo> all_lanes_vec;
  std::vector<LaneBoundaryInfo> all_lane_boundaries_vec;
  std::vector<RoadBoundaryInfo> all_road_boundaries_vec;
  std::vector<StopLineInfo> all_stop_lines_vec;
  std::vector<JunctionInfo> all_junctions_vec;
  std::vector<CrossWalkInfo> all_cross_walks_vec;
  std::vector<SpeedBumpInfo> all_speed_bumps_vec;
  std::vector<ParkingSpaceInfo> all_parking_spaces_vec;
  std::vector<ClearAreaInfo> all_clear_areas_vec;
  std::vector<ExpTrajectoryInfo> all_exp_trajectories_vec;
  RouteInfo route;
  EHPV2Info v2_info;
};

struct LdLiteLaneInfo {
  uint64_t id;
  uint64_t lane_group_id;
  uint32_t lane_seq;  //车道序号，从左到右，从1开始
  LaneType type = LANE_NORMAL;
  LiteTurnType turn_type = LITE_NO_TURN;
  std::vector<uint64_t> next_lanes;
  std::vector<uint64_t> previous_lanes;
  double length;
  double navi_distance = 0.0;
  double min_navi_distance = 0.0;
};

struct LdLiteLaneGroupInfo {
  uint64_t id;
  uint32_t lane_num;
  std::vector<LdLiteLaneInfo> lane_info;
  std::vector<uint64_t> next_lane_group;
  std::vector<uint64_t> previous_lane_group;
  double length;  // unit: m
};

struct LdLiteLaneGroupIndex {
  uint64_t id;
  double start_range;  // uint: m
  double end_range;
};

struct LdLiteSectionInfo {
  uint64_t id;
  double length;
  std::vector<LdLiteLaneGroupIndex> lane_group_idx;
};

struct LdLiteNaviPosition {
  uint64_t section_id;
  uint32_t lane_index;
  double s_offset;
};

struct LdLiteMapInfo {
  std::vector<LdLiteLaneGroupInfo> lane_group_info;
  std::vector<LdLiteSectionInfo> mpp_info;
  LdLiteNaviPosition navi_start;
  int64_t seq = 0;
};

}  // namespace planning
}  // namespace ad_byd

#endif
