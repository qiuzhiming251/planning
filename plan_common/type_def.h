

#ifndef AD_BYD_COMMON_TYPE_DEF_H
#define AD_BYD_COMMON_TYPE_DEF_H
#include <cassert>
#include <cfloat>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "constants.h"
#include "plan_common/math/vec2d.h"
#include "plan_common/math/vec3d.h"

namespace ad_byd {
namespace planning {

enum TurnType { NO_TURN = 0, LEFT_TURN = 1, RIGHT_TURN = 2, U_TURN = 3 };

enum LiteTurnType {
  LITE_NO_TURN = 0,
  LITE_LEFT_TURN = 1,
  LITE_RIGHT_TURN = 2,
  LITE_U_TURN = 3,
  LITE_STRAIGHT_AND_LEFT_TURN = 4,
  LITE_STRAIGHT_AND_RIGHT_TURN = 5,
  LITE_STRAIGHT_AND_U_TURN = 6,
  LITE_LEFT_TURN_AND_U_TURN = 7,
  LITE_RIGHT_TURN_AND_U_TURN = 8,
  LITE_STRAIGHT_AND_LEFT_TURN_AND_RIGHT_TURN = 9,
  LITE_LEFT_TURN_AND_RIGHT_TURN = 10,
  LITE_STRAIGHT_AND_LEFT_TURN_AND_U_TURN = 11,
  LITE_STRAIGHT_AND_RIGHT_TURN_AND_U_TURN = 12,
  LITE_LEFT_TURN_AND_RIGHT_TURN_AND_U_TURN = 13,
  LITE_STRAIGHT_AND_LEFT_AND_RIGHT_AND_U_TURN = 14
};

enum RoadClass {
  ROADCLASSES_UNKNOWN = 0,
  HIGH_WAY = 1,
  EXPRESS_WAY = 2,
  NATIONAL_ROAD = 3,
  PROVINCIAL_ROAD = 4,
  MAIN_ROAD = 5,
  SUB_ROAD = 6
};

enum class RoadType {
  ROADTYPE_UNKNOWN = 0,
  FREEWAY = 1,
  MULTIPLE_CARRIAGEWAY = 2,
  SINGLE_CARRIAGEWAY = 3,
  ROUNDABOUT_CIRCLE = 4,
  PARRALEL_ROAD = 8,
  SLIP_RAMP_ON_FREEWAY = 9,
  SLIP_RAMP_ON_NONFREEWAY = 10,
  SERVICE_ROAD = 11,
  ENTRANCE_EXIT_OF_CAR_PARK = 12,
  ENTRANCE_EXIT_OF_SERVICE = 13,
  PEDESTRIAN_ZONE = 14
};

enum PlanResult {
  PLAN_OK = 0,
  PLAN_FAIL = 1,
  PLAN_FAIL_POINTER = 2,
  PLAN_FAIL_TIMESTAMP = 3,
  PLAN_FAIL_MAP_LOCATION = 4,
  PLAN_FAIL_REF_LINE = 5,
  PLAN_FAIL_PATH = 6,
  PLAN_FAIL_SPEED = 7,
  PLAN_FAIL_TRAJECTORY = 8,
  PLAN_FAIL_BOUNDARY = 9,

  // MAP 10~29
  PLAN_MSG_MAP_TIMEOUT = 10,
  PLAN_MSG_MAP_DELAY = 11,
  PLAN_FAIL_MAP_COORDINATE_MISMATCH = 12,
  PLAN_FAIL_NO_MAP = 13,
  PLAN_FAIL_CURRENT_LANES = 14,
  PLAN_FAIL_TARGET_LANESEQ_SHORT = 15,
  PLAN_FAIL_TARGET_SAME_LANES = 16,
  PLAN_FAIL_TARGET_NEAREST_LANE = 17,
  PLAN_FAIL_TARGET_EXTEND_LANES = 18,
  PLAN_FAIL_NO_LANE_BOUNDARY = 19,
  PLAN_FAIL_CURRENT_NON_MOTOR = 20,
  PLAN_FAIL_CURRENT_NEAREST_LANE = 21,

  // OBSTACLE 30~39
  PLAN_MSG_OBSTACLE_TIMEOUT = 30,
  PLAN_MSG_OBSTACLE_DELAY = 31,
  PLAN_MSG_OBSTACLE_TRANSFORM_FAIL = 32,
  PLAN_MSG_OBSTACLE_COORDINATE_MISMATCH = 33,
  PLAN_MSG_OBSTACLE_STAMP_NONALIGNED = 34,

  // ODOM/LOCATION 40~49
  PLAN_MSG_LOCATION_TIMEOUT = 40,
  PLAN_MSG_ODOMETRY_TIMEOUT = 41,
  PLAN_MSG_LOCATION_DELAY = 42,
  PLAN_MSG_ODOMETRY_DELAY = 43,

  // VEHICLE_STATE 50~59
  PLAN_MSG_VEHICLE_TIMEOUT = 50,
  PLAN_MSG_VEHICLE_DELAY = 51,
  PLAN_MSG_VEHICLE_TRANSFORM_FAIL = 52,
  PLAN_MSG_VMC_TIMEOUT = 53,

  // CHART_BEHAVIOR 60~69
  PLAN_MSG_BEHAVIOR_TIMEOUT = 60,
  PLAN_MSG_BEHAVIOR_DELAY = 61,

  // BASIC_MODEL 100~109
  PLAN_FAIL_START_POINT = 100,

  // TARGET_LANE 110~119
  PLAN_FAIL_TARGET_LANESEQ_MAP = 110,

  // REF_LINE 120~129
  PLAN_FAIL_REF_LINE_POINTS_NUM_SHORT = 119,
  PLAN_FAIL_REF_LINE_NO_LANES = 120,
  PLAN_FAIL_REF_LINE_SPIRAL_BUILDER = 121,
  PLAN_FAIL_REF_LINE_SPIRAL_SOLVER = 122,
  PLAN_FAIL_REF_LINE_LINEAR_BUILDER = 123,
  PLAN_FAIL_REF_LINE_LINEAR_SOLVER = 124,
  PLAN_FAIL_REF_LINE_FITTING_BUILDER = 125,
  PLAN_FAIL_REF_LINE_CURVE_FITTING = 126,

  // LAT_DECISION 130~139
  PLAN_FAIL_HEURISTIC_SPEED_PLANNER = 130,
  PLAN_FAIL_BUILD_OBS_SL_MAP = 131,
  PLAN_FAIL_DP_PATH_PLANNER = 132,
  PLAN_FAIL_LANE_BORROW_DECIDER = 133,
  PLAN_FAIL_UPDATE_OBS_LAT_DESICISON = 134,

  // LON_DECISION 140~149
  PLAN_FAIL_LEADER_SPEED_DECIDER = 140,
  PLAN_FAIL_CURVATURE_LIMIT_DECIDER = 141,
  PLAN_FAIL_MERGE_SPEED_DECIDER = 142,
  PLAN_FAIL_SPEED_LIMIT_DECIDER = 143,
  PLAN_FAIL_OBSTACLE_LONGITUDE_DECIDER = 144,
  PLAN_FAIL_ST_BOUNDARY_MAPPER = 145,
  PLAN_FAIL_DP_SPEED_PLANNER = 146,
  PLAN_FAIL_PATH_LONGITUDE_DECIDER = 147,

  // LAT_PLANNER 150~159
  PLAN_FAIL_PATH_BUILDER = 150,
  PLAN_FAIL_PATH_SOLVER = 151,
  PLAN_FAIL_PATH_PICEWISE = 152,
  PLAN_FAIL_PATH_ASSESSMENT = 153,
  PLAN_FAIL_PATH_SIZE = 154,

  // LON_PLANNER 160~169
  PLAN_FAIL_SPEED_PATH_INVAILD = 160,
  PLAN_FAIL_SPEED_PROBLEM_BUILDER = 161,
  PLAN_FAIL_SPEED_SOLVER = 162,
  PLAN_FAIL_SPEED_TRAJECTORY_BUILDER = 163,

  // LANE_CHANGE 170~179
  PLAN_FAIL_LC_NO_MAP = 170,
  PLAN_FAIL_LC_NO_TARGET_LANE = 171,
  PLAN_FAIL_LC_MAP_ROUTE_FAIL = 172,

  // MAP 180~199
  PLAN_FAIL_INVALID_CENTER_LINE = 180,
  PLAN_FAIL_SEQ_HAS_NULL_LANE = 181,
  PLAN_FAIL_IS_ON_LANE_INVALID = 182,
  PLAN_FAIL_NEXT_LANES_INVALID = 183,

  ROUTING_FAILED = 184,
  REFERENCE_PATH_UNAVAILABLE = 185,
  START_POINT_PROJECTION_TO_ROUTE_FAILED = 186,
  RESET_PREV_TARGET_LANE_PATH_FAILED = 187,
  TARGET_LANE_CANDIDATES_UNAVAILABLE = 188,
  UPDATE_TARGET_LANE_PATH_FAILED = 189,
  LOCAL_LANE_MAP_BUILDER_FAILED = 190,
  PATH_EXTENSION_FAILED = 191,
  PLANNER_STATE_INCOMPLETE = 192,
  INPUT_INCORRECT = 193,

  // TRAFFIC_RULES 200~209
  PLAN_FAIL_TRAFFIC_LIGHT_FAIL = 200

  // RESERVE 210~255

};
enum CoordinateType {
  COORDINATE_VEHICLE = 0,
  COORDINATE_ODOMETRY = 1,
  COORDINATE_ENU = 2,
  COORDINATE_UTM = 3,
  COORDINATE_WGS84 = 4
};
enum MapType {
  UNKNOWN_MAP = 0,
  PERCEPTION_MAP = 1,
  HD_MAP = 2,
  BEV_MAP = 3,
  FUSION_MAP = 4,
  VIRTUAL_MAP = 5,
};
enum MapSubType {
  UNKNOWN_MAP_SUB_TYPE = 0,
  BIG_JUNCTION = 1,
  PERCEPTION_LOSS = 2,
  JUNCTION_T_MAP = 3,
  IN_JUNCTION_T_MAP = 4,
  NEAR_TOLL_FOR_LCC = 5,
  HIGHWAY_HD = 10,
  LOCATION_LOW_PRECISION = 12,
  VIRTUAL_PERCEPTION_LANE = 13,
  NEAR_TOLL = 14,
  NEAR_TOLL_FOR_RAMP = 15,
  HD_TO_CTF = 16,
  BEFORE_NEAR_TOLL = 17
};
enum ObstacleType {
  OBJECT_UNKNOWN = 0,
  OBJECT_CAR = 1,
  OBJECT_TRUCK = 2,
  OBJECT_MOTORCYCLE = 3,
  OBJECT_BICYCLE = 4,
  OBJECT_PEDESTRIAN = 5,
  OBJECT_CONE = 6,
  OBJECT_BARREL = 7,
  OBJECT_BARRIER = 8,
  OBJECT_SAFETY_TRIANGLE = 9,
  OBJECT_PARKING_LOCK = 10,
  OBJECT_SPACE_LIMITER = 11,
  OBJECT_TRICYCLE = 12,
  OBJECT_SPECIALVEHICLE = 13,
  OBJECT_BUS = 14,
  OBJECT_MINIVEHICLE = 15,
  OBJECT_CONSTRUCTION_VEHICLE = 16,
  OBJECT_UNKNOWN_MOVABLE = 17,
  OBJECT_UNKNOWN_UNMOVABLE = 18,
  OBJECT_TRAFFIC_BAR = 19,
  OBJECT_ISOLATE_BAR = 20
};

enum class ObstacleLightType : uint8_t {
  UNKNOWN = 0,
  OFF = 1,
  ON = 2,
};

enum class ObstacleSuperType {
  UNKNOWN = 0,
  STATIC = 1,
  VEHICLE = 2,
  CYCLIST = 3,
  PEDESTRIAN = 4,
};
enum class PriorityType {
  DEFAULT = 0,
  IGNORE = 1,  // ignore obstacle
  CAUTION = 2  // caution obstacle
};
struct RankerType {
  PriorityType priority_type = PriorityType::DEFAULT;
  double score = 0.0;
};
enum class ObstacleStaticScene {
  OFF_JUNCTION = 0,        // off junction
  IN_JUNCTION = 1,         // in junction
  ROUNDABOUT = 2,          // roundabout scene
  ROUNDABOUT_JUNCTION = 3  // roundabout scene

};
enum class ObstacleLaneRelation {
  UNKNOWN = 0,  // unknown scenario
  NEAR_LEFT = 1,
  NEAR_CENTER = 2,
  NEAR_RIGHT = 3,
};
enum class NearbyType { UNSET = 1, LEFT = 2, RIGHT = 3 };  // lane2lane

enum class PredictorType {
  UNKNOWN = 0,
  FREE_MOVE = 1,
  SPLINE = 2,
  INTERPOLATE = 3,
  LANE = 4,
  CUTIN = 5
};

enum ObstacleIntention {
  INTENTION_UNKNOWN = 0,
  INTENTION_PARALLEL = 1,     // 车道内行驶
  INTENTION_LC_LEFT = 2,      // 左换道
  INTENTION_LC_RIGHT = 3,     // 右换道
  INTENTION_TURN_LEFT = 4,    // 左转
  INTENTION_TURN_RIGHT = 5,   // 右转
  INTENTION_TURN_AROUND = 6,  // 调头
  INTENTION_CROSS = 7,        // 横穿
  INTENTION_MERGE_LEFT = 8,
  INTENTION_MERGE_RIGHT = 9
};

enum LcReason {
  LC_REASON_NONE = 0,
  LC_REASON_MANUAL = 1,
  LC_REASON_EMERGENCY = 2,
  LC_REASON_FOR_MERGE = 3,
  LC_REASON_FOR_NAVI = 4,
  LC_REASON_OVERTAKE = 5,
  LC_REASON_CENTER = 6,
  LC_REASON_FOR_AVOID_VEHICLE = 7,
  LC_REASON_FOR_AVOID_STATIC_OBJECT = 8,
  LC_REASON_FOR_AVOID_ROADWORK = 9,
  LC_REASON_FOR_AVOID_GENERAL = 10,
  LC_REASON_FOR_AVOID_LANE_BUS = 11,         // 公交车道
  LC_REASON_FOR_AVOID_LANE_REVERSIBLE = 12,  // 潮汐车道
  LC_REASON_FOR_AVOID_VARIABLE_TURN = 13,    // 可变车道
  LC_REASON_FOR_AVOID_MULTI_DIR = 14,        // 复合车道
  LC_REASON_FOR_AVOID_MERGE_AREA = 15
};

enum ScenarioType {
  ScenarioType_Acc = 0,
  ScenarioType_LaneFollow = 1,
  ScenarioType_Intersection = 2,
  ScenarioType_Uturn = 3,
  ScenarioType_RoundAbout = 4
};
enum LaneChangeState {
  Lane_Keeping = 0,
  Lane_PrepareLeft = 1,
  Lane_PrepareRight = 2,
  Lane_ChangeLeft = 3,
  Lane_ChangeRight = 4,
  Lane_ChangeCancel = 5,
  Lane_KeepingInit = 6,
  Lane_ChangeInvalid = -1
};
enum PushDirection {
  Push_None = 0,
  Push_Normal_Left = 1,
  Push_Normal_Right = 2,
  Push_Congestion_Left = 3,
  Push_Congestion_Right = 4
};
enum PlannerType {
  Lattice_Planner = 0,
  DP_Path_Planner = 1,
  QP_Path_Planner = 2,
  DP_Speed_Planner = 3
};
typedef math::Vec2d Point2d;
typedef math::Vec3d Point3d;
struct SLPoint {
  SLPoint() = default;
  SLPoint(double _s, double _l) {
    s = _s;
    l = _l;
  }
  double s = 0.0;
  double l = 0.0;
};
typedef std::vector<SLPoint> SLPoints;
struct STPoint {
  STPoint() = default;
  STPoint(double _s, double _t) : s(_s), t(_t) {};
  double s = 0.0;
  double t = 0.0;
};
struct FrenetPoint {
  FrenetPoint() = default;
  FrenetPoint(double _s, double _ds, double _dds, double _l, double _dl,
              double _ddl)
      : s(_s), ds(_ds), dds(_dds), l(_l), dl(_dl), ddl(_ddl) {}
  double s = 0.0;
  double ds = 0.0;
  double dds = 0.0;
  double l = 0.0;
  double dl = 0.0;
  double ddl = 0.0;
};
struct PathPoint : public Point2d {
  double theta = 0.0;
  double kappa = 0.0;
  double dkappa = 0.0;
  double s = 0.0;
  double l = 0.0;
  double accum_s = 0.0;
};
struct TrajectoryPoint : public PathPoint {
  TrajectoryPoint() = default;
  double t = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
};
struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;
};
struct SpeedPoint {
  SpeedPoint() = default;
  SpeedPoint(double _t, double _s, double _v, double _a, double _j)
      : t(_t), s(_s), v(_v), a(_a), j(_j) {};
  double t = -1.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
};
struct TransformInfo {
  TransformInfo() = default;
  TransformInfo(double _x, double _y, double _yaw)
      : delta_x(_x), delta_y(_y), delta_yaw(_yaw) {}
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_yaw = 0.0;
};
struct LocalizationInfo {
  double timestamp = 0.0;
  CoordinateType coordinate_type = COORDINATE_ODOMETRY;
  std::string frame_id;
  Point3d position;
  Quaternion q;
  Point3d v;
  Point3d a;
  Point3d angular_v;
  std::vector<double> position_cov;
  std::vector<double> quaternion_cov;
};
struct PerceptionObstacle {
  std::string id;
  ObstacleType type = OBJECT_UNKNOWN;
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double v = 0.0;
  double a = 0.0;
  double theta = 0.0;    // direction of velocity
  double heading = 0.0;  // direction of bounding box
  double length = 0.0;
  double width = 0.0;
  double height = 0.0;
  double confidence = 1.0;
  double cutin_prob = 0.0;
  int obstacle_state = 0;
  std::vector<Point2d> corner_points;
};

struct SLBoundary {
  double s_min = DBL_MAX;
  double s_max = -DBL_MAX;
  double l_min = DBL_MAX;
  double l_max = -DBL_MAX;
  SLPoint center() const {
    return {(s_min + s_max) * 0.5, (l_min + l_max) * 0.5};
  }
  double width() const { return l_max > l_min ? l_max - l_min : 0.0; }
  double length() const { return s_max > s_min ? s_max - s_min : 0.0; }
  bool IsValid() const { return l_max > l_min && s_max > s_min; }
};

enum class PolylineExtractorType {
  LANEPOLYLINE = 0,
  POLYGONPOLYLINE = 1,
  ROADBOUNDARYPOLYLINE = 2,
  STOPLINEPOLYLINE = 3
};

enum class PluginType { NNPLANNERPP = 0, NNPLANNERDECISION = 1 };

enum ObstacleDecision {
  Decision_Ignore = 0,
  Decision_NudgeLeft = 1,
  Decision_NudgeRight = 2,
  Decision_KeepLeft = 3,
  Decision_KeepRight = 4,
  Decision_Follow = 5,
  Decision_Yield = 6,
  Decision_Overtake = 7,
  Decision_None = -1
};

enum DecisionLabel {
  UNKNOWN_DECISION = -1,
  IGNORE = 0,
  YIELD = 1,
  OVERTAKE = 2,
  LEFT_OFFSET = 3,
  RIGHT_OFFSET = 4,
  DECISION_MAXNUM = 6
};

struct ObstacleForLane {
  std::string id;
  std::string current_lane_id;
  Point2d obstacle_odo_pos;
  double speed;
  std::vector<Point2d> boundary_point;
  bool is_dynamic = false;
  ObstacleType obs_type = OBJECT_UNKNOWN;
};

struct TurnInfo {
  int32_t start = 0;
  double angle = 0.0;
};

struct RoadClassInfo {
  int32_t start = 0;
  int32_t end = 0;
  RoadClass class_type = ROADCLASSES_UNKNOWN;
};

struct LaneGroupInfo {
  int32_t start = 0;
  int32_t end = 0;
  int32_t num_of_lanes = 0;
  std::vector<std::string> lane_ids;
};

struct DrivePassageBlockInfo {
  bool is_block{false};
  double pre_final_stop_s{1000.0};
  std::string pre_obs_id{"none"};
};
struct ConstructionInfo {
  bool pre_is_construction_scene{false};
  int none_construction_scene_cnt{15};
  DrivePassageBlockInfo ego_lane_block_info;
  DrivePassageBlockInfo left_lane_block_info;
  DrivePassageBlockInfo right_lane_block_info;
  bool pre_current_is_lane_keep{true};
  bool pre_lc_change_switch_to_lc_keep{false};
  bool pre_construction_is_left{false};
  bool pre_is_emergence_lane_scene{false};
};

struct SpeedState {
  std::string attention_obj_id = "";
  bool lcc_keep_brake = false;
  bool yield_to_vru = false;
  double vru_interact_timer = 0.0;
  std::vector<std::string> infront_vru_ids;
  double pre_fast_speed_limit = 0.0;  // record fast_speed_limit  m/s
  int pre_lc_num = 0;
  bool lc_num_has_up = false;
  double pre_dynamic_acc_limit = 0.0;  // record dynamic_acc_limit  m/s^2
  bool pre_special_acc_gap_secnario = false;
  bool first_has_speed_limit = false;
  // construction scene speed limt
  bool is_construction_scene_speed_plan = false;
  double set_construction_hold_time = 0.0;
  double function_exit_count = 0.0;
  double set_ExitConstructionSpeedLimitTime = 0.0;
  bool is_construction_speed_constrain_working = false;
  bool is_construction_speed_constrain_working_pre = false;
  double eie_atcion_start_time = -1.0;
  bool is_eie_first_action = true;
  bool is_eie_cnoa_action = false;
  double last_break_dist = 1000.0;

  void Reset() {
    attention_obj_id = "";
    lcc_keep_brake = false;
    yield_to_vru = false;
    vru_interact_timer = 0.0;
    infront_vru_ids.clear();
    pre_fast_speed_limit = 0.0;
    pre_lc_num = 0;
    lc_num_has_up = false;
    pre_dynamic_acc_limit = 0.0;
    first_has_speed_limit = false;
    pre_special_acc_gap_secnario = false;
    is_construction_scene_speed_plan = false;
    set_construction_hold_time = 0.0;
    function_exit_count = 0.0;
    set_ExitConstructionSpeedLimitTime = 0.0;
    is_construction_speed_constrain_working = false;
    is_construction_speed_constrain_working_pre = false;
    is_eie_cnoa_action = false;
  }
};

struct FormOfWayInfo {
  FormOfWayInfo() = default;
  FormOfWayInfo(int32_t _start, int32_t _end, RoadType _roadType)
      : start(_start), end(_end), road_type(_roadType) {};
  int32_t start = -1;
  int32_t end = -1;
  RoadType road_type = RoadType::ROADTYPE_UNKNOWN;
};

struct RoundaboutInfo {
  int32_t is_in_roundabout = 0;
  int32_t junction_num_enter_roundabout = -1;
  int32_t junction_num_exit_roundabout = -1;
  int32_t distance_to_enter_roundabout = -1;
  int32_t distance_to_exit_roundabout = -1;
  RoundaboutInfo() = default;
  RoundaboutInfo(int32_t _is_in_roundabout,
                 int32_t _junction_num_enter_roundabout,
                 int32_t _junction_num_exit_roundabout,
                 int32_t _distance_to_enter_roundabout,
                 int32_t _distance_to_exit_roundabout)
      : is_in_roundabout(_is_in_roundabout),
        junction_num_enter_roundabout(_junction_num_enter_roundabout),
        junction_num_exit_roundabout(_junction_num_exit_roundabout),
        distance_to_enter_roundabout(_distance_to_enter_roundabout),
        distance_to_exit_roundabout(_distance_to_exit_roundabout) {};
};

enum SerialJunctionType {
  STRAIGHT_STRAIGHT = 0,
  STRAIGHT_LEFT = 1,
  STRAIGHT_RIGHT = 2,
  LEFT_STRAIGHT = 3,
  LEFT_LEFT = 4,
  LEFT_RIGHT = 5,
  RIGHT_STRAIGHT = 6,
  RIGHT_LEFT = 7,
  RIGHT_RIGHT = 8,
  NO_NAVI = 9
};

enum class EIEChoiceType {
  CHOICE_NONE = 0,
  CHOICE_BRAKING_DOWN = 1,
  CHOICE_RIGHT_LANE = 2,
};
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_COMMON_TYPE_DEF_H
