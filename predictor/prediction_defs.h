

#ifndef ONBOARD_PREDICTION_PREDICTION_DEFS_H_
#define ONBOARD_PREDICTION_PREDICTION_DEFS_H_

#include <array>
#include <cmath>
#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "plan_common/math/geometry/box2d.h"
#include "plan_common/math/piecewise_linear_function.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/perception.pb.h"
#include "plan_common/drive_passage.h"
namespace st {
namespace prediction {

using ResampledObjectsHistory = std::vector<std::vector<st::ObjectProto>>;

using ObjectIDType = std::string;
using ProbTrajPair = std::pair<double, std::vector<Vec2d>>;

using ObjectProbTrajs = std::vector<ProbTrajPair>;
using ObjectsProbTrajs = absl::flat_hash_map<std::string, ObjectProbTrajs>;

enum class PredictTypePrio { HIGH, MED, LOW };
using TypePrioMap =
    std::map<PredictTypePrio, const absl::flat_hash_set<ObjectType>>;

struct ObjectMotionState {
  double timestamp = 0.0;
  Vec2d pos;
  double heading = 0.0;
  Vec2d vel;
  Box2d bbox;

  std::string DebugString() const {
    return absl::StrFormat("timestamp: %.6f, pos: %s, heading: %.6f, vel: %s.",
                           timestamp, pos.DebugString(), heading,
                           vel.DebugString());
  }
};

struct ObjectMotionHistory {
  ObjectIDType id;
  ObjectType type;
  std::vector<ObjectMotionState> states;
  absl::Span<const ObjectMotionState> GetStatesInRange(
      double time_range) const {
    if (states.empty()) return {};
    int idx = 0;
    for (; idx < states.size(); ++idx) {
      if (states[idx].timestamp >= states.back().timestamp - time_range) {
        break;
      }
    }
    return absl::MakeSpan(&(states[idx]), std::size_t(states.size() - idx));
  }
};
using ObjectsMotionHistory = std::vector<ObjectMotionHistory>;

// Agent centric net outputs with uncertainty.
using NLLTrajPoint = std::array<double, 5>;  // x, y, s1, s2, c.
struct AgentCentricObjectProbTraj {
  double mode_prob = 0.0;
  std::array<double, 3> relation_probs = {0.0, 0.0, 0.0};  // void, yield, pass.
  std::vector<NLLTrajPoint> traj_points;
  double rot_rad = 0.0;
};
using AgentCentricObjectProbTrajs = std::vector<AgentCentricObjectProbTraj>;
using AgentCentricObjectsProbTrajs =
    absl::flat_hash_map<std::string, AgentCentricObjectProbTrajs>;

struct AgentCentricObjectOut {
  AgentCentricObjectProbTrajs prob_trajs;
  std::optional<double> startup_prob;
};
using AgentCentricObjecstOut =
    absl::flat_hash_map<std::string, AgentCentricObjectOut>;

// Cutin net outputs.
using CutinTrajPoint = std::array<double, 2>;  // x, y
struct CutinObjectProbTraj {
  double mode_prob = 0.0;
  std::vector<CutinTrajPoint> traj_points;
  double rot_rad = 0.0;
};

using CutinObjectProbTrajs = std::vector<CutinObjectProbTraj>;
using CutinObjectsProbTrajs =
    absl::flat_hash_map<std::string, CutinObjectProbTrajs>;

struct CutinObjectOut {
  CutinObjectProbTrajs prob_trajs;
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};

using CutinObjectsOut = absl::flat_hash_map<std::string, CutinObjectOut>;

// CutinSLNet output
struct CutinSLObjectOut {
  std::vector<double> channle_probs;
  int predicted_channel;
  int cur_channel;
};
using CutinSLObjectsOut = absl::flat_hash_map<std::string, CutinSLObjectOut>;

// LaneSelectionNet output
struct LaneSelectionObjectOut {
  std::map<planning::DrivePassage, double> scores_for_dps;
};
using LaneSelectionObjectsOut =
    absl::flat_hash_map<std::string, LaneSelectionObjectOut>;

struct AgentInfoWithDps {
  ObjectIDType id;
  std::vector<planning::DrivePassage> drive_passages;
  std::vector<float> dp_scores;
};

constexpr double kEpsilon = 1e-8;

constexpr char kAvObjectId[] = "AV";
constexpr char kInvalidObjectId[] = "NA";

// Feature 2.0 motion history configuration.
constexpr double kFeatureV2HistoryStepNum = 21;
constexpr double kLaneSelectionHistoryStepNum = 10;
constexpr double kFeatureV2HistoryStepLen = 0.1;  // Seconds.
// Feature 2.0 map sampling configuration.
constexpr double kFeatureV2MaxMapSampleLen = 10.0;  // m.
constexpr int kFeatureV2MapSegmentNum = 5;

constexpr double kPredictionTimeStep = 0.1;  // Seconds.
constexpr double kPredictionDuration =
    8.0;  // Prediction seconds for non-stationary trajectory.
constexpr double kCutinPredictionDuration =
    5.0;  // Prediction seconds for cutin non-stationary trajectory.
const int kPredictionPointNum =
    static_cast<int>(kPredictionDuration / kPredictionTimeStep);

constexpr double kEmergencyGuardHorizon = 3.0;  // s.
constexpr double kSafeHorizon = 8.0;            // s.
constexpr double kComfortableHorizon = 8.0;     // s.

constexpr double kVehCurvatureLimit = 0.25;    // m^-1.
constexpr double kVehLateralAccelLimit = 2.5;  // m^-1.

constexpr double kHistoryLen = 1.5;

constexpr double kTooShortTrajLen = 0.5;  // m.

constexpr double kAccelerationFitTime = 1.0;  // s.
// Time to maintain heuristic acceleration.
constexpr double kMaintainAccTime = 1.5;  // s.

// J5 trajectory number
constexpr int kActNetJ5ModelTrajNum = 2;

// Post processing related constants.
constexpr int kPolyFitDownSampleStep = 3;

// Pole placement related stuff.
// Length data for pedestrian, bike, shunxxx, aion, jinlv minibus, polerstar
// bus.
const std::vector<double> kObjectLengthDataPoint = {1.0,   1.57,  2.95,
                                                    4.786, 5.995, 10.48};
const PiecewiseLinearFunction<double, double> kLengthToWheelbasePlf(
    kObjectLengthDataPoint,
    std::vector<double>{0.6, 1.22, 2.1, 2.92, 3.85, 5.89});
const PiecewiseLinearFunction<double, double> kLengthToMaxFrontSteerPlf(
    kObjectLengthDataPoint,
    std::vector<double>{M_PI / 3.0, 0.49, 0.496, 0.546, 0.583, 0.67});
struct AVObjectRelation {
  double no_relation;
  double yield;
  double pass;

  std::string DebugString() const {
    return absl::StrFormat("void: %f, yield: %f, pass: %f", no_relation, yield,
                           pass);
  }
};

using ProbTrajPair = std::pair<double, std::vector<Vec2d>>;

using ObjectProbTrajs = std::vector<ProbTrajPair>;
using ObjectsProbTrajs = absl::flat_hash_map<std::string, ObjectProbTrajs>;
// Agent centric net outputs with uncertainty.
using NLLTrajPoint = std::array<double, 5>;  // x, y, s1, s2, c.

using AgentCentricObjectProbTrajs = std::vector<AgentCentricObjectProbTraj>;
using AgentCentricObjectsProbTrajs =
    absl::flat_hash_map<std::string, AgentCentricObjectProbTrajs>;

using AgentCentricObjecstOut =
    absl::flat_hash_map<std::string, AgentCentricObjectOut>;

using CutinSLObjectsOut = absl::flat_hash_map<std::string, CutinSLObjectOut>;

}  // namespace prediction
}  // namespace st

#endif  // ONBOARD_PREDICTION_PREDICTION_DEFS_H_
