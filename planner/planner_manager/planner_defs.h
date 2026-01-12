

#ifndef ONBOARD_PLANNER_PLANNER_DEFS_H_
#define ONBOARD_PLANNER_PLANNER_DEFS_H_

#include "absl/time/time.h"
#include "plan_common/math/util.h"

namespace st {
namespace planning {

// Platform-wise constants.
#ifdef __X9HP__
constexpr int kTrajectorySteps = 80;             // 8s.
constexpr int kInitializerTrajectorySteps = 80;  // 8s.
constexpr int kInitializerCostEvalStep =
    3;  // Evaluate initializer cost every three steps.
constexpr double kPlanningTimeHorizon = 13.0;  // s.
#else
constexpr int kTrajectorySteps = 80;             // 8s.
constexpr int kInitializerTrajectorySteps = 80;  // 8s.
constexpr int kInitializerCostEvalStep =
    3;  // Evaluate initializer cost every three steps.
constexpr double kPlanningTimeHorizon = 16.0;  // s.
#endif

// Run planner main loop at 10Hz.
constexpr absl::Duration kPlannerMainLoopInterval = absl::Milliseconds(100);

constexpr double kTrajectoryTimeStep = 0.1;  // 100ms.
constexpr double kTrajectoryTimeHorizon =
    (kTrajectorySteps - 1) * kTrajectoryTimeStep;  // from 0.0s to 9.9s.
constexpr double kMaxLatAccCheckTime = 3.0;        // 3.0s.
// Initializer trajectory horizon.
constexpr double kInitializerTrajectoryTimeHorizon =
    (kInitializerTrajectorySteps - 1) * kTrajectoryTimeStep;

constexpr int kAccTrajectorySteps = 80;  // 8s.
constexpr double kAccTrajectoryTimeHorizon =
    (kAccTrajectorySteps - 1) * kTrajectoryTimeStep;  // from 0.0s to 7.9s.
constexpr double kSpacetimePlannerTrajectoryHorizon =
    kTrajectoryTimeStep * kTrajectorySteps;

constexpr double kSpacetimePlannerBehindCarTrajectoryHorizon =
    1.0;  // behindCar spacetime trajectory horizon
constexpr double kSpacetimePlannerVRUTrajectoryHorizon =
    2.0;  // VRU spacetime trajectory horizon

constexpr double kSpacetimePlannerInCrossingTrajectoryHorizon =
    3.0;  // obs in crossing spacetime trajectory horizon

constexpr double kPathSampleInterval = 0.2;  // m.
constexpr int kSpeedFinderMaxTrajectorySteps = 80;

constexpr int kMaxPastPointNum = 50;
constexpr int kMaxAccPastPointNum = 5;

constexpr double kSpaceTimeVisualizationDefaultTimeScale = 10.0;  // m/s.

constexpr double kMinLCSpeed = 5.0 / 3.6;  // m/s.
constexpr double kMinLcLaneLength = 20.0;  // m.  Planner 3.0

constexpr double kDefaultLaneWidth = 3.5;  // m.
constexpr double kDefaultHalfLaneWidth = 0.5 * kDefaultLaneWidth;
constexpr double kMaxHalfLaneWidth = 2.7;                  // m.
constexpr double kMinLaneWidth = 2.6;                      // m.
constexpr double kMinHalfLaneWidth = kMinLaneWidth * 0.5;  // m.

// Desired lateral distance for nudge.
constexpr double kMaxLateralOffset = 10.0;         // m.
constexpr double kMaxLaneKeepLateralOffset = 0.4;  // m.

constexpr double kRouteStationUnitStep = 1.0;  // m.

constexpr double kDrivePassageKeepBehindLength = 50.0;  // m.

constexpr double kLaneChangeCheckForwardLength = 180.0;   // m.
constexpr double kLaneChangeCheckBackwardLength = 100.0;  // m.

constexpr double kMaxTravelDistanceBetweenFrames = 200.0;  // m.

constexpr double kInitializerMinFollowDistance = 1.0;

// Should be shorter than the width of a map patch.
constexpr double kPlannerLaneGraphLength = 200.0;  // m.

// Trajectory time length with curvature constraint
constexpr double kCurvatureLimitRange = 6.0;

constexpr double kUTurnCurbGain = 0.3;

constexpr double kCurbGain = 1.0;

constexpr double kNormalToVirtual = 20.0;  // m.
constexpr double kVirtualToNormal = 50.0;  // m.

const char SoftNameString[] = "Soft";
const char HardNameString[] = "Hard";

constexpr double kAlternateRouteAllowRoundaboutDist = 2000.0;  // m.

const std::string kInvalidOnlineMapId = "";

constexpr int kAsyncCounterInitVal = -1;
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_PLANNER_DEFS_H_
