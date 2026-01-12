

#ifndef ONBOARD_AUTONOMY_STATE_AUTONOMY_STATE_UTIL_H_
#define ONBOARD_AUTONOMY_STATE_AUTONOMY_STATE_UTIL_H_

#include "modules/cnoa_pnc/planning/proto/autonomy_state.pb.h"
namespace st {

inline bool IsAutoDrive(AutonomyStateProto::State autonomy_state) {
  return autonomy_state == AutonomyStateProto::AUTO_DRIVE ||
         autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE;
}

inline bool IsAutoDriveToAutoSteerOnly(
    AutonomyStateProto::State pre_autonomy_state,
    AutonomyStateProto::State cur_autonomy_state) {
  return IsAutoDrive(pre_autonomy_state) &&
         cur_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY;
}

inline bool IsAutoSteerOnlyToAutoDrive(
    AutonomyStateProto::State pre_autonomy_state,
    AutonomyStateProto::State cur_autonomy_state) {
  return pre_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY &&
         IsAutoDrive(cur_autonomy_state);
}

inline bool IsAutoSpeedOnlyToAutoDrive(
    AutonomyStateProto::State pre_autonomy_state,
    AutonomyStateProto::State cur_autonomy_state) {
  return pre_autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY &&
         IsAutoDrive(cur_autonomy_state);
}

}  // namespace st
// clang-format off
#define IS_ENGAGE(pre_autonomy_state, last_autonomy_state)                   \
  ((pre_autonomy_state == AutonomyStateProto::READY_TO_AUTO_DRIVE) &&        \
  ((last_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE) ||  \
  (last_autonomy_state == AutonomyStateProto::AUTO_DRIVE)))

#define IS_DISENGAGE(pre_autonomy_state, last_autonomy_state)                \
  (((pre_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (pre_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE)||     \
  (pre_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY) ||             \
  (pre_autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY)) &&            \
  ((last_autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP) ||         \
  (last_autonomy_state == AutonomyStateProto::TAKEOVER) ||                   \
  (last_autonomy_state == AutonomyStateProto::NOT_READY) ||                  \
  (last_autonomy_state == AutonomyStateProto::READY_TO_REMOTE_DRIVE)))

#define IS_KICKOUT(pre_autonomy_state, last_autonomy_state)                  \
  (((pre_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (pre_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE) ||    \
  (pre_autonomy_state == AutonomyStateProto::READY_TO_AUTO_DRIVE)||          \
  (pre_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY)||              \
  (pre_autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY)) &&            \
  ((last_autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP) ||         \
  (last_autonomy_state == AutonomyStateProto::TAKEOVER) ||                   \
  (last_autonomy_state == AutonomyStateProto::NOT_READY) ||                  \
  (last_autonomy_state == AutonomyStateProto::READY_TO_REMOTE_DRIVE)))

#define IS_TAKEOVER(pre_autonomy_state, last_autonomy_state)                 \
  (((pre_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (pre_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE)||     \
  (pre_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY) ||             \
  (pre_autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY)) &&            \
  ((last_autonomy_state == AutonomyStateProto::TAKEOVER) ||                  \
  (last_autonomy_state == AutonomyStateProto::NOT_READY)))

#define IS_AUTO_DRIVE_MODE(pre_autonomy_state, last_autonomy_state)          \
  (((pre_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (pre_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE)) &&   \
  ((last_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (last_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE)))

#define IS_AUTO_DRIVE(autonomy_state)                    \
  ((autonomy_state == AutonomyStateProto::AUTO_DRIVE) || \
  (autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE))

#define IS_L2_DRIVE(autonomy_state)                    \
  ((autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY) || \
  (autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY))

#define IS_EMERGENCY_TO_STOP(pre_autonomy_state, last_autonomy_state)        \
  (((pre_autonomy_state == AutonomyStateProto::AUTO_DRIVE) ||                \
  (pre_autonomy_state == AutonomyStateProto::REMOTE_ASSIST_AUTO_DRIVE) ||    \
  (pre_autonomy_state == AutonomyStateProto::AUTO_STEER_ONLY) ||             \
  (pre_autonomy_state == AutonomyStateProto::AUTO_SPEED_ONLY)) &&            \
  ((last_autonomy_state == AutonomyStateProto::EMERGENCY_TO_STOP)))

// clang-format on

#endif  // ONBOARD_AUTONOMY_STATE_AUTONOMY_STATE_UTIL_H_
