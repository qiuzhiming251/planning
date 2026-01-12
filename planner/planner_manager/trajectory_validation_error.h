

#ifndef ONBOARD_PLANNER_TRAJECTORY_VALIDATION_ERROR_H_
#define ONBOARD_PLANNER_TRAJECTORY_VALIDATION_ERROR_H_

#include <string>

#include "modules/cnoa_pnc/planning/proto/trajectory_validation.pb.h"

namespace st {
namespace planning {

class TrajectoryValidationError {
 public:
  TrajectoryValidationError()
      : error_code_(TrajectoryValidationResultProto::UNKNOWN_ERROR),
        error_message_("unknown error code") {}

  TrajectoryValidationError(
      TrajectoryValidationResultProto::ErrorCode error_code,
      const std::string& error_message)
      : error_code_(error_code), error_message_(error_message) {}

  void AddToProto(TrajectoryValidationResultProto* proto) {
    TrajectoryValidationResultProto::TrajectoryValidationErrorProto* error =
        proto->add_validation_errors_ex();
    error->set_error_code(error_code_);
    error->set_error_message(error_message_);
  }

  void SetErrorCode(TrajectoryValidationResultProto::ErrorCode error_code) {
    error_code_ = error_code;
  }
  const TrajectoryValidationResultProto::ErrorCode& GetErrorCode() const {
    return error_code_;
  }

  const std::string& GetErrorCodeName() const {
    return TrajectoryValidationResultProto::ErrorCode_Name(error_code_);
  }

  void SetErrorMessage(const std::string& error_message) {
    error_message_ = error_message;
  }
  const std::string& GetErrorMessage() const { return error_message_; }

 private:
  TrajectoryValidationResultProto::ErrorCode error_code_;
  std::string error_message_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_TRAJECTORY_VALIDATION_ERROR_H_
