

#ifndef ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_
#define ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_

#include <string>

#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "modules/cnoa_pnc/planning/proto/planner_status.pb.h"

namespace st::planning {

class PlannerStatus {
 public:
  PlannerStatus() {}

  PlannerStatus(PlannerStatusProto::PlannerStatusCode status, std::string str)
      : status_(std::move(status)), message_(std::move(str)) {}

  bool ok() const { return status_ == PlannerStatusProto::OK; }
  const PlannerStatusProto::PlannerStatusCode& status_code() const {
    return status_;
  }
  const std::string& message() const { return message_; }

  std::string ToString() const {
    return PlannerStatusProto::PlannerStatusCode_Name(status_) + ": " +
           message_;
  }

  void ToProto(PlannerStatusProto* proto) const {
    proto->Clear();
    proto->set_status(status_);
    *proto->mutable_message() = message_;
  }

  void FromProto(const PlannerStatusProto& proto) {
    if (proto.has_status()) {
      status_ = proto.status();
    }
    if (proto.has_message()) {
      message_ = proto.message();
    }
  }

 private:
  PlannerStatusProto::PlannerStatusCode status_ = PlannerStatusProto::OK;
  std::string message_;
};

PlannerStatus OkPlannerStatus();

}  // namespace st::planning

#endif  // ONBOARD_PLANNER_COMMON_PLANNER_STATUS_H_
