
#ifndef AD_BYD_PLANNING_NODES_MSG_QUEUE
#define AD_BYD_PLANNING_NODES_MSG_QUEUE
#include <queue>

#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
namespace ad_byd {
namespace planning {

template <typename Msg>
class MsgQueue {
 public:
  using value_type = std::pair<double, std::shared_ptr<Msg>>;
  MsgQueue(size_t capacity) : capacity_(capacity) {}

  size_t Size() const { return msgs_.size(); }

  bool Empty() const { return msgs_.empty(); }

  void Append(double stamp_s, const Msg& m) {
    msgs_.emplace_back(stamp_s, new Msg(m));
    if (capacity_ && msgs_.size() > capacity_) {
      Pop();
    }
  }

  void Clear() { msgs_.clear(); }

  absl::StatusOr<value_type> Back() const {
    if (msgs_.size()) {
      return msgs_.back();
    }

    return absl::UnavailableError("queue empty");
  }

  absl::StatusOr<double> BackStamp() const {
    if (msgs_.size()) {
      return msgs_.back().first;
    }
    return absl::UnavailableError("queue empty");
  }

  void Pop() {
    if (!msgs_.empty()) {
      msgs_.pop_front();
    }
  }

  void Swap(std::deque<value_type>& dq) { dq.swap(msgs_); }

  absl::StatusOr<value_type> GetMsgAt(double stamp_s) const {
    auto it = std::lower_bound(
        msgs_.begin(), msgs_.end(), stamp_s,
        [](const value_type& m, double s) { return m.first < s; });
    if (msgs_.end() == it) {
      return absl::UnavailableError(absl::StrCat("no msg at ", stamp_s));
    }
    return *it;
  }

 private:
  // TODO: use cyclic queue to speed up
  std::deque<value_type> msgs_;
  size_t capacity_{0};
};
}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_MSG_QUEUE
