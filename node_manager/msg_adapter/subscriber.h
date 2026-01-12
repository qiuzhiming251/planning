
#ifndef AD_BYD_PLANNING_NODES_SUBSCRIBER_H
#define AD_BYD_PLANNING_NODES_SUBSCRIBER_H

#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>

#include "absl/status/statusor.h"
#include "plan_common/constants.h"
#include "plan_common/math/double.h"
#include "node_manager/msg_adapter/msg_queue.h"

namespace ad_byd {
namespace planning {

template <typename Msg>
class Subscriber {
 public:
  using callback_t = std::function<void(const Msg&)>;
  using stamp_getter_t = std::function<double(const Msg&)>;

  Subscriber(const std::string& topic, const stamp_getter_t& stamp_getter)
      : topic_(topic), stamp_getter_(stamp_getter) {}

  virtual ~Subscriber() = default;

  void RegisterCounter(size_t count_i, int max) {
    max_counts_[count_i] = max;
    cur_counts_[count_i] = max;
  }

  void OnNewMessage(const Msg& m) {
    {
      std::scoped_lock<std::mutex> lock(mutex_);
      for (size_t i = 0; i < max_counts_.size(); ++i) {
        cur_counts_[i] = max_counts_[i];
      }
      HandleNewMessage(m);
    }
    if (user_cb_) {
      user_cb_(m);
    }
  }

  void DisableCheckDdl() { should_check_ddl_ = false; }
  void SetUserCallback(const callback_t& cb) { user_cb_ = cb; }
  const std::string& GetTopic() const { return topic_; }

 protected:
  virtual void HandleNewMessage(const Msg& m) {}

  absl::Status CheckRecvDdl(size_t count_i) {
    if (should_check_ddl_ && --cur_counts_[count_i] < 0) {
      std::ostringstream ossm;
      ossm << topic_ << " recv ddl exceeded: actual " << cur_counts_[count_i]
           << ", ddl " << max_counts_[count_i];
      return absl::DeadlineExceededError(ossm.str());
    }
    return absl::OkStatus();
  }

  std::string topic_;
  callback_t user_cb_;
  stamp_getter_t stamp_getter_;
  bool should_check_ddl_{true};
  std::vector<int> max_counts_ = {Constants::MSG_TIMEOUT_COUNTER,
                                  Constants::MSG_TIMEOUT_COUNTER};
  std::vector<int> cur_counts_ = {Constants::MSG_TIMEOUT_COUNTER,
                                  Constants::MSG_TIMEOUT_COUNTER};
  mutable std::mutex mutex_;
};

template <typename Msg>
class SingleSubscriber : public Subscriber<Msg> {
 public:
  using stamp_getter_t = typename Subscriber<Msg>::stamp_getter_t;
  SingleSubscriber(const std::string& topic, const stamp_getter_t& stamp_getter)
      : Subscriber<Msg>(topic, stamp_getter) {}

  virtual void HandleNewMessage(const Msg& m) override {
    if (msg_) {
      *msg_ = m;
    } else {
      msg_.reset(new Msg(m));
    }
  }

  absl::StatusOr<std::pair<double, std::shared_ptr<Msg>>> Get(size_t count_i) {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    if (!msg_) {
      return absl::UnavailableError(this->GetTopic());
    }
    if (auto status = this->CheckRecvDdl(count_i); !status.ok()) {
      return status;
    }
    return std::make_pair(this->stamp_getter_(*msg_),
                          std::make_shared<Msg>(*msg_));
  }

 protected:
  std::shared_ptr<Msg> msg_{nullptr};
};

template <typename Msg>
class QueueSubscriber : public Subscriber<Msg> {
 public:
  using stamp_getter_t = typename Subscriber<Msg>::stamp_getter_t;
  using value_type = typename MsgQueue<Msg>::value_type;

  QueueSubscriber(const std::string& topic, size_t capacity,
                  const stamp_getter_t& stamp_getter)
      : Subscriber<Msg>(topic, stamp_getter), queue_(capacity) {}

  virtual void HandleNewMessage(const Msg& m) override {
    auto next_stamp = this->stamp_getter_(m);
    auto prev_stamp = queue_.BackStamp();
    if (prev_stamp.ok()) {
      auto comp = math::Double::Compare(*prev_stamp, next_stamp);
      if (math::Double::CompareType::EQUAL == comp) {
        LOG_ERROR << this->GetTopic() << " stamp repeats " << std::fixed
                  << std::setprecision(4) << *prev_stamp << " -> "
                  << next_stamp;
        queue_.Clear();
      } else if (math::Double::CompareType::GREATER == comp) {
        LOG_ERROR << this->GetTopic() << " stamp goes backward: " << std::fixed
                  << std::setprecision(4) << *prev_stamp << " -> "
                  << next_stamp;
        queue_.Clear();
      }
    }
    queue_.Append(this->stamp_getter_(m), m);
  }

  absl::StatusOr<value_type> GetMsgAt(double stamp_s) const {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    return queue_.GetMsgAt(stamp_s);
  }

  absl::StatusOr<std::deque<value_type>> Dump(size_t count_i) {
    std::deque<value_type> res;
    {
      std::scoped_lock<std::mutex> lock(this->mutex_);
      if (queue_.Empty()) {
        return absl::UnavailableError(this->GetTopic());
      }
      queue_.Swap(res);
      if (auto status = this->CheckRecvDdl(count_i); !status.ok()) {
        return status;
      }
    }
    return res;
  }

  absl::StatusOr<value_type> Back(size_t count_i) {
    std::scoped_lock<std::mutex> lock(this->mutex_);
    if (auto status = this->CheckRecvDdl(count_i); !status.ok()) {
      return status;
    }

    return queue_.Back();
  }

 protected:
  MsgQueue<Msg> queue_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_SUBSCRIBER_H
