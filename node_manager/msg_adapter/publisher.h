

#ifndef AD_BYD_PLANNING_NODES_PUBLISHER_H
#define AD_BYD_PLANNING_NODES_PUBLISHER_H
#include <functional>

namespace ad_byd {
namespace planning {

template <typename Msg>
class Publisher {
 public:
  using msg_type = Msg;
  using msg_ptr_type = std::shared_ptr<Msg>;
  using callback_t = std::function<void(const msg_ptr_type&)>;

  Publisher(const std::string& topic) : topic_(topic) {}

  virtual ~Publisher() = default;

  virtual void Publish(const msg_ptr_type& m) {
    if (user_cb_) {
      user_cb_(m);
    }
  }

  void SetUserCallback(const callback_t& cb) { user_cb_ = cb; }
  const std::string& GetTopic() const { return topic_; }

 protected:
  std::string topic_;
  callback_t user_cb_;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_NODES_PUBLISHER_H
