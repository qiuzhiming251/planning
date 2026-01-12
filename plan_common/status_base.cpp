

#include "status_base.h"

namespace ad_byd {
namespace planning {

StatusBase::InfoMap::InfoMap(const StatusBase::InfoMap &other) {
  for (auto kv : other.data) {
    auto ptr = std::shared_ptr<StatusBase::BaseInfo>(kv.second->clone());
    this->data.insert(std::make_pair(kv.first, ptr));
  }
}

StatusBase::InfoMap &StatusBase::InfoMap::operator=(
    const StatusBase::InfoMap &other) {
  this->data.clear();
  for (auto kv : other.data) {
    auto ptr = std::shared_ptr<StatusBase::BaseInfo>(kv.second->clone());
    this->data.insert(std::make_pair(kv.first, ptr));
  }
  return *this;
}

StatusBase::InfoMap &StatusBase::InfoMap::operator=(
    StatusBase::InfoMap &&other) {
  data.swap(other.data);
  return *this;
}

StatusBase::InfoMap::InfoMap(StatusBase::InfoMap &&other) {
  data.swap(other.data);
}

std::string StatusBase::to_json_string() const {
  std::stringstream ss;
  {
    cereal::JSONOutputArchive archive(ss);
    archive(this->m_infos);
  }
  return ss.str();
}

void StatusBase::from_json_string(const std::string &val) {
  std::stringstream ss(val);
  cereal::JSONInputArchive archive(ss);
  archive(this->m_infos);
}

}  // namespace planning
}  // namespace ad_byd
