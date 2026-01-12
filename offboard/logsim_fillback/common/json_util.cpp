
#include <fstream>
#include <sstream>

#include "common/json_util.h"
#include "common/log.h"
namespace worldview {
namespace util {

Json::Value readJson(const fs::path& file_path) {
  std::ifstream ifs(file_path);
  if (!ifs.good()) {
    LOG_ERROR << "file not exist: " << fs::absolute(file_path);
    throw std::invalid_argument("file not exist: " +
                                fs::absolute(file_path).string());
  }
  Json::Value root;
  ifs >> root;
  return root;
}

void writeJson(const Json::Value& root, const fs::path& file_path) {
  std::ofstream ofs(file_path);
  if (!ofs.good()) {
    LOG_ERROR << "failed to create file " << file_path;
    throw std::invalid_argument("failed to create file: " +
                                fs::absolute(file_path).string());
  }
  ofs << root;
}

Json::Value readJson(const std::string& filename) {
  return readJson(fs::path(filename));
}

void writeJson(const Json::Value& root, const std::string& filename) {
  writeJson(root, fs::path(filename));
}

Json::Value unmarshal(const std::string& str) {
  Json::Value val;
  std::stringstream ssm(str);
  ssm >> val;
  return val;
}

std::string marshal(const Json::Value& j, bool styled) {
  if (styled) {
    std::stringstream ssm;
    ssm << j;
    return ssm.str();
  }

  Json::FastWriter builder;
  return builder.write(j);
}

template <>
int getOptional(const Json::Value& node, const std::string& key,
                int default_value) {
  return node[key].isNull() ? default_value : node[key].asInt();
}

template <>
double getOptional(const Json::Value& node, const std::string& key,
                   double default_value) {
  return node[key].isNull() ? default_value : node[key].asDouble();
}

template <>
bool getOptional(const Json::Value& node, const std::string& key,
                 bool default_value) {
  return node[key].isNull() ? default_value : node[key].asBool();
}

}  // namespace util
}  // namespace worldview