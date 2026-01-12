#pragma once
#include <json/json.h>

#include "common/filesystem.h"
namespace worldview {
namespace util {

Json::Value readJson(const fs::path& file_path);
void writeJson(const Json::Value& root, const fs::path& file_path);

Json::Value readJson(const std::string& filename);
void writeJson(const Json::Value& root, const std::string& filename);

Json::Value unmarshal(const std::string& str);
std::string marshal(const Json::Value& j, bool styled = true);

template <typename T>
T getOptional(const Json::Value&, const std::string& key, T default_value);

template <>
int getOptional(const Json::Value&, const std::string& key, int default_value);

template <>
double getOptional(const Json::Value&, const std::string& key,
                   double default_value);

template <>
bool getOptional(const Json::Value&, const std::string& key,
                 bool default_value);

}  // namespace util
}  // namespace worldview