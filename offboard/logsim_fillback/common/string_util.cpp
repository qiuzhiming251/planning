#include <functional>
#include <iostream>
#include <map>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "common/string_util.h"
namespace worldview {

namespace util {

std::vector<std::string> split(const std::string &str, char ch) {
  std::stringstream ssm(str);
  std::vector<std::string> res;
  std::string s;
  while (std::getline(ssm, s, ch)) {
    res.push_back(s);
  }
  return res;
}

std::vector<std::string> split(const std::string &str, const std::string &ch) {
  std::vector<std::string> res;
  size_t start = 0;
  while (start < str.size()) {
    auto pos = str.find(ch, start);
    if (std::string::npos != pos) {
      res.push_back(str.substr(start, pos - start));
      start = pos + ch.size();
    } else {
      break;
    }
  }
  res.push_back(str.substr(start));
  return res;
}

std::string replace(const std::string &in, const std::string &from,
                    const std::string &to) {
  return std::regex_replace(in, std::regex(from), to);
}

std::string trim(const std::string &s, const std::string &space) {
  auto start = s.find_first_not_of(space);
  if (std::string::npos == start) {
    return "";
  }
  auto end = s.find_last_not_of(space);
  return s.substr(start, end - start + 1);
}

}  // namespace util
}  // namespace worldview