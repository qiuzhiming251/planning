#pragma once
#include <functional>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>
namespace worldview {
namespace util {

template <typename T>
std::string join(const std::vector<T> &container, const std::string &sep) {
  std::ostringstream ossm;
  if (container.empty()) {
    return ossm.str();
  }
  for (auto it = container.begin(); it != container.end(); ++it) {
    if (it == container.begin()) {
      ossm << *it;
    } else {
      ossm << sep << *it;
    }
  }
  return ossm.str();
}

template <typename T>
std::string join(const std::set<T> &container, const std::string &sep) {
  std::ostringstream ossm;
  if (container.empty()) {
    return ossm.str();
  }
  for (auto it = container.begin(); it != container.end(); ++it) {
    if (it == container.begin()) {
      ossm << *it;
    } else {
      ossm << sep << *it;
    }
  }
  return ossm.str();
}

template <typename T>
std::string join(const std::vector<T> &container, char sep) {
  return join(container, std::string(1, sep));
}
template <typename T>
std::string join(const std::set<T> &container, char sep) {
  return join(container, std::string(1, sep));
}

template <typename K, typename V>
std::string join_key(const std::map<K, V> &m, const std::string &sep) {
  std::vector<K> keys;
  for (auto &kv : m) {
    keys.push_back(kv.first);
  }
  return join(keys, sep);
}

template <typename K, typename V>
std::string join_key(const std::map<K, V> &m, char sep) {
  return join_key(m, std::string(1, sep));
}

std::vector<std::string> split(const std::string &str, char ch = ' ');
std::vector<std::string> split(const std::string &str, const std::string &ch);
std::string replace(const std::string &in, const std::string &from,
                    const std::string &to);

std::string trim(const std::string &s,
                 const std::string &space = " \f\n\r\t\v");

}  // namespace util
}  // namespace worldview