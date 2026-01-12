#pragma once
#include <set>
#include <string>
#include <vector>
namespace worldview {

struct Filter {
  Filter() = default;
  Filter(const std::string& name) { white.insert(name); }
  Filter(const std::vector<std::string>& names) {
    white.insert(names.begin(), names.end());
  }
  Filter(const std::vector<std::string>& w, const std::vector<std::string>& b) {
    white.insert(w.begin(), w.end());
    black.insert(b.begin(), b.end());
  }

  std::set<std::string> white, black;

  bool allowed(const std::string& name) const {
    if (white.size()) {
      return white.count(name) > 0;
    }
    return black.count(name) == 0;
  }
};
}  // namespace worldview