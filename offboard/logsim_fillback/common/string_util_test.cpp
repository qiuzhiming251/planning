#include <iostream>

#include "common/string_util.h"

using namespace worldview;
void split_test() {
  std::string str = "pnc_idls::idls::Debug";
  std::string ch = "::";
  auto res = util::split(str, ch);
  for (size_t i = 0; i < res.size(); i++) {
    std::cout << "res[" << i << "] = " << res[i] << std::endl;
  }
}

int main() {
  split_test();
  return 0;
}