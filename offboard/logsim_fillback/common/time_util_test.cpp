#include <iostream>

#include "common/time_util.h"
using namespace worldview;
int main() {
  std::cout << util::stamp2readable(0) << "\n"
            << util::stamp2readable(1649409293272716) << std::endl;
  return 0;
}