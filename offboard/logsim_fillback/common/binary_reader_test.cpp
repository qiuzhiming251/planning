#include <sstream>

#include "common/binary_reader.h"
using namespace worldview;
int main() {
  std::string data(15, 0);
  for (int i = 0; i < 15; i++) {
    data[i] = i + 1;
  }
  std::stringstream ssm(data);
  util::BinaryReader reader(ssm);
  auto i8 = reader.readUInt8();
  auto i16 = reader.readUInt16();
  auto i32 = reader.readUInt32();
  auto i64 = reader.readUInt64();

  std::cout << "expecting: " << 1 << " " << 0x0302 << " " << 0x07060504 << " "
            << 0x0f0e0d0c0b0a0908 << std::endl;
  std::cout << "get: " << static_cast<int>(i8) << " " << i16 << " " << i32
            << " " << i64 << std::endl;
  return 0;
}