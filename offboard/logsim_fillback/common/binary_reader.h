#pragma once
#include <iostream>
#include <sstream>
#include <string>
namespace worldview {
namespace util {
uint8_t parseUInt8(const std::string& raw, bool little_endian = true);
uint16_t parseUInt16(const std::string& raw, bool little_endian = true);
uint32_t parseUInt32(const std::string& raw, bool little_endian = true);
uint64_t parseUInt64(const std::string& raw, bool little_endian = true);

template <typename T>
std::string asHex(T num) {
  std::stringstream ssm;
  ssm << "0x" << std::hex << num;
  return ssm.str();
}

template <typename T>
T parseUInt(const std::string& raw, bool little_endian) {
  auto len = sizeof(T);
  if (len > raw.size()) {
    throw std::invalid_argument("string size smaller than requested len");
  }
  T res = 0;
  if (little_endian) {
    for (size_t i = 0; i < len; ++i) {
      res = (res << 8) + static_cast<uint8_t>(raw[len - 1 - i]);
    }

  } else {
    for (size_t i = 0; i < len; ++i) {
      res = (res << 8) + static_cast<uint8_t>(raw[i]);
    }
  }
  return res;
}

class BinaryReader {
 public:
  BinaryReader(std::istream& s, bool little_endian = true)
      : stream_(s), little_endian_(little_endian) {}

  uint8_t readUInt8() { return readUInt<uint8_t>(); }
  uint16_t readUInt16() { return readUInt<uint16_t>(); }
  uint32_t readUInt32() { return readUInt<uint32_t>(); }
  uint64_t readUInt64() { return readUInt<uint64_t>(); }

  std::string readRaw(size_t len);

  bool good() const { return stream_.good(); }

 private:
  template <typename T>
  T readUInt() {
    std::string data = readRaw(sizeof(T));
    return parseUInt<T>(data, little_endian_);
  }

  std::istream& stream_;
  bool little_endian_;
};

}  // namespace util
}  // namespace worldview