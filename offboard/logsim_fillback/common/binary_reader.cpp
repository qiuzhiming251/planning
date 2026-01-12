#include <algorithm>

#include "common/binary_reader.h"
#include "common/log.h"
namespace worldview {
namespace util {

uint8_t parseUInt8(const std::string& raw, bool little_endian) {
  return parseUInt<uint8_t>(raw, little_endian);
}
uint16_t parseUInt16(const std::string& raw, bool little_endian) {
  return parseUInt<uint16_t>(raw, little_endian);
}
uint32_t parseUInt32(const std::string& raw, bool little_endian) {
  return parseUInt<uint32_t>(raw, little_endian);
}
uint64_t parseUInt64(const std::string& raw, bool little_endian) {
  return parseUInt<uint64_t>(raw, little_endian);
}

std::string BinaryReader::readRaw(size_t len) {
  std::string res(len, 0);
  stream_.read(res.data(), len);
  if (res.size() != len) {
    throw std::runtime_error("failed to read " + std::to_string(len));
  }
  return res;
}

}  // namespace util
}  // namespace worldview