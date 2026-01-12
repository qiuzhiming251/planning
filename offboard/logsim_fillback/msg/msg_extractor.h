
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "common/json_util.h"
#include "common/string_util.h"
#include "msg/typedefs.h"

namespace worldview {

enum MsgFieldValueCategory { DYNAMIC_DATA, STRING, PRIMITIVE };

struct MsgFieldValue {
  using Ptr = std::shared_ptr<MsgFieldValue>;

  MsgFieldValue(MsgFieldValueCategory c) : category(c) {}
  virtual ~MsgFieldValue() = default;
  virtual std::string toString() const = 0;
  virtual double toDouble() const { return 0.0; }
  virtual Json::Value toJson() const = 0;

  MsgFieldValueCategory category;
};

enum MsgFieldCategory { SELECTOR, FILTER, INDEX };

struct MsgField {
  using Ptr = std::shared_ptr<MsgField>;
  MsgField(MsgFieldCategory c, const std::string &n)
      : category(c), field_name(n) {}

  virtual ~MsgField() = default;
  virtual std::string name(const std::string &prefix = "") const = 0;

  MsgFieldCategory category;
  std::string field_name;
  uint32_t field_index{0};
  std::string field_value;

  static std::vector<MsgField::Ptr> merge(
      const std::vector<MsgField::Ptr> &fields);
  static std::string mergeName(const std::vector<MsgField::Ptr> &fields,
                               const std::string &prefix = "");
};

class MsgFieldCreator {
 public:
  MsgFieldCreator() = delete;
  static MsgField::Ptr create(const std::string &s);
  static std::vector<MsgField::Ptr> createList(const std::string &s);
};

struct SelectorMsgField : public MsgField {
  SelectorMsgField(const std::string &n)
      : MsgField(MsgFieldCategory::SELECTOR, n) {}

  virtual std::string name(const std::string &prefix) const override {
    std::string res = util::replace(field_name, "[.]", "/");
    if (prefix.length()) {
      return prefix + "/" + res;
    }
    return res;
  }
};

struct FilterMsgField : public MsgField {
  FilterMsgField(const std::string &n, const std::string &v)
      : MsgField(MsgFieldCategory::FILTER, n), value(v) {
    field_value = v;
  }

  virtual std::string name(const std::string &prefix) const override {
    return realName(prefix, value);
  }

  std::string realName(const std::string &prefix, const std::string &v) const {
    std::string res = field_name == kNoFilter ? "全部" : (field_name + "=" + v);
    if (prefix.length()) {
      return prefix + "/" + res;
    }
    return res;
  }

  std::string value;

  static const std::string kNoFilter;
};

struct IndexMsgField : public MsgField {
  IndexMsgField(uint32_t i) : MsgField(MsgFieldCategory::INDEX, "_index") {
    field_index = i;
  }

  virtual std::string name(const std::string &prefix) const override {
    std::string res = "[" + std::to_string(field_index) + "]";
    if (prefix.length()) {
      return prefix + "/" + res;
    }
    return res;
  }
};
}  // namespace worldview