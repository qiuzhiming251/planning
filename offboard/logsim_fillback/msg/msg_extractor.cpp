#include "msg/msg_extractor.h"

#include "common/json_util.h"
#include "common/log.h"
namespace worldview {

const std::string FilterMsgField::kNoFilter = "不过滤";

std::vector<MsgField::Ptr> MsgField::merge(
    const std::vector<MsgField::Ptr> &fields) {
  std::vector<MsgField::Ptr> res;
  auto begin = fields.begin();
  while (begin != fields.end()) {
    if ((*begin)->category == MsgFieldCategory::SELECTOR) {
      auto end = begin;
      while (end != fields.end() &&
             (*end)->category == MsgFieldCategory::SELECTOR) {
        ++end;
      }
      if (std::distance(begin, end) == 1) {
        res.push_back(*begin);
      } else {
        MsgField::Ptr merged(new SelectorMsgField(""));
        std::vector<std::string> segments;
        for (auto it = begin; it != end; ++it) {
          segments.push_back((*it)->field_name);
        }
        merged->field_name = util::join(segments, '.');
        res.push_back(merged);
      }
      begin = end;
    } else {
      res.push_back(*begin);
      ++begin;
    }
  }
  return res;
}

std::string MsgField::mergeName(const std::vector<MsgField::Ptr> &fields,
                                const std::string &prefix) {
  std::string res = prefix;
  for (auto &f : fields) {
    res = f->name(res);
  }
  return res;
}

MsgField::Ptr MsgFieldCreator::create(const std::string &s) {
  if (s.empty()) {
    throw std::invalid_argument("cannot create field with empty string");
  }
  if (s.front() == '[') {
    if (s.back() != ']') {
      throw std::invalid_argument("unrecognized field " + s);
    }
    std::string content = s.substr(1, s.size() - 1);
    try {
      auto idx = static_cast<uint32_t>(std::stoul(content));
      return MsgField::Ptr(new IndexMsgField(idx));
    } catch (std::exception &e) {
      throw std::invalid_argument("unrecognized field " + s + "(" + e.what() +
                                  ")");
    }
  } else {
    auto segments = util::split(s, "=");
    if (segments.size() == 2) {
      return MsgField::Ptr(new FilterMsgField(segments[0], segments[1]));
    } else if (segments.size() != 1) {
      throw std::invalid_argument("unrecognized field " + s);
    }
    if (s == "全部") {
      return MsgField::Ptr(new FilterMsgField(FilterMsgField::kNoFilter, ""));
    }
    return MsgField::Ptr(new SelectorMsgField(s));
  }
}
std::vector<MsgField::Ptr> MsgFieldCreator::createList(const std::string &s) {
  std::vector<MsgField::Ptr> res;
  if (s.empty()) {
    return res;
  }
  auto segments = util::split(s, "/");
  for (auto &segment : segments) {
    res.emplace_back(create(segment));
  }
  return res;
}
}  // namespace worldview