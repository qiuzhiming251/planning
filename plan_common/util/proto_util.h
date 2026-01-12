

#ifndef ONBOARD_UTILS_PROTO_UTIL_H_
#define ONBOARD_UTILS_PROTO_UTIL_H_

#include <optional>
#include <string>

#include "absl/strings/string_view.h"
#include "google/protobuf/io/tokenizer.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/field_comparator.h"
#include "google/protobuf/util/message_differencer.h"

namespace st {

inline bool ProtoEquals(const google::protobuf::Message& lhs,
                        const google::protobuf::Message& rhs) {
  return google::protobuf::util::MessageDifferencer::Equivalent(lhs, rhs);
}

inline bool operator==(const google::protobuf::Message& lhs,
                       const google::protobuf::Message& rhs) {
  return ProtoEquals(lhs, rhs);
}

inline bool operator!=(const google::protobuf::Message& lhs,
                       const google::protobuf::Message& rhs) {
  return !ProtoEquals(lhs, rhs);
}

template <class Proto>
void TextToProto(const std::string& text, Proto* proto) {
  google::protobuf::TextFormat::ParseFromString(text, proto);
}

// Return the difference between two protobuf messages. The returned value is
// std::nullopt if there is no difference. When partial is true, only fields
// present in the lhs message are considered, fields set only in the rhs message
// will be skipped during comparison. `margin` is used to allow float or double
// type approximate matching.
template <class Proto>
std::optional<std::string> ProtoDiff(const Proto& lhs, const Proto& rhs,
                                     bool partial, double margin = 0.0) {
  using google::protobuf::util::MessageDifferencer;
  MessageDifferencer diff;
  diff.set_report_matches(false);
  diff.set_report_moves(false);
  if (partial) {
    diff.set_scope(MessageDifferencer::PARTIAL);
  }
  using google::protobuf::util::DefaultFieldComparator;
  DefaultFieldComparator field_comparator;
  if (margin != 0.0) {
    // CHECK_GT(margin, 0.0);
    field_comparator.set_float_comparison(DefaultFieldComparator::APPROXIMATE);
    field_comparator.SetDefaultFractionAndMargin(/*fraction=*/0.0, margin);
    diff.set_field_comparator(&field_comparator);
  }
  std::string report_diff;
  bool same = true;
  {
    // This inner scope is necessary to make sure reporter cleaned before using
    // report_diff.
    google::protobuf::io::StringOutputStream ostream(&report_diff);
    MessageDifferencer::StreamReporter reporter(&ostream);
    diff.ReportDifferencesTo(&reporter);
    same = diff.Compare(lhs, rhs);
    if (same) return std::nullopt;
  }
  return report_diff;
}

struct StringToProtoLogCollector : public google::protobuf::io::ErrorCollector {
  struct LastParseError {
    int line = -1;
    int column = -1;
    std::string message;
    std::string DebugString() const;
  };
  StringToProtoLogCollector() = default;
  virtual ~StringToProtoLogCollector() = default;
  void AddError(int line, int column, const std::string& message) override;
  void AddWarning(int line, int column, const std::string& message) override;

  const LastParseError& GetLastParseError() const { return last_parse_error; }

  LastParseError last_parse_error;
};

// Parse string into a proto and output error log if fails.
bool StringToProtoLogDetail(
    std::string_view proto_string, google::protobuf::Message* proto,
    google::protobuf::io::ErrorCollector* error_collector);

// For each optional field (not including repeated), if unset, set using the
// value in default proto. This is done recursively so a subproto will get the
// same treatment, differentiating between existing fields and missing fields.
// Protobuf's built-in MergeFrom() doesn't quite work because it concatenates
// repeated fields which would ruin PLFs and is otherwise semantically incorrect
// in other cases.
void FillInMissingFieldsWithDefault(
    const google::protobuf::Message& default_proto,
    google::protobuf::Message* proto);

}  // namespace st
#endif  // ONBOARD_UTILS_PROTO_UTIL_H_
