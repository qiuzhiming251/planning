

#include <algorithm>
#include <memory>
#include <sstream>
#include <vector>

#include "plan_common/log.h"

#include "google/protobuf/descriptor.h"
#include "google/protobuf/io/tokenizer.h"
#include "google/protobuf/message.h"
#include "google/protobuf/text_format.h"
#include "plan_common/util/proto_util.h"

namespace st {

void StringToProtoLogCollector::AddError(int line, int column,
                                         const std::string& message) {
  last_parse_error.line = line;
  last_parse_error.column = column;
  last_parse_error.message = message;
}

void StringToProtoLogCollector::AddWarning(int line, int column,
                                           const std::string& message) {
  LOG_WARN << "line: " << line << ", column: " << column
           << ", warning: " << message;
}

std::string StringToProtoLogCollector::LastParseError::DebugString() const {
  std::stringstream ss;
  ss << "line: " << line << ", column: " << column << ", error: " << message;
  return ss.str();
}

bool StringToProtoLogDetail(
    std::string_view proto_string, google::protobuf::Message* proto,
    google::protobuf::io::ErrorCollector* error_collector) {
  google::protobuf::TextFormat::Parser parser;
  parser.RecordErrorsTo(error_collector);
  if (!parser.ParseFromString(std::string(proto_string), proto)) {
    return false;
  }
  return true;
}

void FillInMissingFieldsWithDefault(
    const google::protobuf::Message& default_proto,
    google::protobuf::Message* proto) {
  const google::protobuf::Descriptor* descriptor = proto->GetDescriptor();
  const google::protobuf::Reflection* reflection = proto->GetReflection();
  CHECK_EQ(descriptor->full_name(), default_proto.GetDescriptor()->full_name());

  std::vector<const google::protobuf::FieldDescriptor*> fields_to_copy;
  for (int i = 0; i < descriptor->field_count(); ++i) {
    const google::protobuf::FieldDescriptor* field = descriptor->field(i);
    if (field->is_optional()) {
      if (!reflection->HasField(*proto, field)) {
        fields_to_copy.push_back(field);
        continue;
      }
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
        FillInMissingFieldsWithDefault(
            reflection->GetMessage(default_proto, field),
            reflection->MutableMessage(proto, field));
      }
    }
    if (field->is_repeated()) {
      if (reflection->FieldSize(*proto, field) == 0) {
        fields_to_copy.push_back(field);
        continue;
      }
    }
  }

  if (!fields_to_copy.empty()) {
    std::unique_ptr<google::protobuf::Message> default_copy =
        std::unique_ptr<google::protobuf::Message>(default_proto.New());
    default_copy->CopyFrom(default_proto);
    reflection->SwapFields(default_copy.get(), proto, fields_to_copy);
  }
}

}  // namespace st
