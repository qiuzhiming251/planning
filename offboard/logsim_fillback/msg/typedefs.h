#pragma once
// add cyber record
#include <google/protobuf/descriptor.h>
#include <memory>
#include "cyber/message/raw_message.h"
#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "cyber/node/writer.h"
#include "cyber/record/record_message.h"
#include "cyber/record/record_reader.h"
#include "cyber/record/record_viewer.h"
#include "cyber/record/record_writer.h"

using DynamicGoogleMessage = google::protobuf::Message;
using DynamicGoogleMessagePtr = std::shared_ptr<DynamicGoogleMessage>;
using DynamicTypeCyber = google::protobuf::Descriptor;
using DynamicTypeCyberPtr = std::shared_ptr<const DynamicTypeCyber>;
using DynamicDataCyber = apollo::cyber::message::RawMessage;
using DynamicDataCyberPtr = std::shared_ptr<DynamicDataCyber>;
using DynamicDataWriterCyber = apollo::cyber::Writer<DynamicDataCyber>;
using DynamicDataWriterCyberPtr = std::shared_ptr<DynamicDataWriterCyber>;
using DynamicDataReaderCyber = apollo::cyber::record::RecordReader;
using DynamicDataReaderCyberPtr = std::shared_ptr<DynamicDataReaderCyber>;
using DynamicDataRecordMessageCyber = apollo::cyber::record::RecordMessage;
using DynamicDataRecordMessageCyberPtr =
    std::shared_ptr<DynamicDataRecordMessageCyber>;
using RecordViewerCyber = apollo::cyber::record::RecordViewer;
using RecordViewerCyberPtr = std::shared_ptr<RecordViewerCyber>;
using NodeCyberPtr = std::shared_ptr<apollo::cyber::Node>;
using DynamicDataChannelReaderCyber = apollo::cyber::Reader<DynamicDataCyber>;
using DynamicDataChannelReaderCyberPtr =
    std::shared_ptr<DynamicDataChannelReaderCyber>;
using RecordWriterCyber = apollo::cyber::record::RecordWriter;
using RecordWriterCyberPtr = std::shared_ptr<RecordWriterCyber>;
