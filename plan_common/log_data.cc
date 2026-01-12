
#include "plan_common/log_data.h"

#include <absl/time/clock.h>
#include <absl/time/time.h>

#include <google/protobuf/message.h>

#if defined(__x86_64__) || defined(_M_X64)
DEFINE_int32(log2dds_verbosity_level, 4, "log2dds verbosity level local");
#else
DEFINE_int32(log2dds_verbosity_level, 2, "log2dds verbosity level orin");
#endif
namespace st {
namespace planning {
using namespace byd::msg::planning;
Log2DDS::Color SetColor(int r, int g, int b, int a) {
  Log2DDS::Color color;
  color.set_r(r);
  color.set_g(g);
  color.set_b(b);
  color.set_a(a);
  return color;
}
const Log2DDS::Color Log2DDS::kBlack = SetColor(0, 0, 0, 1);
const Log2DDS::Color Log2DDS::kWhite = SetColor(255, 255, 255, 1);
const Log2DDS::Color Log2DDS::kRed = SetColor(255, 0, 0, 1);
const Log2DDS::Color Log2DDS::kGreen = SetColor(0, 128, 0, 1);
const Log2DDS::Color Log2DDS::kBlue = SetColor(0, 0, 255, 1);
const Log2DDS::Color Log2DDS::kYellow = SetColor(255, 255, 0, 1);
const Log2DDS::Color Log2DDS::kOrange = SetColor(255, 165, 0, 1);
const Log2DDS::Color Log2DDS::kPink = SetColor(255, 192, 203, 1);
const Log2DDS::Color Log2DDS::kPurple = SetColor(128, 0, 128, 1);
const Log2DDS::Color Log2DDS::kBrown = SetColor(165, 42, 42, 1);
const Log2DDS::Color Log2DDS::kGray = SetColor(128, 128, 128, 1);
const Log2DDS::Color Log2DDS::kHotpink = SetColor(255, 105, 180, 1);
const Log2DDS::Color Log2DDS::kCoral = SetColor(255, 127, 80, 1);
const Log2DDS::Color Log2DDS::kDarkkhaki = SetColor(189, 183, 107, 1);
const Log2DDS::Color Log2DDS::kViolet = SetColor(238, 130, 238, 1);
const Log2DDS::Color Log2DDS::kLime = SetColor(0, 255, 0, 1);
const Log2DDS::Color Log2DDS::kAqua = SetColor(0, 255, 255, 1);
const Log2DDS::Color Log2DDS::kMagenta = SetColor(255, 0, 255, 1);
const Log2DDS::Color Log2DDS::kDarkRed = SetColor(139, 0, 0, 1);
const Log2DDS::Color Log2DDS::kLightGray = SetColor(179, 179, 179, 0.1);
const Log2DDS::Color Log2DDS::kMiddleBlueGreen = SetColor(141, 217, 204, 1);
const Log2DDS::Color Log2DDS::kTiffanyBlue = SetColor(129, 216, 208, 1);
const Log2DDS::Color Log2DDS::kLightBlue = SetColor(173, 216, 230, 1);
const Log2DDS::Color Log2DDS::kDarkGreen = SetColor(136, 135, 13, 1);
const Log2DDS::Color Log2DDS::kDarkBlue = SetColor(0, 0, 139, 1);
const Log2DDS::Color Log2DDS::kGrassGreen = SetColor(18, 231, 115, 1);

const std::vector<Log2DDS::Color> Log2DDS::kDefaultColors{
    Log2DDS::kOrange,  Log2DDS::kAqua,  Log2DDS::kLime,
    Log2DDS::kMagenta, Log2DDS::kBrown, Log2DDS::kBlue};

byd::msg::planning::DebugFrameProto& Log2DDS::GetThreadDebugFrame() {
  std::scoped_lock<std::mutex> lock(mutex_);
  return frames_[std::this_thread::get_id()];
}

byd::msg::planning::DebugFrameProto Log2DDS::MergeDebugFrames() {
  byd::msg::planning::DebugFrameProto res;
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    VLOG(5) << "merging " << frames_.size() << " frames";
    for (auto& [id, frame] : frames_) {
      for (auto& number : frame.numbers()) {
        res.add_numbers()->CopyFrom(number);
      }
      for (auto& pnc_string : frame.strings()) {
        res.add_strings()->CopyFrom(pnc_string);
      }
      for (auto& number_list : frame.numberlists()) {
        res.add_numberlists()->CopyFrom(number_list);
      }
      for (auto& string_list : frame.stringlists()) {
        res.add_stringlists()->CopyFrom(string_list);
      }
      for (auto& marker : frame.markers()) {
        res.add_markers()->CopyFrom(marker);
      }
      for (auto& group : frame.groups()) {
        res.add_groups()->CopyFrom(group);
      }
      // res.numbers().insert(res.numbers().end(), frame.numbers().begin(),
      //                      frame.numbers().end());
      // res.strings().insert(res.strings().end(), frame.strings().begin(),
      //                      frame.strings().end());
      // res.numberlists().insert(res.numberlists().end(),
      //                          frame.numberlists().begin(),
      //                          frame.numberlists().end());
      // res.stringlists().insert(res.stringlists().end(),
      //                          frame.stringlists().begin(),
      //                          frame.stringlists().end());
      // res.markers().insert(res.markers().end(), frame.markers().begin(),
      //                      frame.markers().end());
      // res.groups().insert(res.groups().end(), frame.groups().begin(),
      //                     frame.groups().end());

      VLOG(5) << "frame tid " << id << " contains " << res.markers().size()
              << " markers, " << res.numbers().size() << " numbers";

      frame.Clear();
      // frame.strings().Clear();
      // frame.numberlists().Clear();
      // frame.stringlists().Clear();
      // frame.markers().Clear();
      // frame.groups().Clear();
    }
  }
  return res;
}

Log2DDS& Log2DDS::Get() {
  static Log2DDS Log2DDS;
  return Log2DDS;
}

std::shared_ptr<byd::msg::planning::DebugFrameProto> Log2DDS::Dump() {
  // auto ret = std::make_shared<byd::msg::planning::DebugFrameProto>();
  auto merged = Get().MergeDebugFrames();

  // calculate debugframe size
  auto all_size = merged.add_numbers();
  all_size->set_name("debugframe_size");
  all_size->set_value(merged.ByteSizeLong());

  size_t msg_size = 0;
  auto numbers_size = merged.add_numbers();
  for (const auto& num : merged.numbers()) {
    msg_size += num.ByteSizeLong();
  }
  numbers_size->set_name("numbers_size");
  numbers_size->set_value(msg_size);

  msg_size = 0;
  for (const auto& str : merged.strings()) {
    msg_size += str.ByteSizeLong();
  }
  auto strings_size = merged.add_numbers();
  strings_size->set_name("strings_size");
  strings_size->set_value(msg_size);

  msg_size = 0;
  for (const auto& numlist : merged.numberlists()) {
    msg_size += numlist.ByteSizeLong();
  }
  auto numberlist_size = merged.add_numbers();
  numberlist_size->set_name("numberlists_size");
  numberlist_size->set_value(msg_size);

  msg_size = 0;
  for (const auto& strlist : merged.stringlists()) {
    msg_size += strlist.ByteSizeLong();
  }
  auto stringlist_size = merged.add_numbers();
  stringlist_size->set_name("stringlists_size");
  stringlist_size->set_value(msg_size);

  msg_size = 0;
  for (const auto& marker : merged.markers()) {
    msg_size += marker.ByteSizeLong();
  }
  auto marker_size = merged.add_numbers();
  marker_size->set_name("markers_size");
  marker_size->set_value(msg_size);

  msg_size = 0;
  for (const auto& group : merged.groups()) {
    msg_size += group.ByteSizeLong();
  }
  auto group_size = merged.add_numbers();
  group_size->set_name("groups_size");
  group_size->set_value(msg_size);

  // std::shared_ptr<byd::msg::planning::DebugFrameProto> ret{&merged};
  auto ret =
      std::make_shared<byd::msg::planning::DebugFrameProto>(std::move(merged));
  return ret;
}

void Log2DDS::LogDataV(int level, const std::string& key,
                       const std::string& value) {
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto pnc_string = GetThreadDebugFrame().add_strings();
  pnc_string->set_name(key);
  pnc_string->set_value(value);
  // GetThreadDebugFrame().strings().emplace_back(key, value);
}

void Log2DDS::LogDataV(int level, const std::string& key, double value) {
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto number = GetThreadDebugFrame().add_numbers();
  number->set_name(key);
  number->set_value(value);
  // GetThreadDebugFrame().numbers().emplace_back(key, value);
}

void Log2DDS::LogDataV(int level, const std::string& key,
                       const std::vector<std::string>& value) {
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto string_list = GetThreadDebugFrame().add_stringlists();
  string_list->set_name(key);
  for (auto& val : value) {
    string_list->add_value(val);
  }
  // GetThreadDebugFrame().stringlists().emplace_back(key, value);
}

void Log2DDS::LogDataV(int level, const std::string& key,
                       const std::vector<double>& value) {
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto number_list = GetThreadDebugFrame().add_numberlists();
  number_list->set_name(key);
  for (auto& val : value) {
    number_list->add_value(val);
  }
  // GetThreadDebugFrame().numberlists().emplace_back(key, value);
}

DebugFrameProto::Marker2d& Log2DDS::AddChart(const std::string& group_name,
                                             const std::string& id,
                                             const Color& color, bool closed) {
  auto& frame = GetThreadDebugFrame();
  auto group_it =
      std::find_if(frame.groups().begin(), frame.groups().end(),
                   [&group_name](const DebugFrameProto::Marker2dGroup& g) {
                     return g.name() == group_name;
                   });
  auto group =
      group_it != frame.groups().end()
          ? &(*frame.mutable_groups())[group_it - frame.groups().begin()]
          : frame.add_groups();
  group->set_name(group_name);
  auto chart = group->add_markers();
  chart->set_id(id);
  chart->set_closed(closed);
  chart->mutable_color()->set_r(color.r());
  chart->mutable_color()->set_g(color.g());
  chart->mutable_color()->set_b(color.b());
  chart->mutable_color()->set_a(color.a());
  return *chart;
}

DebugFrameProto::Marker& Log2DDS::AddLine(
    const std::string& id, const Color& color,
    const std::vector<std::string>& labels, const uint8_t type,
    const double size) {
  auto& frame = GetThreadDebugFrame();
  // auto& marker = frame.markers().emplace_back();
  auto marker = frame.add_markers();
  marker->set_id(id);
  marker->set_type(static_cast<DebugFrameProto::MarkType>(type));
  marker->mutable_color()->set_r(color.r());
  marker->mutable_color()->set_g(color.g());
  marker->mutable_color()->set_b(color.b());
  marker->mutable_color()->set_a(color.a());
  marker->mutable_scale()->set_x(size);
  marker->mutable_scale()->set_y(size);
  marker->mutable_scale()->set_z(size);
  for (auto& label : labels) {
    marker->add_labels(label);
  }

  return *marker;
}

void Log2DDS::LogChartV(int level, const std::string& group_name,
                        const std::string& id, const Color& color, bool closed,
                        const std::vector<double>& xs,
                        const std::vector<double>& ys,
                        const std::vector<Log2DDS::ChartInfos>& infos) {
  CHECK_EQ(xs.size(), ys.size()) << "log chart failure: x.size = " << xs.size()
                                 << ", y.size = " << ys.size();
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto& chart = AddChart(group_name, id, color, closed);
  for (size_t i = 0; i < xs.size(); i++) {
    auto point = chart.add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(0.0);
    // chart.points().emplace_back(xs[i], ys[i], 0);
  }
  if (!infos.empty()) {
    for (auto& info : infos) {
      chart.add_properties()->CopyFrom(info);
    }
    // chart.properties(infos);
  }
}

void Log2DDS::LogLineV(int level, const std::string& id, const Color& color,
                       const std::vector<std::string>& labels,
                       const std::vector<double>& xs,
                       const std::vector<double>& ys, const uint8_t type,
                       const double size) {
  CHECK_EQ(xs.size(), ys.size()) << "log chart failure: x.size = " << xs.size()
                                 << ", y.size = " << ys.size();
  if (level > FLAGS_log2dds_verbosity_level) {
    return;
  }
  auto& line = AddLine(id, color, labels, type, size);
  for (size_t i = 0; i < xs.size(); i++) {
    auto point = line.add_points();
    point->set_x(xs[i]);
    point->set_y(ys[i]);
    point->set_z(0.0);
    // line.points().emplace_back(xs[i], ys[i], 0);
  }
}

}  // namespace planning
}  // namespace st