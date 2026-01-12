#ifndef ST_PLANNING_COMMON_LOG_DATA
#define ST_PLANNING_COMMON_LOG_DATA
#include <gflags/gflags.h>
#include "plan_common/log.h"
#include "absl/strings/str_cat.h"

#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "modules/msg/st_msgs/planning_debug_frame.pb.h"
//#include <pnc_idls/DebugFrame.hpp>

// clang-format off
// Here we use an approach similar to what glog use in VLOG.
// Larger number means more verbose (thus less important).

// Level Definitions:
// log2dds_verbosity_level is set to 4 for development and 2 for product release.
// V0: System and architecture-related
// V1: Almost everyone concerns (e.g., selected_idx)
// V2: Psd data for specific module
// V3: x64 local debugging for specific module
// V4: Temporary usage
// V5: Archived

// User Guide:
// 1.Use debugframe only to record short values, strings, and visualizations.
// 2.Long strings are recommended to use Log.
// 3.Use V4 and V3 first.
// 4.Add V0 - V2 infos should be reviewed.
// 5.Use plan_id to identify different plans and ensure it is named appropriately.
// 6.Utilize additional functions to handle complex visualizations and improve readability.
// clang-format on

DECLARE_int32(log2dds_verbosity_level);

namespace st {
namespace planning {
using namespace byd::msg::planning;

#define ST_LOG_DATA_DEF(level)                                          \
  template <typename T>                                                 \
  static void LogDataV##level(const std::string& key, const T& value) { \
    Log2DDS::Get().LogDataV(level, key, value);                         \
  }

#define ST_LOG_CHART_XY_VALUE_DEF(level)                                       \
  static void LogChartV##level(const std::string& group_name,                  \
                               const std::string& id, const Color& color,      \
                               bool closed, const std::vector<double>& xs,     \
                               const std::vector<double>& ys,                  \
                               const std::vector<Log2DDS::ChartInfos>& infos = \
                                   std::vector<Log2DDS::ChartInfos>()) {       \
    Log2DDS::Get().LogChartV(level, group_name, id, color, closed, xs, ys,     \
                             infos);                                           \
  }

#define ST_LOG_CHART_XY_GETTER_DEF(level)                                  \
  template <typename PointT, typename XGetterT, typename YGetterT>         \
  static void LogChartV##level(                                            \
      const std::string& group_name, const std::string& id,                \
      const Color& color, bool closed, const std::vector<PointT>& points,  \
      const XGetterT& x_getter, const YGetterT& y_getter,                  \
      const std::vector<Log2DDS::ChartInfos>& infos =                      \
          std::vector<Log2DDS::ChartInfos>()) {                            \
    Log2DDS::Get().LogChartV(level, group_name, id, color, closed, points, \
                             x_getter, y_getter, infos);                   \
  }

#define ST_LOG_CHART_XY_GETTER_DEFAULT_DEF(level)                              \
  template <typename PointT>                                                   \
  static void LogChartV##level(const std::string& group_name,                  \
                               const std::string& id, const Color& color,      \
                               bool closed, const std::vector<PointT>& points, \
                               const std::vector<Log2DDS::ChartInfos>& infos = \
                                   std::vector<Log2DDS::ChartInfos>()) {       \
    Log2DDS::Get().LogChartV(                                                  \
        level, group_name, id, color, closed, points,                          \
        [](const PointT& p) -> double { return p.x(); },                       \
        [](const PointT& p) -> double { return p.y(); }, infos);               \
  }

#define ST_LOG_LINE_XY_VALUE_DEF(level)                                      \
  static void LogLineV##level(                                               \
      const std::string& id, const Color& color,                             \
      const std::vector<std::string>& labels, const std::vector<double>& xs, \
      const std::vector<double>& ys, const double size = 2) {                \
    Log2DDS::Get().LogLineV(level, id, color, labels, xs, ys,                \
                            DebugFrameProto::MARKER_TYPE_LINE, size);        \
  }

#define ST_LOG_LINE_XY_GETTER_DEF(level)                                 \
  template <typename PointT, typename XGetterT, typename YGetterT>       \
  static void LogLineV##level(                                           \
      const std::string& id, const Color& color,                         \
      const std::vector<std::string>& labels,                            \
      const std::vector<PointT>& points, const XGetterT& x_getter,       \
      const YGetterT& y_getter, const double size = 2) {                 \
    Log2DDS::Get().LogLineV(level, id, color, labels, points, x_getter,  \
                            y_getter, DebugFrameProto::MARKER_TYPE_LINE, \
                            size);                                       \
  }

#define ST_LOG_LINE_XY_GETTER_DEFAULT_DEF(level)                         \
  template <typename PointT>                                             \
  static void LogLineV##level(const std::string& id, const Color& color, \
                              const std::vector<std::string>& labels,    \
                              const std::vector<PointT>& points,         \
                              const double size = 2) {                   \
    Log2DDS::Get().LogLineV(                                             \
        level, id, color, labels, points,                                \
        [](const PointT& p) -> double { return p.x(); },                 \
        [](const PointT& p) -> double { return p.y(); },                 \
        DebugFrameProto::MARKER_TYPE_LINE, size);                        \
  }

#define ST_LOG_POINTS_XY_VALUE_DEF(level)                                    \
  static void LogPointsV##level(                                             \
      const std::string& id, const Color& color,                             \
      const std::vector<std::string>& labels, const std::vector<double>& xs, \
      const std::vector<double>& ys, const double size = 2) {                \
    Log2DDS::Get().LogLineV(level, id, color, labels, xs, ys,                \
                            DebugFrameProto::MARKER_TYPE_POINT, size);       \
  }

#define ST_LOG_POINTS_XY_GETTER_DEF(level)                                \
  template <typename PointT, typename XGetterT, typename YGetterT>        \
  static void LogPointsV##level(                                          \
      const std::string& id, const Color& color,                          \
      const std::vector<std::string>& labels,                             \
      const std::vector<PointT>& points, const XGetterT& x_getter,        \
      const YGetterT& y_getter, const double size = 2) {                  \
    Log2DDS::Get().LogLineV(level, id, color, labels, points, x_getter,   \
                            y_getter, DebugFrameProto::MARKER_TYPE_POINT, \
                            size);                                        \
  }

#define ST_LOG_POINTS_XY_GETTER_DEFAULT_DEF(level)                         \
  template <typename PointT>                                               \
  static void LogPointsV##level(const std::string& id, const Color& color, \
                                const std::vector<std::string>& labels,    \
                                const std::vector<PointT>& points,         \
                                const double size = 2) {                   \
    Log2DDS::Get().LogLineV(                                               \
        level, id, color, labels, points,                                  \
        [](const PointT& p) -> double { return p.x(); },                   \
        [](const PointT& p) -> double { return p.y(); },                   \
        DebugFrameProto::MARKER_TYPE_POINT, size);                         \
  }

#define ST_LOG_POLYGON_XY_VALUE_DEF(level)                                   \
  static void LogPolygonV##level(                                            \
      const std::string& id, const Color& color,                             \
      const std::vector<std::string>& labels, const std::vector<double>& xs, \
      const std::vector<double>& ys, const double size = 2) {                \
    Log2DDS::Get().LogLineV(level, id, color, labels, xs, ys,                \
                            DebugFrameProto::MARKER_TYPE_POLYGON, size);     \
  }

#define ST_LOG_POLYGON_XY_GETTER_DEF(level)                                 \
  template <typename PointT, typename XGetterT, typename YGetterT>          \
  static void LogPolygonV##level(                                           \
      const std::string& id, const Color& color,                            \
      const std::vector<std::string>& labels,                               \
      const std::vector<PointT>& points, const XGetterT& x_getter,          \
      const YGetterT& y_getter, const double size = 2) {                    \
    Log2DDS::Get().LogLineV(level, id, color, labels, points, x_getter,     \
                            y_getter, DebugFrameProto::MARKER_TYPE_POLYGON, \
                            size);                                          \
  }

#define ST_LOG_POLYGON_XY_GETTER_DEFAULT_DEF(level)                         \
  template <typename PointT>                                                \
  static void LogPolygonV##level(const std::string& id, const Color& color, \
                                 const std::vector<std::string>& labels,    \
                                 const std::vector<PointT>& points,         \
                                 const double size = 2) {                   \
    Log2DDS::Get().LogLineV(                                                \
        level, id, color, labels, points,                                   \
        [](const PointT& p) -> double { return p.x(); },                    \
        [](const PointT& p) -> double { return p.y(); },                    \
        DebugFrameProto::MARKER_TYPE_POLYGON, size);                        \
  }

#define ST_LOG_ALL_LEVEL_DEF(ST_MACRO_NAME) \
  ST_MACRO_NAME(0)                          \
  ST_MACRO_NAME(1)                          \
  ST_MACRO_NAME(2)                          \
  ST_MACRO_NAME(3)                          \
  ST_MACRO_NAME(4)                          \
  ST_MACRO_NAME(5)

class Log2DDS {
 public:
  using Color = byd::msg::planning::DebugFrameProto::Color;
  using ChartInfos = byd::msg::planning::DebugFrameProto::Property;

  static std::shared_ptr<DebugFrameProto> Dump();

  ST_LOG_ALL_LEVEL_DEF(ST_LOG_DATA_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_CHART_XY_VALUE_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_CHART_XY_GETTER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_CHART_XY_GETTER_DEFAULT_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_LINE_XY_VALUE_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_LINE_XY_GETTER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_LINE_XY_GETTER_DEFAULT_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POINTS_XY_VALUE_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POINTS_XY_GETTER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POINTS_XY_GETTER_DEFAULT_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POLYGON_XY_VALUE_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POLYGON_XY_GETTER_DEF)
  ST_LOG_ALL_LEVEL_DEF(ST_LOG_POLYGON_XY_GETTER_DEFAULT_DEF)

  static const Color kBlack;
  static const Color kWhite;
  static const Color kRed;
  static const Color kGreen;
  static const Color kBlue;
  static const Color kYellow;
  static const Color kOrange;
  static const Color kPink;
  static const Color kPurple;
  static const Color kBrown;
  static const Color kGray;
  static const Color kHotpink;
  static const Color kCoral;
  static const Color kDarkkhaki;
  static const Color kViolet;
  static const Color kLime;
  static const Color kAqua;
  static const Color kMagenta;
  static const Color kDarkRed;
  static const Color kLightGray;
  static const Color kMiddleBlueGreen;
  static const Color kTiffanyBlue;
  static const Color kLightBlue;
  static const Color kDarkGreen;
  static const Color kDarkBlue;
  static const Color kGrassGreen;

  static std::string TaskPrefix(int i) { return absl::StrCat("_task", i, "_"); }

 private:
  DebugFrameProto& GetThreadDebugFrame();
  DebugFrameProto MergeDebugFrames();

  // Debug frame data
  void LogDataV(int level, const std::string& key, const std::string& value);
  void LogDataV(int level, const std::string& key, double value);
  void LogDataV(int level, const std::string& key,
                const std::vector<std::string>& value);
  void LogDataV(int level, const std::string& key,
                const std::vector<double>& value);

  DebugFrameProto::Marker2d& AddChart(const std::string& group_name,
                                      const std::string& id, const Color& color,
                                      bool closed);

  DebugFrameProto::Marker& AddLine(const std::string& id, const Color& color,
                                   const std::vector<std::string>& labels,
                                   const uint8_t type, const double size);

  void LogChartV(int level, const std::string& group_name,
                 const std::string& id, const Color& color, bool closed,
                 const std::vector<double>& xs, const std::vector<double>& ys,
                 const std::vector<Log2DDS::ChartInfos>& infos);

  template <typename PointT, typename XGetterT, typename YGetterT>
  void LogChartV(int level, const std::string& group_name,
                 const std::string& id, const Color& color, bool closed,
                 const std::vector<PointT>& points, const XGetterT& x_getter,
                 const YGetterT& y_getter,
                 const std::vector<Log2DDS::ChartInfos>& infos) {
    if (level > FLAGS_log2dds_verbosity_level) {
      return;
    }
    auto& chart = AddChart(group_name, id, color, closed);
    for (auto& p : points) {
      auto point = chart.add_points();
      point->set_x(x_getter(p));
      point->set_y(y_getter(p));
      point->set_z(0.0);
    }
    if (!infos.empty()) {
      for (auto& info : infos) {
        chart.add_properties()->CopyFrom(info);
      }
    }
  }

  // DebugFrame line
  void LogLineV(int level, const std::string& id, const Color& color,
                const std::vector<std::string>& labels,
                const std::vector<double>& xs, const std::vector<double>& ys,
                const uint8_t type, const double size);

  template <typename PointT, typename XGetterT, typename YGetterT>
  void LogLineV(int level, const std::string& id, const Color& color,
                const std::vector<std::string>& labels,
                const std::vector<PointT>& points, const XGetterT& x_getter,
                const YGetterT& y_getter, const uint8_t type,
                const double size) {
    if (level > FLAGS_log2dds_verbosity_level) {
      return;
    }
    auto& line = AddLine(id, color, labels, type, size);
    for (auto& p : points) {
      auto point = line.add_points();
      point->set_x(x_getter(p));
      point->set_y(y_getter(p));
      point->set_z(0.0);
      // line.points().emplace_back(x_getter(p), y_getter(p), 0);
    }
  }

  Log2DDS() = default;

  static Log2DDS& Get();

  static const std::vector<Color> kDefaultColors;

  std::mutex mutex_;
  std::unordered_map<std::thread::id, DebugFrameProto> frames_;
};

}  // namespace planning
}  // namespace st

namespace ad_byd {
namespace planning {
using Log2DDS = st::planning::Log2DDS;
}
}  // namespace ad_byd

#endif  // ST_PLANNING_COMMON_LOG_DATA
