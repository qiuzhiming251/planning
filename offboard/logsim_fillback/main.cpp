#include "logsim/logsim_reprocessor.h"
#include "bag/cyber/cyber_record.h"
#include "cyber/cyber.h"
#include "common/log.h"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <sstream>

std::string getCurrentTime() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t), "%Y%m%d%H%M%S");
  return ss.str();
}

int main(int argc, char** argv) {
  apollo::cyber::Init(argv[0]);
  if (argc != 3 && argc != 4) {
    std::cerr << "Usage: " << argv[0]
              << " <input_record_file> <output_folder_path> "
                 "<logsim_mode>(optional, open/close, default open)"
              << std::endl;
    return 1;
  }
  std::string input_record_file = argv[1];
  std::string output_folder_path = argv[2];
  std::string input_logsim_mode = argv[3];
  worldview::LogsimMode logsim_mode = worldview::LogsimMode::LOGSIM_OPEN_LOOP;
  if (argc == 4 && input_logsim_mode == "close") {
    logsim_mode = worldview::LogsimMode::LOGSIM_CLOSED_LOOP;
  }
  if (output_folder_path.back() != '/') {
    output_folder_path += '/';
  }

  std::string currentTime = getCurrentTime();
  fs::path input_path(input_record_file);
  std::string expected_output_file =
      output_folder_path + input_path.filename().string() + "_fillback_" + currentTime;
  std::string temp_output_file = expected_output_file + ".tmp";
  std::vector<std::string> record_files = {input_record_file};

  auto topic_filter = std::make_shared<worldview::Filter>();
  std::vector<std::string> topics_map{"/drivers/canbus/canbus_uplink",
                                      "/localization/dr",
                                      "/perception/env/routing_map",
                                      "/noa_map/routing_map",
                                      "/noa_map/map_event",
                                      "/mpc_interface_drv_vmc_Debug",
                                      "/st/pnc/sm_behavior",
                                      "/prediction/trajectory_v2",
                                      "/st/pnc/pilot_planning_result",
                                      "/st/pnc/planning_debugframe",
                                      "/drivers/canbus/vehicle_info"};
  for (auto topic : topics_map) {
    topic_filter->white.insert(topic);
  }

  int64_t start_offset = 0;
  int64_t end_offset = 4096;

  if (!worldview::CyberRecord::checkValid(record_files)) {
    throw std::invalid_argument("CyberRecord check invalid");
    return 0;
  }

  std::shared_ptr<worldview::CyberRecord> cyber_record_shp =
      std::make_shared<worldview::CyberRecord>(
          record_files, *topic_filter,
          worldview::TypeManagerCyber::getInstance().cleanTypeFilter(), false,
          true, start_offset * 1e9, end_offset * 1e9);
  worldview::Record::Ptr record = cyber_record_shp;
  cyber_record_shp->join();
  if (record->size() == 0) {
    throw std::invalid_argument("Folder Empty(message num = 0)");
  }

  worldview::LogsimModule logsim_config;
  std::string log_name = input_path.filename().string() + "_logsim.log";
  logsim_config.command =
      "/apollo/modules/cnoa_pnc/planning/scripts/logsim_start.sh " + log_name;
  logsim_config.work_dir = "/apollo";
  logsim_config.mode = logsim_mode;
  worldview::LogsimReprocessor logsim_reprocessor(logsim_config, record);
  logsim_reprocessor.setOutputPath(temp_output_file);
  logsim_reprocessor.start();
  logsim_reprocessor.stop();

  try {
    fs::path temp_path(temp_output_file);
    std::string temp_filename = temp_path.filename().string();

    std::string actual_output_file = "";
    for (const auto& entry : fs::directory_iterator(output_folder_path)) {
      std::string filename = entry.path().filename().string();
      if (filename.find(temp_filename) == 0) {
        actual_output_file = entry.path().string();
        break;
      }
    }

    if (actual_output_file.empty()) {
      LOG(ERROR) << "Failed to find generated output file";
      return 1;
    }

    fs::rename(actual_output_file, expected_output_file);
    LOG(INFO) << "Renamed " << actual_output_file << " to "
              << expected_output_file;
  } catch (const fs::filesystem_error& e) {
    LOG(ERROR) << "Filesystem error: " << e.what();
    return 1;
  }

  return 0;
}