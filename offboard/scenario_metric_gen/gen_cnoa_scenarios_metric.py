import os
import subprocess
import sys
import struct
import hashlib
import argparse
from datetime import datetime
from typing import List

try:
    import pandas as pd
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "--user", "pandas"])
    
    user_site_packages = os.path.expanduser("~/.local/lib/python3.8/site-packages")
    if user_site_packages not in sys.path:
        sys.path.append(user_site_packages)
    import pandas as pd

class ScenarioGeneration:
    DELAY_TIME_RECORD = 0
    RECORD_TIME_BUFFER = 1  
    DURATION_TIME_RECORD = 60  
    DEFAULT_DURATION_TIME = 13.0  
    DEFAULT_CLOSED_LOOP_TIME_OFFSET = -3.0
    DEFAULT_START_TIME_BEFORE_TRIGGER_ACTION = -8.0 
    CNOA_SCENARIO_PB_FILE_PATH = os.path.join(
        os.path.dirname(__file__), "scenario_template/template.pb.txt"
    )  


    def __init__(self):
  
        self._scenario_name_list = []  
        self._replace_info_list = []  
        self._scenario_base_txt = [] 


        self.__generate_template_list()

    def __generate_template_list(self):
        template_file_path = ScenarioGeneration.CNOA_SCENARIO_PB_FILE_PATH
        with open(template_file_path, "r") as f:
            for line in f:
                line = line.rstrip("\n")
                self._scenario_base_txt.append(str(line) + "\n")

    def read_csv_data(self, csv_path: str):
       
        self.data = pd.read_csv(csv_path, engine="python")
        self.data = self.data.dropna(how='all')  
        print(self.data)

    def generate_scenarios_name_list(self) -> List[str]:
       
        for i in range(0, len(self.data), 1):
            date_part = str(self.data["场景日期"][i])
            datetime_str = str(date_part + "_" + self.data["场景时刻"][i])
            datetime_obj = datetime.strptime(datetime_str, "%Y/%m/%d_%H:%M:%S")
            unix_time = datetime_obj.timestamp()  
            time = self.__convert_unix_to_datetime(unix_time)  

            scenario_name = (
                str(time) + "_" + str(self.data["车辆"][i])
            )
            self._scenario_name_list.append(scenario_name)
        return self._scenario_name_list

    def generate_replace_info_list(self) -> List[dict]:
        for i in range(0, len(self.data), 1):
            record_oss_path = str(self.data["场景 Records"][i])
            time_info = record_oss_path.split("/")[-1]
            date = time_info.split("_")[0]

            
            datetime_str = str(date + "_" + self.data["场景时刻"][i])
            datetime_obj = datetime.strptime(datetime_str, "%Y-%m-%d_%H:%M:%S")
            trigger_time = datetime_obj.timestamp()

           
            sim_time_setting = self.data["仿真场景参数"][i]



            if str(sim_time_setting) != "nan":
                if "，" in sim_time_setting:
                    sim_time_setting = sim_time_setting.replace("，", ",")
                parts = str(sim_time_setting).split(",")
                start_time_before_trigger_action = parts[0]
                closed_loop_time_offset = str(sim_time_setting).split(",")[1]
                duration_time = parts[2]
            else:
                duration_time = ScenarioGeneration.DEFAULT_DURATION_TIME
                closed_loop_time_offset = (
                    ScenarioGeneration.DEFAULT_CLOSED_LOOP_TIME_OFFSET
                )
                start_time_before_trigger_action = ScenarioGeneration.DEFAULT_START_TIME_BEFORE_TRIGGER_ACTION


            scenario_info = {
                "record_oss_path": record_oss_path,
                "trigger_time": trigger_time,
                "duration_time": duration_time,
                "closed_loop_time_offset": closed_loop_time_offset,
                "start_time_before_trigger_action": start_time_before_trigger_action,
            }
            self._replace_info_list.append(scenario_info)
        return self._replace_info_list

    def __convert_unix_to_datetime(self, timestamp: float) -> str:

        dt_object = datetime.fromtimestamp(timestamp)
        formatted_date_time = dt_object.strftime("%Y%m%d_%H%M%S")
        return formatted_date_time

    def __is_file_exists(self, oss_file_path: str) -> bool:

        cmd = "ossutil --force-path-style ls " + oss_file_path
        ret_info = os.popen(cmd).read()
        return "Object Number is: 0" not in ret_info

    
    def __generate_int64_hash(self, name: str) -> int:
 
        sha256_hash = hashlib.sha256(name.encode("utf-8")).digest()
        hash_bytes = sha256_hash[:8]
        hash_int = struct.unpack(">q", hash_bytes)[0]
        if hash_int < 0:
            hash_int += 2**64
        return hash_int

    def __generate_first_second_record_and_modify_time(
        self,
        record_oss_path: str,
        trigger_time: int,
        simulation_start_timestamp: int,
        simulation_end_timestamp: int,
    ) -> dict:
        
        
        time_info = record_oss_path.split("/")[-1]
        date = time_info.split("_")[0]
        time = time_info.split(".")[-1].replace("-", ":")
        datetime_obj = datetime.strptime(date + "_" + time, "%Y-%m-%d_%H:%M:%S")
        record_start_time = (
            datetime_obj.timestamp() + ScenarioGeneration.DELAY_TIME_RECORD
        )
        record_end_time = record_start_time + ScenarioGeneration.DURATION_TIME_RECORD

       
        oss_file_dir = "/".join(record_oss_path.split("/")[:-1]) + "/"
        cmd = "ossutil --force-path-style ls " + str(oss_file_dir)
        ret_info = os.popen(cmd).read()
        oss_path_list = []
        for info in ret_info.split(" "):
            if (
                "oss://oss-byd-sh-roadtest" in info
                and ".mp4" in info
                and "bev" not in info
                and "mono" not in info
                and "radar_tracking" not in info
            ):
                oss_path_list.append(info.split("\n")[0][:-4])

        if not oss_path_list:
            print("Oss_path_list is empty!")
            return {
                "first_record": record_oss_path,
                "second_record": "",
                "trigger_time": trigger_time,
                "simulation_start_timestamp": simulation_start_timestamp,
                "simulation_end_timestamp": simulation_end_timestamp,
            }

        first_record_in_folder = oss_path_list[0]
        time_ = first_record_in_folder.split(".")[-1].replace("-", ":")
        datetime_obj = datetime.strptime(date + "_" + time_, "%Y-%m-%d_%H:%M:%S")
        start_time_in_folder = datetime_obj.timestamp()
        if start_time_in_folder > simulation_start_timestamp:
            simulation_start_timestamp += 3600 * 24
            simulation_end_timestamp += 3600 * 24
            trigger_time += 3600 * 24
            record_start_time += 3600 * 24
            record_end_time += 3600 * 24

       
        current_record_index = oss_path_list.index(record_oss_path)
        last_record_oss_path = oss_path_list[current_record_index - 1]
        next_record_oss_path = oss_path_list[current_record_index + 1]

       
        first_record = ""
        second_record = ""
        if (
            simulation_start_timestamp < record_start_time + ScenarioGeneration.RECORD_TIME_BUFFER
            and record_start_time - ScenarioGeneration.DURATION_TIME_RECORD < simulation_start_timestamp
        ):
            first_record = last_record_oss_path
            second_record = record_oss_path
        elif (
            simulation_end_timestamp > record_end_time - ScenarioGeneration.RECORD_TIME_BUFFER
            and simulation_end_timestamp < record_end_time + ScenarioGeneration.DURATION_TIME_RECORD
        ):
            first_record = record_oss_path
            second_record = next_record_oss_path
        elif record_start_time <= simulation_start_timestamp and simulation_end_timestamp <= record_end_time:
            first_record = record_oss_path
        else:
            print("Trigger time error, please check!")

        return {
            "first_record": first_record,
            "second_record": second_record,
            "trigger_time": trigger_time,
            "simulation_start_timestamp": simulation_start_timestamp,
            "simulation_end_timestamp": simulation_end_timestamp,
        }
        
    def generate_single_scenario(
        self, input_scenario_info: dict, scenario_name: str
    ) -> List[str]:

        record_oss_path = input_scenario_info["record_oss_path"]
        trigger_time = input_scenario_info["trigger_time"]
        duration_time = input_scenario_info["duration_time"]
        closed_loop_time_offset = input_scenario_info["closed_loop_time_offset"]
        start_time_before_trigger_action = input_scenario_info["start_time_before_trigger_action"]

       
        simulation_start_timestamp = int(trigger_time) + int(start_time_before_trigger_action)
        closed_loop_start_time_offset = int(closed_loop_time_offset) - int(start_time_before_trigger_action)
        simulation_end_timestamp = int(simulation_start_timestamp) + int(duration_time)

       
        ret_oss_state = self.__is_file_exists(record_oss_path)
        modified_content = []
        if not ret_oss_state:
            print("There is no " + record_oss_path )
            return modified_content


        ret_first_second_record_path = (
            self.__generate_first_second_record_and_modify_time(
                record_oss_path,
                trigger_time,
                simulation_start_timestamp,
                simulation_end_timestamp,
            )
        )

        if ret_first_second_record_path["second_record"] != "":
            record_oss_path = (
                str(ret_first_second_record_path["first_record"])
                + '"'
                + "\n"
                + '  record_path: "'
                + str(ret_first_second_record_path["second_record"])
            )

        trigger_time = ret_first_second_record_path["trigger_time"]
        simulation_start_timestamp = ret_first_second_record_path[
            "simulation_start_timestamp"
        ]
        simulation_end_timestamp = ret_first_second_record_path[
            "simulation_end_timestamp"
        ]
        uid_input = self.__generate_int64_hash(scenario_name)

  
        for line in self._scenario_base_txt:
            line = line.replace("scenario_name", str(scenario_name))
            line = line.replace("uid_input", str(uid_input))
            line = line.replace("cyber_record_path_input", str(record_oss_path))
            line = line.replace("time_to_start_play_record", str(float(simulation_start_timestamp))) 
            line = line.replace("offset_seconds_to_begin_evalate", str(float(closed_loop_start_time_offset)))  
            line = line.replace("duration_input_seconds", str(duration_time))


            modified_content.append(line)
        return modified_content

    def generate_new_scenario_pbtxt(
        self, output_file: str, modified_content: str
    ) -> None:
        
        with open(output_file, "w", encoding="utf-8") as file:
            file.writelines(modified_content)
if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--scenarios_info_csv_filepath", help="Input CSV file path")
    parser.add_argument("-o", "--output_dirpath", help="Output scene file directory")
    args = parser.parse_args()


    assert os.path.exists(args.scenarios_info_csv_filepath) 

    
    if not os.path.exists(args.output_dirpath):
        os.makedirs(args.output_dirpath)


    scenarios_info_csv_filepath = args.scenarios_info_csv_filepath
    output_dirpath = args.output_dirpath
    dir_name = output_dirpath.split("/")[-1]

   
    generate_scenarios = ScenarioGeneration()
    generate_scenarios.read_csv_data(scenarios_info_csv_filepath)  
    scenario_list = generate_scenarios.generate_scenarios_name_list()    
    replace_info_list = generate_scenarios.generate_replace_info_list()  
    print("CSV data parsing completed!")

   
    for i in range(0, len(scenario_list), 1):
        new_scenario_info = generate_scenarios.generate_single_scenario(
            replace_info_list[i], scenario_list[i]
        )
        if new_scenario_info:
            scenario_txt_path = output_dirpath + "/" + str(scenario_list[i]) + ".pb.txt"
            generate_scenarios.generate_new_scenario_pbtxt(scenario_txt_path, new_scenario_info)
            scenario_pb_path_info = (
                'scenario_pb_path: "' + dir_name + "/" + str(scenario_list[i]) + ".pb.txt" + '"'
            )

            print(scenario_pb_path_info)
        else:
            print("Unable to create scene " + str(scenario_list[i]))

    print("Task completed!")