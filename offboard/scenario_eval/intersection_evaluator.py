'''

'''
import argparse
from concurrent.futures import ThreadPoolExecutor
from google.protobuf.json_format import MessageToDict

from modules.msg.st_msgs.planning_debug_frame_pb2 import DebugFrameProto
from message_parser.parse_selector_debug_string import SelectorDebugStringParser
from bazel_tools.tools.python.runfiles import runfiles

from common.common import TimestampConverter 
from common.common import ChannelMessageExtractor

import os
import re



class IntersectionAnalyzer:

    
    def analyze(self,messages):

        filtered_timestamps = []
        current_scene_start = None

        for message in messages:
            parser = SelectorDebugStringParser(message['debug_strings'])
            parser.parse_data()
            parsed_dict = {key: value for key, value in parser.parsed_data}

            if parsed_dict.get('ego_near_intersection') == '1':
                lane_change_type = parsed_dict.get('lane_change_type')
                if lane_change_type and lane_change_type != 'TYPE_NO_CHANGE':
                    if current_scene_start is None:
                        current_scene_start = message['timestamp']
                    
                    current_scene_end = message['timestamp']
                else:
                    if current_scene_start is not None:
                        filtered_timestamps.append({
                            'start_time': current_scene_start,
                            'end_time': current_scene_end
                        })
                        current_scene_start = None
            else:
                if current_scene_start is not None:
                    filtered_timestamps.append({
                        'start_time': current_scene_start,
                        'end_time': current_scene_end
                    })
                    current_scene_start = None
        
  
        if current_scene_start is not None:
            filtered_timestamps.append({
                'start_time': current_scene_start,
                'end_time': current_scene_end
            })

       
        for data in filtered_timestamps:
                
            start_time_bj = TimestampConverter.ns_to_bj_time(data['start_time'])
            end_time_bj = TimestampConverter.ns_to_bj_time(data['end_time'])
            print(f"There is lane-change scenario within intersections. Start_Time: {start_time_bj.strftime('%Y-%m-%d_%H-%M-%S')}, End_time: {end_time_bj.strftime('%Y-%m-%d_%H-%M-%S')}")

        return filtered_timestamps

class IntersectionExtractor:
    def __init__(self, record_path):
        self.record_path = record_path
        self.extractor = ChannelMessageExtractor(record_path)
        self.analyzer = IntersectionAnalyzer()

    def _parse_message(self, msg):
        try:
            pb_obj = DebugFrameProto()
            pb_obj.ParseFromString(msg.message)
            proto_dict = MessageToDict(
                pb_obj,
                preserving_proto_field_name=True,
                including_default_value_fields=True
            )
            string_lists = proto_dict.get('stringLists', [])
            selector_debug_list = []

            for item in string_lists:
                if isinstance(item, dict) and item.get('name', '').lower() == 'selector_debug_string':
                    raw_value = item.get('value', [])
                    if isinstance(raw_value, list):
                        selector_debug_list = raw_value
                    else:
                        selector_debug_list = [str(raw_value)]
                    break

            return {
                'timestamp': msg.timestamp,
                'debug_strings': selector_debug_list
            }
        except Exception as e:
            print(f"Parsing message failed (timestamp {msg.timestamp}):{str(e)}")
            return None

    def extract_target_channel(self):
        return self.extractor.extract_and_parse("/st/pnc/planning_debugframe", self._parse_message)

    def analysis(self, parsed_data):
        return self.analyzer.analyze(parsed_data)

class IntersectionProcesser:
    def process_file(self,record_path):
        try:
            processor = IntersectionExtractor(record_path)
            parsed_data = processor.extract_target_channel()
            lane_change_events = processor.analysis(parsed_data)
            
            if lane_change_events:
                return False
            else:
                return True
        except Exception as e:
            print(f"Error processing file {record_path}:{e}")
            return False  


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Intersection Lane - Change Detection Tool')
    parser.add_argument('-d', '--directory', help='Enter the directory for the record files')

    args = parser.parse_args()

    if not args.directory:
        print("Please provide the directory containing the record files.")
        exit(1)

    # Files 2025-05-19_20-56-28.record.00066.22-02-28
    record_files = [os.path.join(args.directory, f) for f in os.listdir(args.directory) 
                    if re.search(r'record', f)]

    if not record_files:
        print("No record file matching the criteria found")
        exit(1)

    failed_files = []

    def check_file(file):
        try:
            intersectionprocesser = IntersectionProcesser()
            success = intersectionprocesser.process_file(file)
            if not success:
                print(f"File {file} fails the check.\n")
                return file
            else:
                print(f"File {file} passed the check.\n")
                return None
        except Exception as e:
            print(f"Error processing {file}：{e}\n")
            return file

    with ThreadPoolExecutor() as executor:
        failed_results = list(executor.map(check_file, record_files))

    failed_files = [file for file in failed_results if file]

    if failed_files:
        print(f"\nThe following files failed the check:")
        for file in failed_files:
            print(file)
            
    else:
        print("\nAll files passed the check.")

    print(f"\nTotal files: {len(record_files)}, Files failed: {len(failed_files)}")