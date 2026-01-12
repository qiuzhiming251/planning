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


class LaneChangeAnalyzer:
    def __init__(self, target_strings):
        self.target_strings = target_strings

    def analyze(self, messages):
        res = False
        events = []
        for target in self.target_strings:
            if target.startswith("!"):
                res = True
        
        for message in messages:
            parser = SelectorDebugStringParser(message['debug_strings'])
            parser.parse_data()
            parsed_dict = {key: value for key, value in parser.parsed_data}
            
            for target in self.target_strings:
                if target.startswith("!"):
                    if target[1:] in parsed_dict.values():
                        events.append({
                            'timestamp': message['timestamp'],
                            'debug_strings': message['debug_strings']
                        })
                        return False
                else:
                    
                    if target in parsed_dict.values():
                        events.append({
                            'timestamp': message['timestamp'],
                            'debug_strings': message['debug_strings']
                        })
                        res = True
                        return res
        
        return res


class LaneChangeExtractor:
    def __init__(self, record_path):
        self.record_path = record_path
        self.extractor = ChannelMessageExtractor(record_path)

    def _parse_message(self, msg, file_path):
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
                'debug_strings': selector_debug_list,
                'file_path': file_path
            }
        except Exception as e:
            print(f"Parsing message failed (timestamp {msg.timestamp}):{str(e)}")
            return None

    def extract_target_channel(self, file_path):
        return self.extractor.extract_and_parse("/st/pnc/planning_debugframe", lambda msg: self._parse_message(msg, file_path))

class LaneChangeProcesser:
    def process_file(self,record_path, target_strings):
        try:
            target_strings = [target_strings] if isinstance(target_strings, str) else target_strings
            processor = LaneChangeExtractor(record_path)
            parsed_data = processor.extract_target_channel(record_path)
            analyzer = LaneChangeAnalyzer(target_strings)
            matching_events = analyzer.analyze(parsed_data)
            
            if matching_events:
                return True
            else:
                return False
        except Exception as e:
            print(f"Error processing file {record_path}:{e}")
            return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Intersection Lane - Change Detection Tool')
    parser.add_argument('-d', '--directory', help='Enter the directory for the record files', required=True)
    parser.add_argument('-t', '--targets', nargs='+', help='Enter the target strings to search for', required=True)

    args = parser.parse_args()

    target_strings = args.targets

    # Files 2025-05-19_20-56-28.record.00066.22-02-28
    record_files = [os.path.join(args.directory, f) for f in os.listdir(args.directory) 
                    if re.search(r'record', f)]

    if not record_files:
        print("No record file matching the criteria found")
        exit(1)

    matching_files = []

    def check_file(file):
        try:
            lane_change_processer = LaneChangeProcesser()
            result = lane_change_processer.process_file(file, target_strings)

            if result:
                print(f"File {file} contains  target lane_change_type\n")
                return file
            else:
                print(f"File {file} does not contain  target lane_change_type.\n")
                return None
        except Exception as e:
            print(f"Error processing {file}：{e}\n")
            return file

    with ThreadPoolExecutor() as executor:
        results = list(executor.map(check_file, record_files))

    matching_files = [file for file in results if file]

    if matching_files:
        print(f"\nThe following files contain  lane_change_type.'{target_strings}':")
        for file in matching_files:
            print(file)
    else:
        print(f"\nNo files contain any of target lane_change_type. '{target_strings}'.")

    print(f"\nTotal files: {len(record_files)}, Files containing lane_change_type.: {len(matching_files)}")


