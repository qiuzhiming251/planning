import argparse
import traceback
from concurrent.futures import ThreadPoolExecutor
from google.protobuf.json_format import MessageToDict

from modules.msg.st_msgs.planning_result_pb2 import PLanningResultProto
from modules.msg.st_msgs.planning_debug_frame_pb2 import DebugFrameProto
from message_parser.parse_selector_debug_string import SelectorDebugStringParser
from bazel_tools.tools.python.runfiles import runfiles

from common.common import TimestampConverter  
from common.common import ChannelMessageExtractor

import os
import re

#Use light signal and lane_change_type in selector debug string to eval
class BackForthChangeFillAnalyzer:
    def __init__(self, time_threshold):
        self.time_threshold = time_threshold

    def analyze(self, messages):
        lane_change_events = []
        signals = []

        for message in messages:
            parser = SelectorDebugStringParser(message['debug_strings'])
            parser.parse_data()
            parsed_dict = {key: value for key, value in parser.parsed_data}

            if parsed_dict.get('TurnSignal'):
                turn_signal = parsed_dict.get('TurnSignal')
                timestamp = message['timestamp']
                lane_change_type = parsed_dict.get('lane_change_type')
                if turn_signal == '1':  
                    turn_signal_str = 'TURN_LIGHT_LEFT'
                elif turn_signal == '2':  
                    turn_signal_str = 'TURN_LIGHT_RIGHT'
                else:
                    turn_signal_str = 'TURN_LIGHT_NO'  

                signals.append({
                    'turn_light_request': turn_signal_str,
                    'timestamp': timestamp,
                    'lane_change_type':lane_change_type
                })

        recorded_start_timestamps = set()

        i = 0
        while i < len(signals):
            current_turn_light = signals[i]['turn_light_request']
            current_timestamp = signals[i]['timestamp']
            cunrent_lane_change_type = signals[i]['lane_change_type']
            
            if current_timestamp in recorded_start_timestamps or cunrent_lane_change_type == 'TYPE_NO_CHANGE':
                i += 1
                continue

            start_index = None
            start_consecutive_count = 0  
            for j in range(i - 1, -1, -1):
                prev_turn_light = signals[j]['turn_light_request']
                prev_lane_change_type = signals[j]['lane_change_type']
                
                if prev_lane_change_type == 'TYPE_NO_CHANGE':
                    continue

                if (current_turn_light == 'TURN_LIGHT_RIGHT' and prev_turn_light == 'TURN_LIGHT_LEFT') or (current_turn_light == 'TURN_LIGHT_LEFT' and prev_turn_light == 'TURN_LIGHT_RIGHT'):
                    
                    start_index = j
                    count = 0
                    
                    for k in range(j, -1, -1):
                        if signals[k]['turn_light_request'] == prev_turn_light:
                            count += 1
                        else:
                            break
                    
                    start_consecutive_count = count

                    break

            if start_index is not None and start_consecutive_count >= 5:
                end_index = i
                end_consecutive_count = 0
                for k in range(i, len(signals)):
                    if signals[k]['lane_change_type'] == 'TYPE_NO_CHANGE':
                        break
                    if signals[k]['turn_light_request'] == current_turn_light:
                        end_consecutive_count += 1
                        end_index = k
                    else:
                        break
                    if end_consecutive_count >= 5:
                        break

                if end_consecutive_count >= 5:
                    time_diff = (signals[end_index]['timestamp'] - signals[start_index]['timestamp']) / 1e9
                    if time_diff < self.time_threshold:
                        event_type = 'left_to_right' if signals[start_index]['turn_light_request'] == 'TURN_LIGHT_LEFT' else 'right_to_left'
                        start_time = TimestampConverter.ns_to_bj_time(signals[start_index]['timestamp']).strftime('%Y-%m-%d_%H-%M-%S')
                        end_time = TimestampConverter.ns_to_bj_time(signals[end_index]['timestamp']).strftime('%Y-%m-%d_%H-%M-%S')
                        
                        event_exists = any(event['start_timestamp'] == signals[start_index]['timestamp'] for event in lane_change_events)
                        if not event_exists:
                            lane_change_events.append({
                                'event': event_type,
                                'time_diff': time_diff,
                                'start_time': start_time,
                                'end_time': end_time,
                                'start_timestamp': signals[start_index]['timestamp'],
                                'end_timestamp': signals[end_index]['timestamp']
                            })
                            recorded_start_timestamps.add(signals[start_index]['timestamp'])  
                        i = end_index + 1
                    else:
                        i += 1
                else:
                    i += 1
            else:
                i += 1

        for event in lane_change_events:
            print(f"Type：{event['event']}, Start_Time：{event['start_time']}, End_Time：{event['end_time']}, Time_diff：{event['time_diff']:.3f} second")

        return lane_change_events


#use light signal in planning_result to eval
class BackForthChangeAnalyzer:
    def __init__(self, time_threshold):
        self.time_threshold = time_threshold
    
    def analyze(self, parsed_data):
        lane_change_events = []
        signals = [{'turn_light_request': data['turn_light_request'], 'timestamp': data['timestamp']} for data in parsed_data]

        recorded_start_timestamps = set()  

        i = 0
        while i < len(signals):
            current_turn_light = signals[i]['turn_light_request']
            current_timestamp = signals[i]['timestamp']

            if current_timestamp in recorded_start_timestamps:
                i += 1
                continue

            start_index = None
            start_consecutive_count = 0  
            for j in range(i - 1, -1, -1):
                prev_turn_light = signals[j]['turn_light_request']
                if (current_turn_light == 'TURN_LIGHT_RIGHT' and prev_turn_light == 'TURN_LIGHT_LEFT') or (current_turn_light == 'TURN_LIGHT_LEFT' and prev_turn_light == 'TURN_LIGHT_RIGHT'):
                    start_index = j
                    count = 0
                    for k in range(j, -1, -1):
                        if signals[k]['turn_light_request'] == prev_turn_light:
                            count += 1
                        else:
                            break
                    start_consecutive_count = count
                    break

            if start_index is not None and start_consecutive_count >= 5:
                end_index = i
                end_consecutive_count = 0
                for k in range(i, len(signals)):
                    if signals[k]['turn_light_request'] == current_turn_light:
                        end_consecutive_count += 1
                        end_index = k
                    else:
                        break
                    if end_consecutive_count >= 5:
                        break

                if end_consecutive_count >= 5:
                    time_diff = (signals[end_index]['timestamp'] - signals[start_index]['timestamp']) / 1e9
                    if time_diff < self.time_threshold:
                        event_type = 'left_to_right' if signals[start_index]['turn_light_request'] == 'TURN_LIGHT_LEFT' else 'right_to_left'
                        start_time = TimestampConverter.ns_to_bj_time(signals[start_index]['timestamp']).strftime('%Y-%m-%d_%H-%M-%S')
                        end_time = TimestampConverter.ns_to_bj_time(signals[end_index]['timestamp']).strftime('%Y-%m-%d_%H-%M-%S')
                        
                        event_exists = any(event['start_timestamp'] == signals[start_index]['timestamp'] for event in lane_change_events)
                        if not event_exists:
                            lane_change_events.append({
                                'event': event_type,
                                'time_diff': time_diff,
                                'start_timestamp': signals[start_index]['timestamp'],
                                'end_timestamp': signals[end_index]['timestamp'],
                                'start_time': start_time,
                                'end_time': end_time,
                            })
                            recorded_start_timestamps.add(signals[start_index]['timestamp'])  
                        i = end_index + 1
                    else:
                        i += 1
                else:
                    i += 1
            else:
                i += 1

        for event in lane_change_events:
            print(f"Type：{event['event']}, Start_Time：{event['start_time']}, End_Time：{event['end_time']}, Time_diff：{event['time_diff']:.3f} second")

        return lane_change_events

class BackForthChangeExtractor:
    def __init__(self, record_path, time_threshold):
        self.record_path = record_path
        self.time_threshold = time_threshold
        self.extractor = ChannelMessageExtractor(record_path)
        self.analyzer = BackForthChangeAnalyzer(time_threshold)

    def _parse_message(self, msg):
        try:
            pb_obj = PLanningResultProto()
            pb_obj.ParseFromString(msg.message)
            proto_dict = MessageToDict(
                pb_obj,
                preserving_proto_field_name=True,
                including_default_value_fields=True
            )
            plan_states = proto_dict.get('state', {})
            turn_light = plan_states.get('turn_light_request', 0)
            timestamp = msg.timestamp
            return {
                'turn_light_request': turn_light, 
                'timestamp': timestamp
            }
        except Exception as e:
            print(f"Parsing message failed (timestamp {msg.timestamp}): {str(e)}")
            return None

    def extract_target_channel(self):
        return self.extractor.extract_and_parse("/st/pnc/pilot_planning_result", self._parse_message)

    def analysis(self, parsed_data):
        return self.analyzer.analyze(parsed_data)

class BackForthChangeFillbackExtractor:
    def __init__(self, record_path, time_threshold):
        self.record_path = record_path
        self.time_threshold = time_threshold
        self.extractor = ChannelMessageExtractor(record_path)
        self.analyzer = BackForthChangeFillAnalyzer(time_threshold)

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
                print(f"Parsing message failed (timestamp {msg.timestamp}): {str(e)}")
                return None

    def extract_target_channel(self):
        return self.extractor.extract_and_parse("/st/pnc/planning_debugframe", self._parse_message)

    def analysis(self, parsed_data):
        return self.analyzer.analyze(parsed_data)
class BackForthChangeProcesser:
    def process_file(self,record_path, time_threshold, check_type):
        try:
            if check_type == '1':
                processor = BackForthChangeFillbackExtractor(record_path, time_threshold) # 
            elif check_type == '2':
                processor = BackForthChangeExtractor(record_path, time_threshold)         # strict
            parsed_data = processor.extract_target_channel()
            lane_change_events = processor.analysis(parsed_data)
        
            if lane_change_events:
                return False
            else:
                return True
            
        except Exception as e:
            print(f"Error processing file {record_path}: {e}")
            return False  


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='back_and_forth_lane_change_checker')
    parser.add_argument('-d', '--directory',  help='Enter the directory for the record files')
    parser.add_argument('-t', '--threshold',  type=float, help='Time threshold (unit: seconds)', default=10.0)
    parser.add_argument('-c', '--check_type', help='Input 1 uses TurnSignal detection in debugFrame. Input 2 uses planningResult.', default='1')

    args = parser.parse_args()

    if not args.directory:
        print("Please provide the directory containing the record files.")
        exit(1)

    time_threshold = args.threshold
    check_type     = args.check_type
    record_files = [os.path.join(args.directory, f) for f in os.listdir(args.directory)  
                    if re.search(r'record', f)]


    if not record_files:
        print("No record file matching the criteria found")
        exit(1)

    failed_files = []

    def check_file(file):
        try:
            back_forth_change_processer = BackForthChangeProcesser()
            success = back_forth_change_processer.process_file(file, time_threshold, check_type)
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
        print(f"\n The following files failed the check:：")
        for file in failed_files:
            print(file)
    else:
        print("\n All files passed the check.")

    print(f"\nTotal files: {len(record_files)}, Files failed: {len(failed_files)}")