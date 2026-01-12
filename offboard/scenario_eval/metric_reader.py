
from modules.cnoa_pnc.planning.offboard.scenario_eval.proto.selector_scenatro_eval_pb2 import MetricMessage
class ConfigToProtoConverter:
    def __init__(self):
        pass

    def convert(self,config_dict):
        metric = MetricMessage()
        metric.name = config_dict['name']
        metric.uid = str(config_dict['uid'])

       
        data_config_dict = config_dict['data_config']
        metric.data_config.record_path = data_config_dict['record_path']
        try:
            metric.data_config.start_timestamp = float(data_config_dict.get('start_timestamp', -1.0))
        except ValueError:
            metric.data_config.start_timestamp = -1.0

        try:
            metric.data_config.eval_time_offset = float(data_config_dict.get('eval_time_offset', -1.0))
        except ValueError:
            metric.data_config.eval_time_offset  = -1.0
       
        try:
            metric.data_config.duration = float(data_config_dict.get('duration', -1.0))
        except ValueError:
            metric.data_config.duration  = -1.0


        
        metric_configs = config_dict['metric_eval_configs']
        for metric_config_dict in metric_configs:
            metric_eval_config = metric.metric_eval_configs.add()
            metric_eval_config.metric_id = metric_config_dict['metric_id']
            if_needed_value = metric_config_dict.get('if_needed', False)
            
            if not isinstance(if_needed_value, bool):
                if_needed_value = False
            metric_eval_config.if_needed = if_needed_value

          
            if metric_eval_config.metric_id == 'change_lane_type_eval':
                specific_config = metric_eval_config.change_lane_type_config
                specific_config_dict = metric_config_dict.get('change_lane_type_eval_config', {})
                specific_config = metric_eval_config.change_lane_type_config
                
             
                try:
                    specific_config.timestamp_begin_to_eval = float(specific_config_dict.get('timestamp_begin_to_eval', -1.0))
                except ValueError:
                    specific_config.timestamp_begin_to_eval = -1.0
                
               
                try:
                    specific_config.duration = float(specific_config_dict.get('duration', -1.0))
                except ValueError:
                    specific_config.duration = -1.0
                
                specific_config.target_type = specific_config_dict.get('target_type', '')
            elif metric_eval_config.metric_id == 'change_lane_intersection_eval':
                specific_config = metric_eval_config.change_lane_intersection_config
                specific_config.reserved = metric_config_dict['change_lane_intersection_eval_config'].get('reserved', '')
            elif metric_eval_config.metric_id == 'change_lane_backandforth_eval':
                specific_config = metric_eval_config.change_lane_backandforth_config
                specific_config.reserved = metric_config_dict['change_lane_backandforth_eval_config'].get('reserved', '')
            else:
                raise ValueError(f"Unkown metric_id: {metric_eval_config.metric_id}")

        return metric

class MetricReader:
    def __init__(self):
       pass

    def read_and_parse_file(self,file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
        return self.parse_lines(lines)

    def parse_lines(self, lines):
        config_dict = {}
        stack = [config_dict]
        current_dict = config_dict

        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            if line.endswith('{'):
                key = line[:-1].strip()
                new_dict = {}
                if key == 'metric_eval_configs':
                    if key not in current_dict:
                        current_dict[key] = []
                    current_dict[key].append(new_dict)
                else:
                    current_dict[key] = new_dict
                stack.append(current_dict)
                current_dict = new_dict
            elif line.endswith('}'):
                stack.pop()
                current_dict = stack[-1]
            else:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]  
                if value.lower() == 'true':
                    value = True
                elif value.lower() == 'false':
                    value = False
                else:
                    try:
                        value = float(value)
                        if value.is_integer():
                            value = int(value)
                    except ValueError:
                        pass
                current_dict[key] = value

        return config_dict

if __name__ == "__main__":
    parser = MetricReader()
    config = parser.read_and_parse_file('20250328_122604_HC25.pb.txt')
    print(config["data_config"]["record_path"])
    metric_configs = config["metric_eval_configs"]
    for config in metric_configs:
        print(f"Metric ID: {config['metric_id']}")
        print(f"If Needed: {config['if_needed']}")
        print(f"metric_config: {config}")

        if config['metric_id'] == 'change_lane_type_eval':
            change_lane_type_config = config.get('change_lane_type_eval_config', {})
            print(f"Timestamp Begin: {change_lane_type_config.get('timestamp_begin_to_eval')}")
            print(f"Duration: {change_lane_type_config.get('duration')}")
            print(f"Target Type: {change_lane_type_config.get('target_type')}")
        elif config['metric_id'] == 'change_lane_intersection_eval':
            pass
        elif config['metric_id'] == 'change_lane_backandforth_eval':
            pass
        else:
            print("ERROR!!!!")

    parser = MetricReader()
    config_dict = parser.read_and_parse_file('20250328_122604_HC25.pb.txt')

    
    converter = ConfigToProtoConverter()
    metric = converter.convert(config_dict)

    
    print(metric)
    for metric_eval_config in metric.metric_eval_configs:
            print(metric_eval_config)
            if metric_eval_config.if_needed:
                if metric_eval_config.metric_id == 'change_lane_type_eval':
                    specific_config = metric_eval_config.change_lane_type_config
                    print("metric_eval_config.change_lane_type_config",specific_config)
                elif metric_eval_config.metric_id == 'change_lane_intersection_eval':
                    print( metric_eval_config.metric_id)
                elif metric_eval_config.metric_id == 'change_lane_backandforth_eval':
                    print("special",metric_eval_config.metric_id)
                else:
                    raise ValueError(f"UNKOWN metric_id: {metric_eval_config.metric_id}")

