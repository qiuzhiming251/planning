import os

from bazel_tools.tools.python.runfiles import runfiles

from metric_reader import MetricReader, ConfigToProtoConverter
from modules.cnoa_pnc.planning.offboard.scenario_eval.back_and_forth_evaluator import BackForthChangeProcesser
from modules.cnoa_pnc.planning.offboard.scenario_eval.intersection_evaluator import IntersectionProcesser
from modules.cnoa_pnc.planning.offboard.scenario_eval.lane_change_evaluator import LaneChangeProcesser
from modules.cnoa_pnc.planning.offboard.scenario_eval.proto.selector_scenatro_eval_pb2 import MetricMessage


from config_manager import ConfigManager
from fillback_processor import FillbackProcessor
from record_data_fetcher import DataSourceFactory
from result_reporter import Reporter



def process_scenario_test(fillback_record_files, metric_proto):
    file_results = {}

    for fillback_record in fillback_record_files:

        for metric_eval_config in metric_proto.metric_eval_configs:

            if metric_eval_config.if_needed:

                if metric_eval_config.metric_id == 'change_lane_type_eval':
                    lane_change_eval_processer = LaneChangeProcesser()
                    specific_config = metric_eval_config.change_lane_type_config
                    result = lane_change_eval_processer.process_file(
                        fillback_record, specific_config.target_type)
                    file_results['change_lane_type_eval'] = result
                elif metric_eval_config.metric_id == 'change_lane_intersection_eval':
                    lane_change_eval_processer = IntersectionProcesser()
                    result = lane_change_eval_processer.process_file(
                        fillback_record)
                    file_results['change_lane_intersection_eval'] = result
                elif metric_eval_config.metric_id == 'change_lane_backandforth_eval':
                    lane_change_eval_processer = BackForthChangeProcesser()
                    result = lane_change_eval_processer.process_file(
                        fillback_record, 10.0, '1')
                    file_results['change_lane_backandforth_eval'] = result
                else:
                    raise ValueError(
                        f"UNKOWN metric_id: {metric_eval_config.metric_id}")

    return file_results



if __name__ == "__main__":

    #Initialization
    config_manager = ConfigManager()
    cloud_fetcher = DataSourceFactory.get_fetcher('cloud')
    local_fetcher = DataSourceFactory.get_fetcher('local')
    fillback_processor = FillbackProcessor()
    report = Reporter()
    
    #about message 
    metric_reader = MetricReader()
    converter = ConfigToProtoConverter()
    metric_proto = MetricMessage()
    
    fillback_record_files = []
    overall_results = {}

    #Get metrics
    args = config_manager.parse_args()
    metrics_files = config_manager.get_metrics_files(args)
   
    #get target_dir
    target_dir = args.target_path
    output_file = os.path.join(target_dir,  "result.csv")
    #about result report
    write_to_file = True
    if not args.metrics_directory:
        write_to_file = False
    
    for metric_file in metrics_files:

        metric_data = metric_reader.read_and_parse_file(metric_file)
        metric_proto = converter.convert(metric_data)
        record_path = metric_proto.data_config.record_path
       
        if record_path.startswith("oss://"):

            result = cloud_fetcher.fetch_data(record_path, target_dir)
            record_path = os.path.join(target_dir, os.path.basename(record_path))

        else:

            result = local_fetcher.fetch_data(record_path)

        if result:
            fillback_record_file = fillback_processor.process_fillback(
                record_path, target_dir,
                metric_proto.data_config.eval_time_offset,
                metric_proto.data_config.duration)
            
            eval_result = process_scenario_test(fillback_record_file,
                                                metric_proto)
         
            overall_results[metric_file] = eval_result

    if overall_results:
        report.report_results(overall_results,
                              write_to_file,
                              output_file)
