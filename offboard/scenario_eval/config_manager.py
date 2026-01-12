import argparse
import os


class ConfigManager:

    def parse_args(self):
        parser = argparse.ArgumentParser(description='Evaluator Integration')
        group = parser.add_mutually_exclusive_group(required=True)
        group.add_argument('-d',
                           '--metrics_directory',
                           help='Directory containing metric files')
        group.add_argument('-m',
                           '--metric_file',
                           help='Single metric file to process')
        parser.add_argument('-t', '--target_path', required=True)
        return parser.parse_args()

    def get_metrics_files(self, args):
        if args.metric_file:
            return [args.metric_file]
        elif args.metrics_directory:
            return [
                os.path.join(args.metrics_directory, f)
                for f in os.listdir(args.metrics_directory)
                if f.endswith(".pb.txt")
            ]
        else:
            raise ValueError(
                "Either --metrics_directory or --metric_file must be provided")


if __name__ == "__main__":
    #test
    config_manager = ConfigManager()

    args = config_manager.parse_args()

    metrics_files = config_manager.get_metrics_files(args)

    print("Arguments parsed:")
    print(f"Metrics Directory: {args.metrics_directory}")
    print(f"Metric File: {args.metric_file}")
    print(f"Data Path: {args.target_path}")
    print("Metrics Files to process:")
    for file in metrics_files:
        print(file)
    if not args.metric_file:
        print("Not Single file")
    if not args.metrics_directory:
        print("Not directory")
