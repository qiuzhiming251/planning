import subprocess
import datetime
import sys
import os


class FillbackProcessor:

    def __init__(self):
        self.shell_script = '/apollo/modules/cnoa_pnc/planning/scripts/batch_fillback.sh'

    def get_start_and_end(self, file_path):
        """
        Retrieve the start and end time of the record.
        """
        result = subprocess.run(['cyber_recorder', 'info', file_path],
                                capture_output=True,
                                text=True)
        output = result.stdout

        begin_time = None
        end_time = None

        for line in output.split('\n'):
            if 'begin_time:' in line:
                begin_time_str = line.split(':', 1)[1].strip()
                begin_time = datetime.datetime.strptime(
                    begin_time_str, '%Y-%m-%d-%H:%M:%S').timestamp()
            elif 'end_time:' in line:
                end_time_str = line.split(':', 1)[1].strip()
                end_time = datetime.datetime.strptime(
                    end_time_str, '%Y-%m-%d-%H:%M:%S').timestamp()

        if begin_time is None or end_time is None:
            raise ValueError(
                "Begin or end time not found in cyber_recorder info output")

        return begin_time, end_time

    def process_fillback(self, file_path, target_dir, start, duration):
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)
            print(f"Directory created: {target_dir}")

        start = max(0, start)
        duration = float(duration)

        try:
            begin_time, end_time = self.get_start_and_end(file_path)
            epoch_start = begin_time

            if duration >= 0:
                start_time = epoch_start + start
                end_time = start_time + duration
            else:
                start_time = epoch_start + start
                end_time = end_time

        except ValueError as e:
            print(f"Warning: {e}. Using provided start time instead.")
            start_time = start
            end_time = start + duration if duration >= 0 else None

        start_datetime = datetime.datetime.fromtimestamp(start_time).strftime(
            '%Y-%m-%d-%H:%M:%S')
        end_datetime = datetime.datetime.fromtimestamp(end_time).strftime(
            '%Y-%m-%d-%H:%M:%S')

        files_before_fillback = set(os.listdir(target_dir))

        cmd = [
            "bash", self.shell_script, file_path, target_dir, start_datetime,
            end_datetime
        ]

        proc = subprocess.Popen(cmd,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT,
                                text=True)
        for line in proc.stdout:
            print(line, end='')
        proc.wait()

        files_after_fillback = set(os.listdir(target_dir))

        fillback_files = files_after_fillback - files_before_fillback
        fillback_files_with_prefix = [
            os.path.join(target_dir, f) for f in fillback_files
        ]

        return fillback_files_with_prefix


if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(
            "Usage: python fillback_processor.py <file> <target_dir> <start> <duration>"
        )
        sys.exit(1)

    file_path = sys.argv[1]
    target_dir = sys.argv[2]
    start = float(sys.argv[3])
    duration = float(sys.argv[4])

    processor = FillbackProcessor()
    fillback_files = processor.process_fillback(file_path, target_dir, start,
                                                duration)
    if fillback_files:
        print("Generated Fillback Files:")
        for file in fillback_files:
            print(file)
    else:
        print("No fillback files generated.")
