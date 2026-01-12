import os
import subprocess


class DataSourceFactory:

    @staticmethod
    def get_fetcher(data_source):
        if data_source == 'cloud':
            return CloudDataFetcher()
        elif data_source == 'local':
            return LocalDataFetcher()
        else:
            raise ValueError("Unsupported data source")


class LocalDataFetcher:

    def fetch_data(self, source_path, target_directory=None):
        if not os.path.isfile(source_path):
            print(f"Local file {source_path} does not exist.")
            return False
        else:
            print(f"File {source_path} is valid.")

            return True


class CloudDataFetcher:

    def fetch_data(self, source_path, target_directory):
        if not os.path.exists(target_directory):
            os.makedirs(target_directory)

        cmd = [
            "ossutil", "--force-path-style", "cp", "-r", "-u", "-f",
            "--exclude", "*.mp4", source_path, target_directory
        ]

        env = os.environ.copy()
        env["OSS_PROGRESS"] = "1"
        env["OSS_LOGLEVEL"] = "info"

        proc = subprocess.Popen(cmd,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT,
                                text=True,
                                env=env)
        for line in iter(proc.stdout.readline, ""):
            print(line, end='')
        proc.wait()
        return proc.returncode == 0


if __name__ == "__main__":
    target_directory = "/apollo/modules/cnoa_pnc/planning/offboard/scenario_eval"

    # local
    local_source = "2025-08-27_10-29-49-149.record"
    result = DataSourceFactory.get_fetcher('local').fetch_data(
        local_source, target_directory)
    print(f"Local data fetch result: {result}")

    # oss
    cloud_source = "oss://oss-byd-wl-roadtest/PNC/CNOA/0904/2025-09-04_20-33-19/2025-09-04_21-23-22.record.00089.22-52-15"
    result = DataSourceFactory.get_fetcher('cloud').fetch_data(
        cloud_source, target_directory)
    print(f"Cloud data fetch result: {result}")
