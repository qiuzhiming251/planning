import traceback
import datetime
from thirdparty.record import RecordReader,RecordWriter
from concurrent.futures import ThreadPoolExecutor

class ChannelMessageExtractor:
    def __init__(self, record_path):
        self.record_path = record_path

    def extract_and_parse(self, channel, parse_func):
        parsed_data = []
        try:
            reader = RecordReader(self.record_path)
            if channel not in reader.get_channellist():
                raise ValueError(f"There is no {channel} in cyber data!!")

            with ThreadPoolExecutor() as executor:
                futures = []
                for msg in reader.read_messages():
                    if msg.topic == channel:
                        futures.append(executor.submit(parse_func, msg))
                for future in futures:
                    parsed_dict = future.result()
                    if parsed_dict:
                        parsed_data.append(parsed_dict)
        except Exception as e:
            print(f"\nInterrupted while processing channel {channel}")
            traceback.print_exc()
        return parsed_data

class TimestampConverter:
    @staticmethod
    def ns_to_bj_time(timestamp):
        time_ns = timestamp
        time_s = time_ns // 1e9
        time_ms = (time_ns // 1e6) % 1e3
        utc_time = datetime.datetime.utcfromtimestamp(time_s)
        utc_time_with_ms = utc_time.replace(microsecond=int(time_ms) * 1000)
        bj_time = utc_time_with_ms + datetime.timedelta(hours=8)
        return bj_time
