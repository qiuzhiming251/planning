import csv
from abc import ABC, abstractmethod


class ResultWriter(ABC):

    @abstractmethod
    def write(self, results):
        pass


class CSVResultWriter(ResultWriter):

    def __init__(self, output_file):
        self.output_file = output_file

    def write(self, results):
        if not results:
            print("No results to write.")
            return

        with open(self.output_file, 'w', newline='') as csvfile:
            fieldnames = ["File"] + list(results[next(iter(results))].keys())
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for filename, result in results.items():
                writer.writerow({"File": filename, **result})
        print(f"Results written to {self.output_file}")


class TerminalResultWriter(ResultWriter):

    def write(self, results):
        if not results:
            print("No results to write.")
            return

        fieldnames = ["File"] + list(results[next(iter(results))].keys())
        print(", ".join(fieldnames))
        for filename, result in results.items():
            row = [filename] + [str(result[field]) for field in fieldnames[1:]]
            print(", ".join(row))


class ResultWriterFactory:

    @staticmethod
    def get_writer(write_to_file, output_file=None):
        if write_to_file:
            if output_file is None:
                raise ValueError(
                    "Output file must be provided when writing to file.")
            return CSVResultWriter(output_file)
        else:
            return TerminalResultWriter()


class Reporter:

    def __init__(self):
        self.writer_factory = ResultWriterFactory()

    def report_results(self, results, write_to_file, output_file=None):
        writer = self.writer_factory.get_writer(write_to_file, output_file)
        writer.write(results)


if __name__ == "__main__":
    results = {
        "20250429_220243_HC25.pb.txt": {
            "change_lane_type_eval": False,
            "change_lane_intersection_eval": True,
            "change_lane_backandforth_eval": True
        },
        "20250429_222021_HC25.pb.txt": {
            "change_lane_type_eval": False,
            "change_lane_intersection_eval": True,
            "change_lane_backandforth_eval": False
        },
        "20250727_225553_MC.pb.txt": {
            "change_lane_type_eval": True,
            "change_lane_intersection_eval": True,
            "change_lane_backandforth_eval": False
        },
        "20250426_145215_HC25.pb.txt": {
            "change_lane_type_eval": False,
            "change_lane_intersection_eval": True,
            "change_lane_backandforth_eval": True
        }
    }

    print("Test csv reporter")

    report = Reporter()
    report.report_results(results, write_to_file=True, output_file="output.csv")
    print("-----------------------------------------------------------")
    print("Test terminal reporter")

    report.report_results(results,
                          write_to_file=False,
                          output_file="output_1.csv")
