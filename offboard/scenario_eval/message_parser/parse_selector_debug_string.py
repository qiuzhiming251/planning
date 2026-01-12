import csv
class SelectorDebugStringParser():
    def __init__(self, data):
        self.data = data
        self.parsed_data = []

    def parse_data(self):
        for line in self.data:
            if line.startswith("FindLastSelectedTrjectory"):
                if "last_state" in line:
                    parts = line.split(": ")
                    value = parts[-1].strip()
                    self.parsed_data.append(("last_state", value))
            elif line.startswith("ego_near_intersection"):
                parts = line.split(": ") # ego_near_intersection/ id lane_change_for_intersection_obs / id
                if len(parts) > 1:
                    ego_parts = parts[1].split()
                    if len(ego_parts) > 0:
                        self.parsed_data.append(("ego_near_intersection", ego_parts[0].strip()))
                    if len(ego_parts) > 1:
                        if len(parts) > 2:
                            self.parsed_data.append(("lane_change_for_intersection_obs", parts[2]))
            elif "best_traj_idx by_cost is" in line:
                parts = line.split(" is ")
                value = parts[-1].strip()
                self.parsed_data.append(("best_traj_idx", value))

            elif "cost_best_traj is same as last selector result" in line:
                parts = line.split(" success count is ")
                value = parts[-1].strip()
                self.parsed_data.append(("success_count", value))
            elif "last_lc_stage" in line and "best_lc_stage" in line:
                parts = line.split()
                if len(parts) > 9:
                    self.parsed_data.append(("last_lc_stage", parts[5]))
                    self.parsed_data.append(("best_lc_stage", parts[7]))
                    self.parsed_data.append(("lock_intersection", parts[9]))
            elif line.startswith("last_selected_idx:"):
                parts = line.split(": ")
                value = parts[-1].strip()
                self.parsed_data.append(("last_selected_idx", value))
            elif line.startswith("final_selected_idx:"):
                parts = line.split(": ")
                value = parts[-1].strip()
                self.parsed_data.append(("final_selected_idx", value))

            elif "Selector state" in line:
                if "lane_change_type" in line and "lane_change_general_type" in line:
                    lane_change_type_start = line.find("lane_change_type: ") + len("lane_change_type: ")
                    lane_change_type_end = line.find(" lane_change_general_type")
                    lane_change_type = line[lane_change_type_start:lane_change_type_end].strip()
                    self.parsed_data.append(("lane_change_type", lane_change_type))

                    lane_change_general_type_start = line.find("lane_change_general_type: ") + len("lane_change_general_type: ")
                    lane_change_general_type = line[lane_change_general_type_start:].strip()
                    self.parsed_data.append(("lane_change_general_type", lane_change_general_type))
            
            elif "TurnSignal" in line:
                parts = line.split(": ")
                value = parts[1].strip().split(" ")[0]
                self.parsed_data.append(("TurnSignal", value))

    def save_to_csv(self, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['Attribute', 'Value'])
            for row in self.parsed_data:
                writer.writerow(row)

if __name__ == "__main__":
    raw_data1 = [
        "FindLastSelectedTrjectory last_state: 1",
        "ego_near_intersection: 0 lane_change_for_intersection_obs: 0",
        "best_traj_idx by_cost is 1",
        "cost_best_traj is same as last selector result, success count is 23",
        "last_selected_idx: 1",
        "last_selected_idx: 1 best_traj_idx: 1 last_lc_stage: LCS_NONE best_lc_stage: LCS_EXECUTING lock_intersection: 0",
        "lc to lk , last_lc_stage: LCS_EXECUTING",
        "ego_corner_cross_line",
        "begin_lane_change_frame-1 successive_count: 23",
        " dont satisfy frame , keep last",
        "not ego_near_intersection,clear lock",
        "final_selected_idx: 0",
        "has_selected_intersection: 0",
        "is_going_force_route_change_left: -1 selected_idx: 1 last_selected_idx: 1",
        "TurnSignal: 2 TurnSignalReason: 1",
        "Selector state: lane_change_type: TYPE_DEFAULT_ROUTE_CHANGE lane_change_general_type: LCGT_ROUTE_CHANGE"
    ]

    raw_data2 = [
        'FindLastSelectedTrjectory last_state: -1',
        'ego_near_intersection: 1 lane_change_for_intersection_obs: 1',
        'best_traj_idx by_cost is 0',
        'Selector state: lane_change_type: TYPE_NO_CHANGE lane_change_general_type: LCGT_NO_CHANGE',
        'TurnSignal: left right'
    ]

    parser1 = SelectorDebugStringParser(raw_data1)
    parser1.parse_data()
    parser1.save_to_csv("result/test_selector_debug_string_parser1.csv")

    parser2 = SelectorDebugStringParser(raw_data2)
    parser2.parse_data()
    parser2.save_to_csv("result/test_selector_debug_string_parser2.csv")