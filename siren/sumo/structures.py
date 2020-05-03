import json

from typing import List


class LaneTrajectory:
    # Lane Trajectories (sources or sinks) have a Leg and Lane 
    def __init__(self, leg_name: str, lane_name: str, type: str):
        self.leg_name = leg_name
        self.lane_name = lane_name
        self.type = type

    def __str__(self):
        return self.type + ": " + self.leg_name + ", " + self.lane_name


class Lane:
    # Each Lane has a source and sink
    def __init__(self, sink: LaneTrajectory, source: LaneTrajectory, name: str,
                 ltype: str = "vehicle", flow_rate: float = 1.0):
        self.name = name
        self.ltype = ltype
        self.flow_rate = flow_rate
        self.sink = sink
        self.source = source

    def __str__(self):
        return self.name + "\n\t\t" + \
               str(self.source) + "\n\t\t" + \
               str(self.sink) + "\n\t\t" + \
               "Lane Type: " + self.ltype + "\n\t\t" + \
               "Flow Rate: " + str(self.flow_rate)


class Leg:
    # Each Leg has a list of Lane objects
    def __init__(self, lanes: List[Lane], name: str):
        self.name = name
        self.lanes = lanes

    def __str__(self):
        return self.name + "\n\t" + "\n\t".join(map(str, self.lanes))


class Intersection:
    # Each Intersection has a list of Lane objects
    legs = []

    def __init__(self, filename: str, intersection_name: str):
        self.name = intersection_name
        file = open(filename)
        intersection_data = json.load(file)
        self.parse_json(intersection_data)

    def parse_json(self, intersection_data: "json dictionary"):
        for leg in intersection_data["legs"]:
            lanes = []
            for lane in intersection_data["legs"][leg]["lanes"]:
                source = intersection_data["legs"][leg]["lanes"][lane]["source"]
                sink = intersection_data["legs"][leg]["lanes"][lane]["sink"]
                type = intersection_data["legs"][leg]["lanes"][lane]["type"]
                flow_rate = intersection_data["legs"][leg]["lanes"][lane]["flow_rate"]
                source_obj = LaneTrajectory(source["leg"], source["lane"], "Source")
                sink_obj = LaneTrajectory(sink["leg"], sink["lane"], "Sink")
                lane_obj = Lane(sink_obj, source_obj, lane, type, flow_rate)
                lanes.append(lane_obj)

            leg_obj = Leg(lanes, leg)
            self.legs.append(leg_obj)

    def __str__(self):
        pstr = self.name
        for leg in self.legs:
            pstr += "\n" + leg.print_string
        return pstr

    def create_sumo_files(self):
        # TODO: implement
        pass


def main():
    simple_intersection = Intersection("sample_itersection.json", "8 Lane, no lefts")
    print(simple_intersection)


if __name__ == "__main__":
    main()
