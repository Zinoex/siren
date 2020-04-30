import json

class Lane_Trajectory:
    # Lane Trajectories (sources or sinks) have a Leg and Lane 
    def __init__(self, leg_name: "string", lane_name: "string", type: "string"):
        self.leg_name = leg_name
        self.lane_name = lane_name
        self.type = type
        self.print_string = self.type + ": " + self.leg_name + ", " + self.lane_name 
    def __str__(self):
        return self.print_string

class Lane:
    # Each Lane has a source and sink
    def __init__(self, sink: "Lane_Trajectory", source: "Lane_Trajectory", name: "string", type: "string" = "vehicle", flow_rate: "float" = 1.0):
        self.name = name
        self.type = type
        self.flow_rate = flow_rate
        self.sink = sink
        self.source = source
        self.print_string = self.name + "\n\t\t" + \
            self.source.print_string + "\n\t\t" + \
            self.sink.print_string + "\n\t\t" + \
            "Lane Type: " + self.type + "\n\t\t" + \
            "Flow Rate: " + str(self.flow_rate)
    def __str__(self):
        return self.print_string

class Leg:
    # Each Leg has a list of Lane objects
    def __init__(self, lanes: "List[Lane]", name: "string"):
        self.name = name
        self.lanes = lanes
        self.print_string = self.name
        for lane in self.lanes:
            self.print_string +=  "\n\t" + lane.print_string
    def __str__(self):
        return self.print_string    

class Intersection:
    # Each Intersection has a list of Lane objects
    legs = []
    def __init__(self, filename: "string", intersection_name:""):
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
                source_obj = Lane_Trajectory(source["leg"], source["lane"], "Source")
                sink_obj = Lane_Trajectory(sink["leg"], sink["lane"], "Sink")
                lane_obj = Lane(sink_obj, source_obj, lane, type, flow_rate)
                lanes.append(lane_obj)

            leg_obj = Leg(lanes, leg)
            self.legs.append(leg_obj)
                
    def __str__(self):
        str = self.name
        for leg in self.legs:
            str += "\n" + leg.print_string
        return str

    def create_sumo_files(self):
        # TODO: implement
        pass

def main():
    simple_intersection = Intersection("sample_itersection.json", "8 Lane, no lefts")
    print(simple_intersection)

if __name__ == "__main__":
    main()