import argparse
import time
import msgpack
from enum import Enum, auto
import numpy as np
import networkx as nx

# from planning_utils import a_star, heuristic, create_grid, prune_path
from planning_utils2 import a_star_graph, heuristic, create_grid_and_edges, closest_point

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5.0
        SAFETY_DISTANCE = 5.0

        self.target_position[2] = TARGET_ALTITUDE

        # Read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline()
            lat0 = first_line.split(',')[0].strip().split(' ')[1]
            lon0 = first_line.split(',')[1].strip().split(' ')[1]
        
        # Set home position to (lat0, lon0, 0)
        self.set_home_position(lat0, lon0, 0.0)

        # Retrieve current global position
        # global_position = (self._longitude, self._latitude, self._altitude)
 
        # Convert current global position (lon, lat, up) to a local position (north, east, down)
        # local_position = global_to_local(global_position, self.global_home)
        
        print('global home (lat, lon, alt) {}'.format(self.global_home))
        print('global position (lat, lon, alt) {}'.format(self.global_position))
        print('local position (nor, est, dn) {}'.format(self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        # grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
#        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Define starting point on the grid (this is just grid center)
#        grid_start = (-north_offset, -east_offset)

        # Convert start position to current position rather than map center
#        grid_start = (int(self.local_position[0])-north_offset, 
#                      int(self.local_position[1])-east_offset)

        # Adapt to set goal as latitude / longitude position and convert
#        goal_global = (-122.396270, 37.793330, 0.0)
        # goal_global = (-122.400344, 37.794996, 0.0)
#        goal_local  = global_to_local(goal_global, self.global_home)
#        grid_goal = (int(goal_local[0])-north_offset, 
#                     int(goal_local[1])-east_offset)

#        print('Local Start and Goal: ', grid_start, grid_goal)

        # Graph search using Voronoi
#        G = nx.Graph()
#        for e in edges:
#            p1 = e[0]
#            p2 = e[1]
#            dist = np.linalg.norm(np.array(p2) - np.array(p1))
#            G.add_edge(p1, p2, weight=dist)

#        start_ne_g = closest_point(G, grid_start)
#        goal_ne_g = closest_point(G, grid_goal)
#        print("Start localtion on graph:", start_ne_g)
#        print("Goal localtion on graph:", goal_ne_g)

        # path, _ = a_star_graph(G, heuristic, start_ne_g, goal_ne_g)
        # print("length of path: ", len(path))

        # Convert path to waypoints
        # waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path] 
        # takeoff_point = [grid_start[0] + north_offset, grid_start[1] + east_offset, TARGET_ALTITUDE, 0]
        # landing_point = [grid_goal[0] + north_offset, grid_goal[1] + east_offset, TARGET_ALTITUDE, 0]
        # waypoints.insert(0,takeoff_point)
        # waypoints.append(landing_point)
        # print(waypoints)

        waypoints =  [[-0.2388599999999883, 0.7684600000000046, 5, 0], 
                      [7.761140000000012, 0.7684600000000046, 5, 0], 
                      [26.427832022254165, 14.101811444467216, 5, 0], 
                      [32.83805789275101, 17.30692568579923, 5, 0], 
                      [35.7021033141174, 24.859333727697333, 5, 0], 
                      [36.189711051020424, 25.768460000000005, 5, 0],
                      [37.83806221301779, 29.614612766272217, 5, 0], [42.76114000000001, 35.768460000000005, 5, 0], [54.76114000000001, 55.768460000000005, 5, 0], [64.76114000000001, 65.76846, 5, 0], [74.76119749995632, 75.76851749995626, 5, 0], [87.26110072914287, 83.26845593749817, 5, 0], [88.09443749998121, 85.76846, 5, 0], [91.79816473938746, 107.99072343620196, 5, 0], [94, 103, 5, 0] ]
        waypoints = [[ int(i) for i in waypoint] for waypoint in waypoints]
        print(waypoints)
        # Set self.waypoints
        self.waypoints = waypoints

        # Send waypoints to sim
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
