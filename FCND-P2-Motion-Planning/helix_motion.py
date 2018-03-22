import argparse
import time
import msgpack
from enum import Enum, auto
import numpy as np
import networkx as nx

# from planning_utils import a_star, heuristic, create_grid, prune_path

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
            print("   local position", self.local_position)
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            print("   local position", self.local_position)
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 5.0:
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
            lat0 = float(first_line.split(',')[0].strip().split(' ')[1])
            lon0 = float(first_line.split(',')[1].strip().split(' ')[1])
        
        print(lat0, lon0)
        # Set home position to (lat0, lon0, 0)
        self.set_home_position(lon0,lat0, 0.0)

        # Retrieve current global position
        # global_position = (self._longitude, self._latitude, self._altitude)
 
        # Convert current global position (lon, lat, up) to a local position (north, east, down)
        # local_position = global_to_local(global_position, self.global_home)

        print('global home (lat, lon, alt) {}'.format(self.global_home))
        print('global position (lat, lon, alt) {}'.format(self.global_position))
        print('local position (nor, est, dwn) {}'.format(self.local_position))
 
        # goal_global = (-122.400835, 37.795407, 0.0)
        # waypoints =  [[[0, 0, 5, 0], [7, 0, 5, 0], [26, 14, 5, 0], [32, 17, 5, 0], [34, 15, 5, 0], [44, 5, 5, 0], [54, -1, 5, 0], [64, -6, 5, 0], [84, -11, 5, 0], [93, -15, 5, 0], [94, -16, 5, 0], [98, -17, 5, 0], [104, -24, 5, 0], [129, -39, 5, 0], [129, -54, 5, 0], [129, -64, 5, 0], [129, -64, 5, 0], [116, -84, 5, 0], [117, -91, 5, 0], [122, -104, 5, 0], [124, -114, 5, 0], [124, -124, 5, 0], [124, -134, 5, 0], [122, -146, 5, 0], [144, -154, 5, 0], [154, -154, 5, 0], [164, -154, 5, 0], [174, -157, 5, 0], [178, -160, 5, 0], [184, -161, 5, 0], [191, -159, 5, 0], [204, -164, 5, 0], [214, -164, 5, 0], [224, -164, 5, 0], [234, -164, 5, 0], [244, -167, 5, 0], [248, -170, 5, 0], [264, -174, 5, 0], [264, -174, 5, 0], [259, -194, 5, 0], [259, -201, 5, 0], [258, -204, 5, 0], [251, -222, 5, 0], [247, -225, 5, 0], [241, -244, 5, 0], [244, -254, 5, 0], [244, -279, 5, 0], [244, -299, 5, 0], [250, -306, 5, 0], [254, -307, 5, 0], [264, -309, 5, 0], [274, -309, 5, 0], [284, -309, 5, 0], [287, -309, 5, 0], [299, -317, 5, 0], [309, -315, 5, 0], [314, -304, 5, 0], [322, -300, 5, 0]]

        helix = []
        r = 20
        w = 20
        v_h = 1
        T = 100
        dt = 1
        for t in np.arange(0, T, dt):
            pt = [r*np.cos(np.deg2rad(w*t)), 
                  r*np.sin(np.deg2rad(w*t)),
                  TARGET_ALTITUDE + v_h*t,
                  0]
            helix.append(pt)

        waypoints = (np.array(helix) + np.append(self.local_position, [0])).astype(int).tolist()
        # waypoints.insert(0, [0,0,5,0])
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
