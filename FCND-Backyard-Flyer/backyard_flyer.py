"""
Backyard Flyer Project.
"""
import time
import numpy as np
from enum import Enum
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True

        # initial state
        self.flight_state = States.MANUAL

        # register all callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            # make a smooth curved turn by starting transit to next waypoint 
            # when it's less than 1.0 meter to the target position.
            # a smaller value will make a sharp turn, but it takes time 
            # for drone to stablize in the target location.
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    # transit to the next waypoint
                    self.waypoint_transition()
                else:
                    # if no more waypoint, prepare landing by making sure there's not much horizontal velocity
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                # chane from 0.01 to 0.05 to tolerate more altitude uncertainly,
                # so propeller wouldn't keep spinning on the ground 
                if abs(self.local_position[2]) < 0.05:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                # comment the following line, because self.armed is set to True 
                # when self.arm() command is issued in arming_transition() function
                # if self.armed:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                # comment the following line, because self.disarm() and 
                # self.release_control() commands are issued in disarming_transition() function
                # if ~self.armed & ~self.guided:
                self.manual_transition()

    def calculate_box(self):
        print("Setting Home")
        local_waypoints = [[20.0,  0.0,  5.0], 
                           [20.0, 20.0,  5.0], 
                           [ 0.0, 20.0,  5.0], 
                           [ 0.0,  0.0,  5.0]]
        return local_waypoints

    def arming_transition(self):
        print("arming transition")

        # the following code solve problem that drone go straing tinot trees after take off
        # curtersy of Mark70117,  https://github.com/udacity/fcnd-issue-reports/issues/96
        # begin added code to check self.global_position
        if self.global_position[0] == 0.0 and self.global_position[1] == 0.0:
            print("no global position data, wait")
            return
        # end added code

        self.take_control()
        self.arm()

        # set the current location to be the home position
        self.set_home_position(self.global_position[0], 
                               self.global_position[1],
                               self.global_position[2]) 

        self.flight_state = States.ARMING

    def takeoff_transition(self):
        print("takeoff transition")

        # flying at altitude 3 meters
        target_altitude = 6.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        print("waypoint transition")

        # get each way point in sequence
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        # the last parameters (heading) has no effect 
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        self.start_log("Logs", "NavLog.txt")
        # self.connect()
        print("starting connection")
        # self.connection.start()
        super().start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()