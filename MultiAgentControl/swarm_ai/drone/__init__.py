# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 6th, 2020
# 
# Description: Drone class to hold all details of the individual drone.
# ===============================================================

class Drone():

    def __init__(self, name='A', drone_type='SimpleFlight', autopilot='SimpleFlight', weight=2):
        self.drone_type = drone_type    # SimpleFlight is the default for AirSim
        self.autopilot = autopilot      # Carrot-Stick method for AirSim
        self.weight = weight            # kilograms
        self.pos_vec3 = [0,0,0]         # X, Y, Z where Z is down
        self.gps_pos_vec3 = [0,0,0]     # Latitude, Longitude, Altitude
        self.starting_pos_vec3 = [0,0,0]# X, Y, Z where Z is down
        self.name = name
        self.swarm_positions = []


