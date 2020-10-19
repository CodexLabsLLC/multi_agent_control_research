# ===============================================================
# Created By: Arpit Amin
# Authors: Arpit Amin
# Created On: September 7th, 2020
# 
# Description: Swarm class to hold all details of the swarm(s) that will
#              be used for the actual simulation.
# ===============================================================
from drone import Drone

class Swarm():

    def __init__(self):
        self.num_drones = 0
        self.drones = dict()

    def add_drone(self, drone_type='SimpleFlight'):
        self.num_drones += 1
        drone_name = self.create_name_for_drone(self.drones)
        drone = Drone(drone_type=drone_type, name=drone_name)
        self.drones[drone_name] = drone

    def add_n_drones(self, n, drone_type='SimpleFlight'):
        for _ in range (n):
            self.add_drone(drone_type=drone_type)
    
    def create_name_for_drone(self, drones):
        name = "A"
        for _ in range(len(self.drones)):
            j = ord(name[0])
            j += 1
            name = chr(j)
        return name