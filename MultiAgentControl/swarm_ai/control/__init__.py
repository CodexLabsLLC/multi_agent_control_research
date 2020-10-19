# ===============================================================
# Created By: Arpit Amin
# Authors: Arpit Amin
# Created On: September 7th, 2020
#
# Description: Control class to hold all details of the swarm controls that will
#              be used for the actual simulation.
# ===============================================================
import random


class Control():

    def __init__(self, swarm, scenario, min_separation_distance=5):
        """
            Currently, we are overwriting the __init__ function for
            the control class when we instantiate this model. This may
            change in the future.
        """
        self.segmentation = self.generate_semgementation("square", 4)
        self.map = scenario.map_boundary
        self.swarm = swarm
        self.scenario = scenario
        self.min_separation_distance = min_separation_distance
        self.state_vector = None
        self.objective_met = False

    def generate_semgementation(self, shape, number_divisions):
        print(shape)
        return 0
    
    def generate_state_vector(self):
        self.state_vector = { drone_name:self.swarm.drones[drone_name].pos_vec3 for drone_name in list(self.swarm.drones) }
        print(self.state_vector)

    def randomize_starting_positions(self):
        drones = self.swarm.drones
        for drone_name in list(drones):
            drones[drone_name].pos_vec3[2] = -5  # We fix the Z-Axis (Down in NED) to avoid terrain
            drones[drone_name].pos_vec3[1] = random.randrange(self.map["-Y"],
                                        self.map["+Y"],
                                        self.min_separation_distance)
            drones[drone_name].pos_vec3[0] = random.randrange(self.map["-X"],
                                        self.map["+X"],
                                        self.min_separation_distance)
            drones[drone_name].starting_pos_vec3 = drones[drone_name].pos_vec3        
    
    def evaluate_state_vector(self):
        self.objective_met = self.scenario.selection.evaluate(self.swarm.drones, self.state_vector)        

