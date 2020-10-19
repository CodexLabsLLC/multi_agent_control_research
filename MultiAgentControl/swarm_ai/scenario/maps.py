# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 26th, 2020
# 
# Description: Map classes to allow for easy generation.
# ===============================================================
from scenario.objectives import Objective


class Rendezvous(Objective):
    """
    The current map of the area is a square with area 200 meters^2. This will
    be a boundary limit we will use to stop drones from going absolutely crazy.
    """
    def __init__(self):
        self.pos_x_boundary = 200 # meters
        self.neg_x_boundary = -200 # meters
        self.pos_y_boundary = 200 # meters
        self.neg_y_boundary = -200 # meters
    
    def evaluate(self, state):
        return self.together(state)