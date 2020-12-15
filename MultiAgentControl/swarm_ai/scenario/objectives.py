# ===============================================================
# Created By: Tyler Fedrizzi
# Authors: Tyler Fedrizzi
# Created On: September 26th, 2020
# 
# Description: Objective functions.
# ===============================================================
import numpy as np
import math
class Objective():

    def __init__(self):
        pass

    def together(self, positions, state_vector):
        state_matrix = self.build_together_matrix(positions)
        matrix_sum = np.sum(state_matrix)
        if np.floor((matrix_sum / len(state_matrix[0])**2)) == 1:
            return True
        else:
            return False
    
    def build_together_matrix(self, positions):
        distance_matrix = np.zeros((len(positions), len(positions)), dtype=float)
        for i, row in enumerate(distance_matrix):
            for j, column in enumerate(row):
                if i != j:
                    first_drone = positions[self.find_name(i)].pos_vec3
                    second_drone = positions[self.find_name(j)].pos_vec3
                    distance_matrix[i, j] = math.sqrt((first_drone[0] - second_drone[0])**2 + 
                                                      (first_drone[1] - second_drone[1])**2 +
                                                      (first_drone[2] - second_drone[2])**2)
                else:
                    distance_matrix[i, j] = 0
        print(distance_matrix)
        return distance_matrix < 10

    def find_name(self, numb: int) -> str:
        name = "A"
        j = ord(name[0])
        j += numb
        return chr(j)