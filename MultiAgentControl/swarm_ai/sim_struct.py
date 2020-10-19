# import setup_path
# import airsim

import numpy as np
import pprint
import time
import copy
import traceback
import boto3
import sys
import json

from utils.distance_utils import haversine
from swarm import Swarm
from scenario import Scenario
from control import Control
from drone import Drone


def enable_control(client, drones: list) -> None:
    for vehicle in list(drones):
        client.enableApiControl(True, vehicle)
        client.armDisarm(True, vehicle)


def disable_control(client, drones: list) -> None:
    for vehicle in list(drones):
        client.armDisarm(False, vehicle)
        client.enableApiControl(False, vehicle)


def takeoff(client, drones: list) -> None:
    """
       Make all vehicles takeoff, one at a time and return the
       pointer for the last vehicle takeoff to ensure we wait for
       all drones
    """
    vehicle_pointers = []
    for drone_name in list(drones):
        vehicle_pointers.append(client.takeoffAsync(vehicle_name=drone_name))
    # All of this happens asynchronously. Hold the program until the last vehicle
    # finishes taking off.
    return vehicle_pointers[-1]


def get_all_drone_positions(client, drones: dict) -> np.array:
    for i, drone_name in enumerate(list(drones)):
        state_data = client.getMultirotorState(vehicle_name=drone_name)
        drones[drone_name].pos_vec3 = position_to_list(state_data.kinematics_estimated.position)
        drones[drone_name].gps_pos_vec3 = gps_position_to_list(state_data.kinematics_estimated.position)


def update_communication_matrix(client, comm_matrix: np.array, drones: np.array) -> bool:
    for i, parameter_list in enumerate(comm_matrix):
        for j, individual_list in enumerate(parameter_list):
            if i != j:
                # i will give you the current drone, e.g. "A"
                # j will give you the comparison drone, e.g. "B, C, ..."
                comparison_drone = drones[find_name(j)].gps_pos_vec3
                # Get comms data for the drone we want "A" in relation to the comparison drone "B, C, ..."
                comm_matrix[i, j] = client.getCommunicationsData(
                    comparison_drone[0], # latitude
                    comparison_drone[1], # longitude
                    comparison_drone[2], # altitude
                    vehicle_name=find_name(i)).can_communicate
            else:
                comm_matrix[i, j] = True


def position_to_list(position_vector) -> list:
    return [position_vector.x_val, position_vector.y_val, position_vector.z_val]


def gps_position_to_list(gps_vector) -> list:
    return [gps_vector.latitude, gps_vector.longitude, gps_vector.altitude]



def propagate_coordinates(client, comm_matrix: np.array, drones: np.array):
    for i, drone_comm_params in enumerate(comm_matrix):
        for j, individual_param in enumerate(drone_comm_params):
            if comm_matrix[i, j] == True and len(drones[find_name(i)].swarm_positions) < len(drones):
                drones[find_name(i)].swarm_positions.append(drones[find_name(j)].pos_vec3)


"""
[[[cog
import cog

with open(FILENAME, 'r') as f:
    for line in f:
        cog.out(line)
    cog.outl()
]]] """
#[[[end]]]


def fly_to_new_positions(client, drones: list) -> None:
    for drone_name in enumerate(list(drones)):
        z_coord = ensure_negative_z_coordinates(drones[drone_name].pos_vec3[2])
        client.moveToPositionAsync(
            drones[drone_name].pos_vec3[0], drones[drone_name].pos_vec3[1], z_coord, 3, vehicle_name=drone_name)
        time.sleep(0.1)


def ensure_negative_z_coordinates(z_value):
    if z_value > 0:
        return z_value * -1
    else:
        return z_value


def transform_to_standard_basis_coordinates(new_position: list, vehicle_offsets: list) -> list:
    # Each drone reports its position in the x,y,z plane relative to it's own starting position,
    # which is relative to the PlayerStart position in Unreal Engine. You must compensate the reported
    # position of each drone with it's starting position relative to the standard basis to properly calculate
    # the average position for each drone.
    new_position = new_position
    new_position[0] += vehicle_offsets[0]
    new_position[1] += vehicle_offsets[1]
    new_position[2] += vehicle_offsets[2]
    return new_position


def transform_to_relative_basis_coordinates(new_position: list, vehicle_offsets: list) -> list:
    # Transform the coordinates of each drone their independent representation frames.
    new_position = new_position
    new_position[0] -= vehicle_offsets[0]
    new_position[1] -= vehicle_offsets[1]
    new_position[2] -= vehicle_offsets[2]
    return new_position


def build_vehicle_distance_matrix(drones: dict,
                                  distance_matrix) -> bool:
    for i, row in enumerate(distance_matrix):
        for j, column in enumerate(row):
            if i != j:
                first_drone = drones[find_name(i)].gps_pos_vec3
                second_drone = drones[find_name(i)].gps_pos_vec3
                distance_matrix[i, j] = round(haversine(
                    first_drone[0],
                    first_drone[1],
                    second_drone[0],
                    second_drone[1])*1000, 3)
            else:
                distance_matrix[i, j] = 0


def find_name(numb: int) -> str:
    name = "A"
    j = ord(name[0])
    j += numb
    return chr(j)


def evaluate_together_matrix(distance_matrix: list) -> bool:
    """
        Given a matrix containing all of the distances between each drone,
        evaluate each cell to see if the drones are within some distance
        between each other.

        Input: distance_matrix - i x j matrix - each cell is the Haversine distance
               between the ith and jth drone.

        Output: together - i x j matrix - boolean matrix with each cell representing
                whether the ith and jth drones are within the final separation distance.
    """
    together = copy.deepcopy(
        distance_matrix)  # Ensure the matrix doesn't copy the original matrix.
    return together < final_separation_distance


# ====================================================================================================== #
# Start of main process
# ====================================================================================================== #

# Generate a set of drones based upon a given number input and number of swarms.
# Convention: Capital Letter = Drone Swarm Number = Number of drone in that swarm
# Ex: A1, A2, A3, etc.
# Load list of parameters into the system -> Some sort of class module to set all of these for me.


# Load vehicle names as a list for easy iteration.
# TO DO: This will be drawn from the parameters file loading (Rules sheet)

file_key = sys.argv[1]
bucket_name = 'swarmsimulations'

s3 = boto3.resource('s3')
sim_settings = s3.Object(bucket_name, file_key)
sim_settings = sim_settings.get()['Body'].read().decode('utf-8')
sim_settings = json.loads(sim_settings)

scenario = Scenario(id=sim_settings["Algorithm"]["Scenario"]["scenario_id"],
                        weather=sim_settings["Algorithm"]["Scenario"]["weather_effects"])
swarm = Swarm()
swarm.add_n_drones(sim_settings["Algorithm"]["Swarm"]["number_of_drones"])

name = "A"
for i, drone in enumerate(swarm.drones):
    drone.name = name
    j = ord(name[0])
    j += 1
    name = chr(j)

control = Control(swarm, scenario)

with open('tmp/settings.json', 'r') as f:
    settings = json.load(f)

for drone in swarm.drones:
    positions = settings["Vehicles"][drone.name]
    drone.starting_pos_vec3[0] = positions['X']
    drone.pos_vec3[0] = positions['X']
    drone.starting_pos_vec3[1] = positions['Y']
    drone.pos_vec3[1] = positions['Y']
    drone.starting_pos_vec3[2] = positions['Z']
    drone.pos_vec3[2] = positions['Z']

vehicle_names = [ drone.name for drone in swarm.drones ]

time_step = 5  # seconds
final_separation_distance = 10  # meters

# We want a matrix to track who can communicate with who!
# It should be a nxn matrix, with each drone tracking itself and the matrix looks like
#            drone_1 drone_2 ... drone n
# drone_1    true    false   ... true
# drone_2    false   true    ... true
# drone_n    false   false   ... true
communications_tracker = np.zeros(
    (len(vehicle_names), len(vehicle_names)), dtype=bool)

# We mimic the memory bank of a drone, tracking the relative positions.
# It should be a n-length vector, with each drone tracking itself and the matrix looks like

# drone_1 drone_2 ... drone n
# [x,y,z] [x,y,z] ... [x,y,z]

try:
    # client = airsim.MultirotorClient()
    client = None
    print(client)
    client.confirmConnection()
    print('Connection Confirmed')

    enable_control(client, vehicle_names)

    # airsim.wait_key('Press any key to takeoff')
    last_vehicle_pointer = takeoff(client, vehicle_names)
    # We wait until the last drone is off the ground
    last_vehicle_pointer.join()

    # airsim.wait_key('Press any key to rendevous the drones!')
    start_time = time.time()
    stop_matrix = np.zeros((len(vehicle_names)), dtype=bool)
    while not control.objective_met:
        # Get initial locations
        get_all_drone_positions(client, control.swarm)

        print("="*50)
        for drone in list(control.swarm.drones):
            print('\n', drone, control.swarm.drones[drone].pos_vec3, control.swarm.drones[drone].gps_pos_vec3)

        update_communication_matrix(client, communications_tracker, control.swarm.drones)

        for i, drone_name in enumerate(list(swarm.drones)):
            swarm.drones[drone_name].pos_vec3 = transform_to_standard_basis_coordinates(swarm.drones[drone_name].pos_vec3,
                                                                               swarm.drones[drone_name].starting_pos_vec3)

        # Propagate location to drones that can communicate
        propagate_coordinates(client, communications_tracker, control.swarm.drones)
        
        # This state vector will be modified in the future to track something other then states!
        for i, drone_name in enumerate(list(swarm.drones)):
            control.state_vector[drone_name] = control.swarm.drones[drone_name].pos_vec3
        
        # Will need to add in a selection of communications!

        """
        [[[cog
            import cog
            import re

            import sys
            
            with open(FILENAME, 'r') as f:
                for i, line in enumerate(f):
                    if i == 0:
                        function_title = line
                        break
            function_title = re.sub('def', '', function_title)
            function_title = re.sub('state_vector', 'control.state_vector', function_title)
            function_title = re.sub(':', '', function_title)
            cog.outl(function_title)
        ]]] """

        #[[[end]]]

        for i, drone in enumerate(list(swarm.drones)):
            control.swarm.drones[drone_name].pos_vec3 = control.state_vector[drone_name]

        for i, drone in enumerate(swarm.drones):
            swarm.drones[drone_name].pos_vec3 = transform_to_relative_basis_coordinates(control.swarm.drones[drone_name].pos_vec3,
                                                                       control.swarm.drones[drone_name].starting_pos_vec3)

        print("="*50)
        for drone in list(control.swarm.drones):
            print('\n', drone, control.swarm.drones[drone].pos_vec3, control.swarm.drones[drone].gps_pos_vec3)

        control.evaluate_state_vector()

        if not control.objective_met:
            fly_to_new_positions(client, control.swarm.drones)

        time.sleep(time_step)
        print("="*50)
        print('\n')

    end_time = time.time()
    total_time = end_time - start_time
    minutes = round(total_time / 60, 3)
    seconds = (total_time / 60) - minutes
    print("Total Time: {mins} mins {secs} secs".format(
        mins=minutes, secs=seconds))
    # airsim.wait_key('Press any key to reset to original state')
    client.reset()
except Exception:
    traceback.print_exc()
finally:
    if client:
        client.reset()
        disable_control(client, vehicle_names)
    print("Finished!")
