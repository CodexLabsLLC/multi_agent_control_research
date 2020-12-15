import setup_path
import airsim

import numpy as np
import pprint
import time
import copy
import traceback
import boto3
import sys
import json
import pandas as pd
import uuid

from datetime import datetime
from utils.distance_utils import haversine
from utils.date_utils import convert_datetime_to_str
from swarm import Swarm
from scenario import Scenario
from control import Control
from drone import Drone
from db import DB


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


def get_all_drone_positions(client, drones: list) -> np.array:
    for i, drone_name in enumerate(list(drones)):
        state_data = client.getMultirotorState(vehicle_name=drone_name)
        if debug:
            print(state_data)
            print(state_data.gps_location)
        drones[drone_name].pos_vec3 = position_to_list(state_data.kinematics_estimated.position)
        drones[drone_name].gps_pos_vec3 = gps_position_to_list(state_data.gps_location)
        drones[drone_name].current_velocity = gps_velocity_to_list(state_data.kinematics_estimated.linear_velocity)

"""
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
"""


def position_to_list(position_vector) -> list:
    return [position_vector.x_val, position_vector.y_val, position_vector.z_val]


def gps_position_to_list(gps_vector) -> list:
    return [gps_vector.latitude, gps_vector.longitude, gps_vector.altitude]


def gps_velocity_to_list(velocity_vector) -> list:
    return [velocity_vector.x_val, velocity_vector.y_val, velocity_vector.z_val]


"""
def propagate_coordinates(client, comm_matrix: np.array, drones: np.array):
    for i, drone_comm_params in enumerate(comm_matrix):
        for j, individual_param in enumerate(drone_comm_params):
            if comm_matrix[i, j] == True and len(drones[find_name(i)].swarm_positions) < len(drones):
                # packet build_comms_packet()
                # drone_i.communicate(drone_j, packet, fade=0.2, max_dist=dist, packet_drop="Gaussian")
                drones[find_name(i)].swarm_positions.append(drones[find_name(j)].pos_vec3)
"""

"""
[[[cog
import cog

with open(FILENAME, 'r') as f:
    for line in f:
        cog.out(line)
    cog.outl()
]]] """
def average_drone_positions(state_vector):
    positions = [state_vector[name] for name in list(state_vector)]
    addition = 1
    x = 0.0
    y = 0.0
    z = 0.0
    for i, drone_positions in enumerate(positions):
        x += drone_positions[0]
        y += drone_positions[1]
        z += drone_positions[2]
    for i, drone_name in enumerate(state_vector):
        if i % 2:
            addition = 0
        else:
            addition = -1
        state_vector[drone_name] = [(x/float(len(state_vector))) + addition, (y/float(len(state_vector))) + addition, -42]
#[[[end]]]


def fly_to_new_positions(client, drones: list) -> None:
    for i, drone_name in enumerate(list(drones)):
        z_coord = ensure_negative_z_coordinates(drones[drone_name][2])
        client.moveToPositionAsync(
            drones[drone_name][0], drones[drone_name][1], z_coord, 5, vehicle_name=drone_name)
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
    # the average position for each drone.
    pos = new_position
    new_position[0] += vehicle_offsets[0]
    new_position[1] += vehicle_offsets[1]
    new_position = pos
    # new_position[2] += vehicle_offsets[2]


def transform_to_relative_basis_coordinates(new_position: list, vehicle_offsets: list) -> list:
    # Transform the coordinates of each drone their independent representation frames.
    pos = new_position
    pos[0] -= vehicle_offsets[0]
    pos[1] -= vehicle_offsets[1]
    return pos
    # new_position[2] -= vehicle_offsets[2]


def build_vehicle_distance_matrix(drones: dict,
                                  distance_matrix) -> bool:
    for i, row in enumerate(distance_matrix):
        for j, column in enumerate(row):
            if i != j:
                first_drone = drones[find_name(i)].gps_pos_vec3
                second_drone = drones[find_name(j)].gps_pos_vec3
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


def send_finished_file_to_s3(minutes, seconds):
    s3 = boto3.client('s3', aws_access_key_id='AKIAZLJJC2FVAE5VNBR6', aws_secret_access_key='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')
    execution_file_name = 'finished.txt'
    with open(execution_file_name, 'w') as f:
        try:
            f.write('Completed on: {}\n'.format(datetime.today().strftime('%m/%d/%Y %I:%M%p')))
            f.write('Total time of execution: {} mins {} secs\n'.format(minutes, seconds))
            f.write('Finished algorithm Execution!')
        except Exception as error:
            print(error)

    with open(execution_file_name, 'rb') as f:
        try:
            response = s3.upload_fileobj(f,
                                         "swarmsimulations",
                                         'notifications/finished_{}.txt'.format(datetime.today().strftime('%m_%d_%Y')))
            print(response)
        except Exception as e:
            print(e)


def save_data_to_database(position_history, state_vectors, distances, commanded_position_history, database):
    print(len(state_vectors))
    print(len(distances))
    print(state_vectors)
    print(distances)
    pos_commit = database.commit_many_results(database.data_structs['positions'], position_history)
    state_vec_commit = database.commit_many_results(database.data_structs['state_vectors'], state_vectors)
    cmd_pos_commit = database.commit_many_results(database.data_structs['cmd_poss'], commanded_position_history)
    dist_commit = database.commit_many_results(database.data_structs['distance_matrices'], distances)
    if pos_commit:
        print("Positions were saved to the database!")
    if state_vec_commit:
        print("State vectors were saved to the database!")
    if cmd_pos_commit:
        print("Commanded position vectors were saved to the database!")
    if dist_commit:
        print("Distance matrices were saved to the database!")


def modify_data_object_for_data_transfer(data_tuple):
    data_tuple = list(data_tuple)
    data_tuple[-1] = convert_datetime_to_str(data_tuple[-1])
    return data_tuple

def generate_data_file_and_upload_to_s3(position_history, state_vectors, distances, commanded_position_history, drones, objective_met, minutes, seconds):
    # Read in the JSON file
    with open('data_structure.json', 'r') as f: 
        data_struct = json.load(f)

    # Update the JSON structure
    data_struct['SimulationID'] = sim_id
    data_struct['S3UUID'] = file_key
    data_struct['CompletionTimeSeconds'] = seconds
    data_struct['CompletionTimeMinutes'] = minutes
    if objective_met:
        data_struct['ObjectiveMet'] = 'True'
    else:
        data_struct['ObjectiveMet'] = 'False'
    for drone_name in list(drones):
        data_struct['Drones'].append(drones[drone_name].name)
    for state_vector in state_vectors:
        state_vector = modify_data_object_for_data_transfer(state_vector)
        data_struct['StateVectors'].append(state_vector)
    for position in position_history:
        position = modify_data_object_for_data_transfer(position)
        data_struct['Positions'].append(position)
    for distance in distances:
        distance = modify_data_object_for_data_transfer(distance)
        data_struct['Distances'].append(distance)
    for cmd_pos in commanded_position_history:
        cmd_pos = modify_data_object_for_data_transfer(cmd_pos)
        data_struct['CommandedPositions'].append(cmd_pos)

    # Save the JSON file to the container
    with open('data_structure.json', 'w') as f:
        json.dump(data_struct, f)

    # Upload the JSON file to S3
    s3_client = boto3.client('s3',
                             aws_access_key_id='AKIAZLJJC2FVAE5VNBR6',
                             aws_secret_access_key='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')
    with open('data_structure.json', 'rb') as f:
        try:
            response = s3_client.upload_fileobj(f,
                                         "swarmsimulations",
                                         'data_files/{}/data_structure.json'.format(file_key))
            print('Data file has been uploaded to S3!')
        except Exception as e:
            print(e)
            print('There was an error uploading the file to S3!')


# ====================================================================================================== #
# Start of main process
# ====================================================================================================== #

# Generate a set of drones based upon a given number input and number of swarms.
# Convention: Capital Letter = Drone Swarm Number = Number of drone in that swarm
# Ex: A1, A2, A3, etc.
# Load list of parameters into the system -> Some sort of class module to set all of these for me.


# Load vehicle names as a list for easy iteration.
# TO DO: This will be drawn from the parameters file loading (Rules sheet)
debug = False

file_key = sys.argv[1]
bucket_name = 'swarmsimulations'

print(file_key)
print("sim_settings/{}/user_input.json".format(file_key))

database = DB(user='tyler', password='LaX!!616678')

try:
    connected = database.make_connection()
    if connected:
        print("Connected to data acquisition system!")
    else:
        print("There was an issue connecting to the data acquisition.")
except Exception:
    traceback.print_exc()

s3 = boto3.resource('s3', aws_access_key_id='AKIAZLJJC2FVAE5VNBR6', aws_secret_access_key='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')
sim_settings = s3.Object(bucket_name, "sim_settings/{}/user_input.json".format(file_key))
sim_settings = sim_settings.get()['Body'].read().decode('utf-8')
sim_settings = json.loads(sim_settings)

sim_id = int(sim_settings["Algorithm"]["ID"])

scenario = Scenario(id=sim_settings["Algorithm"]["Scenario"]["scenario_id"],
                        weather=sim_settings["Algorithm"]["Scenario"]["weather_effects"])
swarm = Swarm()
swarm.add_n_drones(sim_settings["Algorithm"]["Swarm"]["number_of_drones"])

name = "A"
for i, drone_name in enumerate(swarm.drones):
    swarm.drones[drone_name].name = name
    j = ord(name[0])
    j += 1
    name = chr(j)

control = Control(swarm, scenario)
control.generate_state_vector()

for i, drone_name in enumerate(list(control.swarm.drones)):
    if connected:
        drone_data = (sim_id, drone_name, 2.0, "SimpleFlight")
        committed = database.commit_result(database.data_structs["drone"], drone_data)
        if committed:
            print("Drone {} was committed to the database!".format(drone_name))
        else:
            print("There was an error committing the drone to the database.")

with open('settings.json', 'r') as f:
    settings = json.load(f)

print(control.swarm.drones)
print(swarm.drones)

for drone_name in control.swarm.drones:
    drone = control.swarm.drones[drone_name]
    positions = settings["Vehicles"][drone_name]
    drone.starting_pos_vec3[0] = positions['X']
    drone.pos_vec3[0] = positions['X']
    drone.starting_pos_vec3[1] = positions['Y']
    drone.pos_vec3[1] = positions['Y']
    drone.starting_pos_vec3[2] = positions['Z']
    drone.pos_vec3[2] = positions['Z']
    positions['X'] = float(positions['X'])
    positions['Y'] = float(positions['Y'])
    positions['Z'] = float(positions['Z'])
    if connected:
        pos_data = (int(sim_id), drone_name, positions['X'], positions['Y'], abs(positions['Z']), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, datetime.utcnow())
        print(pos_data)
        committed = database.commit_result(database.data_structs['position'], pos_data)
        if committed:
            print("Initial drone position recorded for drone {}!".format(drone_name))
        else:
            print("There was an issue updating the position for drone {}".format(drone_name))

for drone_name in list(control.swarm.drones):
    print('\n', drone_name, control.swarm.drones[drone_name].pos_vec3)

vehicle_names = [ drone_name for drone_name in control.swarm.drones ]

# Each list here is meant to store the records for each set of data we
# are collecting. This process is much faster then trying to commit all
# of the data during actual algorithm execution.
position_history = []
commanded_position_history = []
distances = []
state_vectors = []

time_step = 3  # seconds
final_separation_distance = 10  # meters
minutes = 0
seconds = 0

# We want a matrix to track who can communicate with who!
# It should be a nxn matrix, with each drone tracking itself and the matrix looks like
#            drone_1 drone_2 ... drone n
# drone_1    true    false   ... true
# drone_2    false   true    ... true
# drone_n    false   false   ... true

distance_matrix = np.zeros((len(vehicle_names), len(vehicle_names)))

# We mimic the memory bank of a drone, tracking the relative positions.
# It should be a n-length vector, with each drone tracking itself and the matrix looks like

# drone_1 drone_2 ... drone n
# [x,y,z] [x,y,z] ... [x,y,z]

minutes = 0
seconds = 0

try:
    client = airsim.MultirotorClient()
    # client = None
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
        get_all_drone_positions(client, control.swarm.drones)

        print("="*100)
        for drone_name in control.swarm.drones:
            print('\n', drone_name, control.swarm.drones[drone_name].pos_vec3, control.swarm.drones[drone_name].gps_pos_vec3)

        # update_communication_matrix(client, communications_tracker, control.swarm.drones)

        for i, drone_name in enumerate(control.swarm.drones):
            transform_to_standard_basis_coordinates(control.swarm.drones[drone_name].pos_vec3,
                                                    control.swarm.drones[drone_name].starting_pos_vec3)

        # Propagate location to drones that can communicate
        # propagate_coordinates(client, communications_tracker, control.swarm.drones)

        for drone_name in control.swarm.drones:
            position_history.append((sim_id,
                                     drone_name,
                                     control.swarm.drones[drone_name].pos_vec3[0],
                                     control.swarm.drones[drone_name].pos_vec3[1],
                                     abs(control.swarm.drones[drone_name].pos_vec3[2]),
                                     control.swarm.drones[drone_name].gps_pos_vec3[0],
                                     control.swarm.drones[drone_name].gps_pos_vec3[1],
                                     control.swarm.drones[drone_name].gps_pos_vec3[2],
                                     control.swarm.drones[drone_name].current_velocity[0],
                                     control.swarm.drones[drone_name].current_velocity[1],
                                     control.swarm.drones[drone_name].current_velocity[2],
                                     datetime.utcnow()))
        
        # This state vector will be modified in the future to track something other then states!
        for i, drone_name in enumerate(control.state_vector):
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
        average_drone_positions(control.state_vector)
        #[[[end]]]

        print("="*50)
        for drone_name_4 in control.swarm.drones:
            print('\n', drone_name_4, control.swarm.drones[drone_name_4].pos_vec3, control.swarm.drones[drone_name_4].gps_pos_vec3)

        control.evaluate_state_vector()

        print(control.state_vector)
        
        state_vectors.append((sim_id, json.dumps(control.state_vector), datetime.utcnow()))
        
        build_vehicle_distance_matrix(control.swarm.drones, distance_matrix)

        distances.append((sim_id, json.dumps(np.ndarray.tolist(distance_matrix)), datetime.utcnow()))

        for key, value in control.state_vector.items():
            control.state_vector[key] = transform_to_relative_basis_coordinates(value,
                                                          control.swarm.drones[key].starting_pos_vec3)

        print("="*50)
        for drone_name_2 in control.swarm.drones:
            print('\n', drone_name_2, control.state_vector[drone_name_2], control.swarm.drones[drone_name_2].gps_pos_vec3)

        if not control.objective_met:
            fly_to_new_positions(client, control.state_vector)

        time.sleep(time_step)
        print("="*50)
        print('\n')

    end_time = time.time()
    total_time = end_time - start_time
    minutes = np.floor(total_time / 60)
    seconds = round(((total_time / 60) - minutes) * 60, 2)
    print("Total Time: {mins} mins {secs} secs".format(
        mins=minutes, secs=seconds))
    # airsim.wait_key('Press any key to reset to original state')
    client.reset()
except Exception:
    traceback.print_exc()
finally:
    if client:
        send_finished_file_to_s3(minutes, seconds)
        client.reset()
        disable_control(client, vehicle_names)
    save_data_to_database(position_history, state_vectors, distances, commanded_position_history, database)
    generate_data_file_and_upload_to_s3(position_history, state_vectors, distances, commanded_position_history, control.swarm.drones, control.objective_met, minutes, seconds)
    print("Finished!")