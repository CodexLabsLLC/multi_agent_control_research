import setup_path 
import airsim

import numpy as np
import pprint
import time

from utils.distance_utils import haversine

def enable_control(client, vehicle_names: list) -> None:
	for vehicle in vehicle_names:	
		client.enableApiControl(True, vehicle)
		client.armDisarm(True, vehicle)

def disable_control(client, vehicle_names: list) -> None:
	for vehicle in vehicle_names:
		client.armDisarm(False, vehicle)
		client.enableApiControl(False, vehicle)


def takeoff(client, vehicle_names: list) -> None:
	vehicle_pointers = []
	for vehicle in vehicle_names:
		vehicle_pointers.append(client.takeoffAsync(vehicle_name=vehicle))
	# All of this happens asynchronously. Hold the program until the last vehicle
	# finishes taking off.
	return vehicle_pointers[-1]


def get_all_drone_positions(client, vehicle_names: list, position_tracker: np.array) -> np.array:
	for i, vehicle in enumerate(vehicle_names):
		position_tracker[i] = client.getMultirotorState(vehicle_name=vehicle).gps_location
	return position_tracker


def update_communication_matrix(client, comm_matrix: np.array, positions: np.array, vehicle_names: list) -> bool:
	for i, parameter_list in enumerate(comm_matrix):
		for j, individual_list in enumerate(parameter_list):
			if i != j:
				# i will give you the current drone, e.g. "A"
				# j will give you the comparison drone, e.g. "B, C, ..."
				comparison_drone = positions[j]
				# Get comms data for the drone we want "A" in relation to the comparison drone "B, C, ..."
				comm_matrix[i,j] = client.getCommunicationsData(comparison_drone.latitude, comparison_drone.longitude, comparison_drone.altitude, vehicle_name=vehicle_names[i]).can_communicate
			else:
				comm_matrix[i,j] = True


def propagate_coordinates(client, comm_matrix: np.array, positions: np.array, vehicle_names: list):
	pass

# Generate a set of drones based upon a given number input and number of swarms.
# Convention: Capital Letter = Drone Swarm Number = Number of drone in that swarm
# Ex: A1, A2, A3, etc.
# Load list of parameters into the system -> Some sort of class module to set all of these for me.

# Load vehicle names as a list for easy iteration.
# TO DO: This will be drawn from the parameters file loading (Rules sheet)
vehicle_names = ["A", "B", "C"]
time_step = 1 # seconds
final_separation_distance = 3 # meters

# We want a matrix to track who can communicate with who!
# It should be a nxn matrix, with each drone tracking itself and the matrix looks like
#            drone_1 drone_2 ... drone n
# drone_1    true    false   ... true
# drone_2    false   true    ... true
# drone_n    false   false   ... true  
communications_tracker = np.zeros((len(vehicle_names),len(vehicle_names)), dtype=bool)

# We mimic the memory bank of a drone, tracking the relative positions.
# It should be a n-length vector, with each drone tracking itself and the matrix looks like

# drone_1 drone_2 ... drone n
# [x,y,z] [x,y,z] ... [x,y,z]

position_tracker = np.zeros((len(vehicle_names)), dtype=list)

try:
	client = airsim.MultirotorClient()
	print(client)
	client.confirmConnection()
	print('Connection Confirmed')

	enable_control(client, vehicle_names)

	airsim.wait_key('Press any key to takeoff')
	last_vehicle_pointer = takeoff(client, vehicle_names)
	# We wait until the last drone is off the ground
	last_vehicle_pointer.join()

	airsim.wait_key('Press any key to rendevous the drones!')

	not_together = True
	while not_together:
		# Get initial locations
		position_tracker = get_all_drone_positions(client, vehicle_names, position_tracker)
		# Update Communications parameters
		update_communication_matrix(client, communications_tracker, position_tracker, vehicle_names)
		print(communications_tracker)
		not_together = False
		# Propagate location to drones that can communicate
		#propagate_coordinates(client, communications_tracker, position_tracker, vehicle_names)

	airsim.wait_key('Press any key to reset to original state')
	client.reset()
except Exception as error:
	print(error)
finally:
	if client:
		client.reset()
		disable_control(client, vehicle_names)
	print("Finished!")