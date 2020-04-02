import setup_path 
import airsim

import numpy as np
import pprint
import time
import traceback
from scipy import floor, ceil
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
	"""
	   Make all vehicles takeoff, one at a time and return the
	   pointer for the last vehicle takeoff to ensure we wait for
	   all drones  
	"""
	vehicle_pointers = []
	for vehicle in vehicle_names:
		vehicle_pointers.append(client.takeoffAsync(vehicle_name=vehicle))
	# All of this happens asynchronously. Hold the program until the last vehicle
	# finishes taking off.
	return vehicle_pointers[-1]


def get_all_drone_positions(client, vehicle_names: list, position_tracker: np.array) -> np.array:
	for i, vehicle in enumerate(vehicle_names):
		state_data = client.getMultirotorState(vehicle_name=vehicle)
		position_tracker[i] = [state_data.gps_location, state_data.kinematics_estimated.position] 
	return position_tracker


def update_communication_matrix(client, comm_matrix: np.array, positions: np.array, vehicle_names: list) -> bool:
	for i, parameter_list in enumerate(comm_matrix):
		for j, individual_list in enumerate(parameter_list):
			if i != j:
				# i will give you the current drone, e.g. "A"
				# j will give you the comparison drone, e.g. "B, C, ..."
				comparison_drone = positions[j][0]
				# Get comms data for the drone we want "A" in relation to the comparison drone "B, C, ..."
				comm_matrix[i,j] = client.getCommunicationsData(comparison_drone.latitude, comparison_drone.longitude, comparison_drone.altitude, vehicle_name=vehicle_names[i]).can_communicate
			else:
				comm_matrix[i,j] = True


def propagate_coordinates(client, comm_matrix: np.array, positions: np.array, vehicle_names: list):
	new_positions = np.zeros((len(vehicle_names)), dtype=list)
	for i, position in enumerate(new_positions):
		new_positions[i] = []
	# print(new_positions)
	for i, drone_comm_params in enumerate(comm_matrix):
		for j, individual_param in enumerate(drone_comm_params):
			if comm_matrix[i,j] == True and len(new_positions[i]) < len(vehicle_names):
				new_positions[i].append(positions[j][1])
				# print(new_positions)
	#print(new_positions)
	for i, drone_positions in enumerate(new_positions):
		# print('Before the numbers')
		x = 0.0
		y = 0.0
		z = 0.0
		# print('Right before we go to add them up')
		for position in drone_positions:
			# print(position)
			x += position.x_val
			y += position.y_val
			z += position.z_val
		# print(x, y, z)
		new_positions[i] = [x/len(drone_positions), y/len(drone_positions), z/len(drone_positions)]
		# print(drone)
	return new_positions


def fly_to_new_positions(client, vehicle_names: list, new_positions: list, vehicle_offsets: dict, together_tracker: list) -> None:
	for i, drone in enumerate(vehicle_names):
		new_position = new_positions[i]
		# print(new_position)
		# You have to compensate for each drone's initial starting position, as each command
		# will be relative to where the drone starts.
		if drone == "B":
			new_position[0] += vehicle_offsets["B"][0]
			new_position[1] += vehicle_offsets["B"][1]
			new_position[2] += vehicle_offsets["B"][2]
			velocity = 5
		elif drone == "C":
			new_position[0] += vehicle_offsets["C"][0]
			new_position[1] += vehicle_offsets["C"][1]
			new_position[2] += vehicle_offsets["C"][2] + 20
			velocity = 5
		elif drone == "A":
			new_position[0] = new_position[0] * -1
			new_position[1] = new_position[1] * -1
			velocity = 3
		print("\n")
		print("{drone} -> {position}\n".format(drone=drone, position=new_position))
		if together_tracker[i] == True:
			client.moveByVelocityAsync(0, 0, 0, 0, vehicle_name=drone)
		else:
			client.moveToPositionAsync(new_position[0], new_position[1], -abs(new_position[2]), velocity, vehicle_name=drone)
		time.sleep(0.1)


def determine_distance_between(vehicle_names: list, position_tracker: list) -> bool:
	distances = np.zeros((len(vehicle_names)), dtype=float)
	for i, position in enumerate(position_tracker):
		try:
			first_drone = position_tracker[i][0]
			second_drone = position_tracker[i + 1][0]
			print(first_drone, second_drone)
			distances[i] = round(haversine(first_drone.latitude, first_drone.longitude, second_drone.latitude, second_drone.longitude)*1000, 3)
		except IndexError:
			first_drone = position_tracker[i][0]
			second_drone = position_tracker[i - 1][0]
			print(first_drone, second_drone)
			distances[i] = round(haversine(first_drone.latitude, first_drone.longitude, second_drone.latitude, second_drone.longitude)*1000, 3)
	print("\n", distances, "\n")
	together = distances < 5
	print("\n", together, "\n")
	return together


# Generate a set of drones based upon a given number input and number of swarms.
# Convention: Capital Letter = Drone Swarm Number = Number of drone in that swarm
# Ex: A1, A2, A3, etc.
# Load list of parameters into the system -> Some sort of class module to set all of these for me.

# Load vehicle names as a list for easy iteration.
# TO DO: This will be drawn from the parameters file loading (Rules sheet)
vehicle_names = ["A", "B", "C"]
vehicle_offsets = {"B": [-10, 95, -5], "C": [-20, 190, -25]}
time_step = 3 # seconds
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
	start_time = time.time()
	not_together = True
	while not_together:
		# Get initial locations
		position_tracker = get_all_drone_positions(client, vehicle_names, position_tracker)
		print("\n")
		print(position_tracker)
		print("\n")
		# Update Communications parameters
		update_communication_matrix(client, communications_tracker, position_tracker, vehicle_names)
		print("\n")
		print(communications_tracker)
		print("\n")
		# Propagate location to drones that can communicate
		new_positions = propagate_coordinates(client, communications_tracker, position_tracker, vehicle_names)
		print("\n")
		print(new_positions)
		print("\n")
		# enable_control(client, vehicle_names)
		together_tracker = determine_distance_between(vehicle_names, position_tracker)
		fly_to_new_positions(client, vehicle_names, new_positions, vehicle_offsets, together_tracker)
		# Returns a boolean array to track who is together
		time.sleep(time_step)

	end_time = time.time()
	total_time = end_time - start_time
	minutes = round(total_time / 60, 1)
	seconds = ceil((total_time / 60) - minutes)
	print("Total Time: {mins} mins {secs} secs".format(mins=minutes, secs=seconds))
	airsim.wait_key('Press any key to reset to original state')
	client.reset()
except Exception:
	traceback.print_exc()
finally:
	if client:
		client.reset()
		disable_control(client, vehicle_names)
	print("Finished!")