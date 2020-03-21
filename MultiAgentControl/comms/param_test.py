import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time

veh_1_name = 'Lead'
veh_2_name = 'Follow'

# connect to the AirSim simulator
client = airsim.MultirotorClient()
print(client)
client.confirmConnection()
print('Connection Confirmed')
client.enableApiControl(True, veh_1_name)
client.enableApiControl(True, veh_2_name)
client.armDisarm(True, veh_1_name)
client.armDisarm(True, veh_2_name)

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name=veh_1_name)
f2 = client.takeoffAsync(vehicle_name=veh_2_name)
f1.join()
f2.join()

state = client.getMultirotorState(vehicle_name=veh_1_name)
state_2 = client.getMultirotorState(vehicle_name=veh_2_name)
drone_2_location = state_2.gps_location
comms_data = client.getCommunicationsData(drone_2_location.latitude, drone_2_location.longitude, drone_2_location.altitude, vehicle_name=veh_1_name)
print(veh_1_name, "\n")
print("State: %s" % pprint.pformat(state))
print("Comms Data: %s" % pprint.pformat(comms_data))

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, veh_1_name)
client.armDisarm(False, veh_2_name)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, veh_1_name)
client.enableApiControl(False, veh_2_name)
