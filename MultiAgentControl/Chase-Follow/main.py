import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
from nav import Nav

veh_1_name = 'Lead'
veh_2_name = 'Follow'
waypoints_path_list = 'waypoints.json'

# Load route
route = Nav(waypoints_path_list)

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
print(veh_1_name, "\n")
print("State: %s" % pprint.pformat(state))

state = client.getMultirotorState(vehicle_name=veh_2_name)
print(veh_2_name, "\n")
print("State: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to begin flying the route:')

#Fly the waypoint route. Ensure that the second vehicle triggers the join()
# so as the halt the loop.
for waypoint in route.waypoints["points"]:
    f1 = client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_1_name)
    f2 = client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2], waypoint[3], vehicle_name=veh_2_name)
    f2.join()

client.hoverAsync().join()

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, veh_1_name)
client.armDisarm(False, veh_2_name)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, veh_1_name)
client.enableApiControl(False, veh_2_name)
