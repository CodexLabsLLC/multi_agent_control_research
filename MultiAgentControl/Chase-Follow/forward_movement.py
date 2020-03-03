import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time
from nav import Nav

veh_1_name = 'Lead'
veh_2_name = 'Follow'
waypoints_path_list = 'waypoints.json'
CONFIRMATION_DISTANCE = 1.5

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
print('Connection Confirmed')
client.enableApiControl(True)
client.armDisarm(True)

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync()
f1.join()

airsim.wait_key('Begin movement towards obstacle')
client.moveByVelocityAsync(-1, 0, 0, 200)

while True:
    collision_info = client.simGetCollisionInfo()

    if collision_info.has_collided:
        print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
            pprint.pformat(collision_info.position), 
            pprint.pformat(collision_info.normal), 
            pprint.pformat(collision_info.impact_point), 
            collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
        break

    time.sleep(0.1)

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)