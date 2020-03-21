#!/usr/bin/env python3
'''
image_collection.py : uses AirSim to collect vehicle first-person-view images

Copyright (C) 2017 Jack Baird, Alex Cantrell, Keith Denning, Rajwol Joshi, 
Simon D. Levy, Will McMurtry, Jacob Rosen

This file is part of AirSimTensorFlow

MIT License
'''

import airsim
import setup_path 
from image_helper import IMAGEDIR
import pprint
import os
import time

# We maintain a queue of images of this size
QUEUESIZE = 10

# Create image directory if it doesn't already exist
try:
    os.stat(IMAGEDIR)
except:
    os.mkdir(IMAGEDIR)
    
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
client.moveByVelocityAsync(0, -2, 0, 2000)

imagequeue = []

while True:

    # get RGBA camera images from the car
    responses = client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.Scene)])  

    # add image to queue        
    imagequeue.append(responses[0].image_data_uint8)

    # dump queue when it gets full
    if len(imagequeue) == QUEUESIZE:
        for i in range(QUEUESIZE):
            airsim.write_file(os.path.normpath(IMAGEDIR + '/image%03d.png'  % i ), imagequeue[i])
        imagequeue.pop(0)    

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

client.enableApiControl(False)
