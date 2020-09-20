#!/usr/bin/env python3
'''
collision_testing.py : tests pickled network on ability to predict a collision

Copyright (C) 2017 Jack Baird, Alex Cantrell, Keith Denning, Rajwol Joshi, 
Simon D. Levy, Will McMurtry, Jacob Rosen

This file is part of AirSimTensorFlow

MIT License
'''

import airsim
import os
import time
import tensorflow as tf
import pickle
import sys
import pprint

from image_helper import loadgray, IMAGEDIR
from tf_softmax_layer import inference

TMPFILE = IMAGEDIR + '/active.png'
PARAMFILE = 'params.pkl'
IMGSIZE = 1032
INITIAL_THROTTLE= 0.65
BRAKING_DURATION = 15

try:
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

    # Load saved training params as ordinary NumPy
    W,b = pickle.load(open('params.pkl', 'rb'))

    with tf.Graph().as_default():

        # Placeholder for an image
        x = tf.placeholder('float', [None, IMGSIZE])

        # Our inference engine, intialized with weights we just loaded
        output = inference(x, IMGSIZE, 2, W, b)

        # TensorFlow initialization boilerplate
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        init_op = tf.global_variables_initializer()
        sess.run(init_op)

        # Once the brakes come on, we need to keep them on for a while before exiting; otherwise,
        # the vehicle will resume moving.
        brakingCount = 0

        # Loop until we detect a collision
        while True:

            # Get RGBA camera images from the car
            responses = client.simGetImages([airsim.ImageRequest(1, airsim.ImageType.Scene)])

            # Save it to a temporary file
            image = responses[0].image_data_uint8
            airsim.write_file(os.path.normpath(TMPFILE), image)

            # Read-load the image as a grayscale array
            image = loadgray(TMPFILE)

            # Run the image through our inference engine.
            # Engine returns a softmax output inside a list, so we grab the first
            # element of the list (the actual softmax vector), whose second element
            # is the absence of an obstacle.
            safety = sess.run(output, feed_dict={x:[image]})[0][1]
            print(safety)

            collision_info = client.simGetCollisionInfo()

            if collision_info.has_collided:
                print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
                    pprint.pformat(collision_info.position), 
                    pprint.pformat(collision_info.normal), 
                    pprint.pformat(collision_info.impact_point), 
                    collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
                break

            # Slam on the brakes if it ain't safe!
            if safety < 0.9:
                print('BRAKING TO AVOID COLLISSION')
                f1 = client.moveByVelocityAsync(0, 0.1, -2, 5)
                f1.join()
                sys.stdout.flush()
                break
           
            
            # Wait a bit on each iteration
            time.sleep(0.05)

    airsim.wait_key('Press any key to reset to original state')
except Exception as error:
    print(error)
finally:
    if client:
        client.armDisarm(False)
        client.reset()

        client.enableApiControl(False)
print('We are done!')
