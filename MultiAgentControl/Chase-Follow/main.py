import setup_path 
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2

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

state = client.getMultirotorState(vehicle_name=veh_2_name)
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData(vehicle_name=veh_2_name)
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)

barometer_data = client.getBarometerData(vehicle_name=veh_2_name)
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData(vehicle_name=veh_2_name)
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData(vehicle_name=veh_2_name)
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

airsim.wait_key('Press any key to takeoff')
f1 = client.takeoffAsync(vehicle_name='Drone1')
f2 = client.takeoffAsync(vehicle_name='Drone2')
f1.join()
f2.join()

state = client.getMultirotorState(vehicle_name=veh_1_name)
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move vehicles to (-10, 10, -10) at 5 m/s')
f1 = client.moveToPositionAsync(-10, 10, -10, 5, vehicle_name=veh_1_name)
f2 = client.moveToPositionAsync(-10, 10, -11, 5, vehicle_name=veh_2_name)
f1.join()
f2.join()

client.hoverAsync().join()
"""
state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move vehicle to (15, 5, -100) at 4 m/s')
client.moveToPositionAsync(15, 5, -100, 4).join()

airsim.wait_key('Press any key to take images')
# get camera images from the car
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
    airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite('C:\\Users\\tyler\\programming\\research\\' + filename + '.png', img_rgb) # write to png

"""

airsim.wait_key('Press any key to reset to original state')

client.armDisarm(False, veh_1_name)
client.armDisarm(False, veh_2_name)
client.reset()

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False, veh_1_name)
client.enableApiControl(False, veh_2_name)
