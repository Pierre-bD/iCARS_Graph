import time
import math
import numpy as np
from queue import Queue
from queue import Empty

import carla
import logging
import random
import cv2

 # Connect the client and set up bp library and spawn points\n",
client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points() 


 # Spawn ego vehicle
vehicle_bp = bp_lib.find('vehicle.audi.a2')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

 # Move spectator behind vehicle to view
spectator = world.get_spectator() 
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation)
spectator.set_transform(transform)

 # Iterate this cell to find desired camera location
camera_bp = bp_lib.find('sensor.camera.rgb') 
camera_init_trans = carla.Transform(carla.Location(z=2)) # Change this to move camera
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
time.sleep(0.2)
spectator.set_transform(camera.get_transform())
camera.destroy()

 # Spawn camera\
camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)

 # Callback stores sensor data in a dictionary for use outside callback                         \n",
def camera_callback(image, data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
 # Get gamera dimensions and initialise dictionary           
image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
camera_data = {'image': np.zeros((image_h, image_w, 4))}

 # Start camera recording
camera.listen(lambda image: camera_callback(image, camera_data))

# OpenCV named window for rendering
cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
cv2.imshow('RGB Camera', camera_data['image'])
cv2.waitKey(1)

    # Game loop
while True:
    
        # Imshow renders sensor data to display\n",
    cv2.imshow('RGB Camera', camera_data['image'])
        
        # Quit if user presses 'q'\n",
    if cv2.waitKey(1) == ord('q'):
        break
    
    # Close OpenCV window when finished\n",
cv2.destroyAllWindows()
camera.stop()
