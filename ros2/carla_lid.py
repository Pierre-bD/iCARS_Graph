import time
import sys
import os
from queue import Queue
from queue import Empty
import numpy as np

import carla
import random
import cv2
import open3d as o3d
from matplotlib import cm

######
# Connect the client and set up bp library and spawn points\n",
######
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
settings = world.get_settings()
weather = carla.WeatherParameters(cloudiness=50.0, precipitation=20.0)
world.set_weather(weather)
settings.no_rendering_mode = True
#settings.substepping =True
#settings.max_substep_delta_time = 0.01
#settings.max_substeps = 10
#settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.1
world.apply_settings(settings)
bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points() #get spawn points

######
# Add vehicle
######
vehicle_bp = bp_lib.find('vehicle.tesla.model3')
vehicle_bp.set_attribute('role_name','ego_vehicle')
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points[79])

# Move spectator to view ego vehicle
spectator = world.get_spectator() 
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
spectator.set_transform(transform)

# Add traffic and set in motion with Traffic Manager
for i in range(2): 
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    vehicle_bp.set_attribute('role_name',f"vehicle{i}")
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
for v in world.get_actors().filter('*vehicle*'): 
    v.set_autopilot(True) 

######
# Set NPC
######

#walker_bp = world.get_blueprint_library().find('controller.ai.walker')
#world.SpawnActor(walker_bp, carla.Transform(), parent_walker)

# Auxilliary code for colormaps and axes
VIRIDIS = np.array(cm.get_cmap('plasma').colors)
VID_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
    
COOL_RANGE = np.linspace(0.0, 1.0, VIRIDIS.shape[0])
COOL = np.array(cm.get_cmap('winter')(COOL_RANGE))
COOL = COOL[:,:3]

def add_open3d_axis(vis):
    """Add a small 3D axis on Open3D Visualizer"""
    axis = o3d.geometry.LineSet()
    axis.points = o3d.utility.Vector3dVector(np.array([
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    axis.lines = o3d.utility.Vector2iVector(np.array([
        [0, 1],
        [0, 2],
        [0, 3]]))
    axis.colors = o3d.utility.Vector3dVector(np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0]]))
    vis.add_geometry(axis)

######
# LIDAR callbacks
######
def lidar_callback(point_cloud, point_list):
#"""Prepares a point cloud with intensitycolors ready to be consumed by Open3D"""
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype('f4')))
    data = np.reshape(data, (int(data.shape[0] / 4), 4))
    
        # Isolate the intensity and compute a color for it\n",
    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIRIDIS[:, 2])]
    
    points = data[:, :-1]
    
    points[:, :1] = -points[:, :1]
    
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    
######
# Camera callback
######
#def camera_callback(image, data_dict):
#    data_dict['image'] = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

######
# Gnss & Imu callback
######
def gnss_callback(gnss):
    print("GNSS measure:\n"+str(gnss)+'\n')

def imu_callback(imu):
    print("IMU measure:\n"+str(imu)+'\n')

######
# Set up Imu sensor
######
imu_bp = bp_lib.find('sensor.other.imu')
imu_bp.set_attribute('role_name','imu')
#imu_bp.set_attribute("sensor_tick", '0.02')
imu_init_trans = carla.Transform(carla.Location(x=1.0, y=+0, z=1.6), carla.Rotation(pitch=0.0,yaw=0.0, roll=0.0))
imu_clb = world.spawn_actor(imu_bp, imu_init_trans, attach_to=vehicle)


######
# Set up Gnss sensor
######
gnss_bp = bp_lib.find('sensor.other.gnss')
gnss_bp.set_attribute('role_name','gnss')
#gnss_bp.set_attribute('sensor_tick', '0.01')
gnss_init_trans = carla.Transform(carla.Location(x=1.2,y=+0, z=1.6), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
gnss_clb = world.spawn_actor(gnss_bp, gnss_init_trans, attach_to=vehicle)



######
# Set up LIDAR, parameters are to assisst visualisation
######
lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('role_name','lidar')
lidar_bp.set_attribute('range', '50.0')
lidar_bp.set_attribute('noise_stddev', '0.0')
lidar_bp.set_attribute('upper_fov', '11.5')
lidar_bp.set_attribute('lower_fov', '-11.5')
lidar_bp.set_attribute('channels', '64.0')
lidar_bp.set_attribute('rotation_frequency', '10.0')
lidar_bp.set_attribute('points_per_second', '1200000')
#lidar_bp.set_attribute("sensor_tick", '0.1')
lidar_init_trans = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.6))
lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)

# Spawn camera
camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('role_name','camera')
camera_init_trans = carla.Transform(carla.Location(x=-3, y=0, z=2.5), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    
# Add auxilliary data structures
point_list = o3d.geometry.PointCloud()
   
# Set up dictionary for camera data
#image_w = camera_bp.get_attribute("image_size_x").as_int()
#image_h = camera_bp.get_attribute("image_size_y").as_int()
#camera_data = {'image': np.zeros((image_h, image_w, 4))} 
# Start sensors
lidar.listen(lambda point_cloud: point_cloud.save_to_disk('/home/pierro/Programmation/tutorial/%.6d.ply' % point_cloud.frame))
#lidar.listen(lambda data: lidar_callback(data, point_list))
#camera.listen(lambda image: camera_callback(image, camera_data))
imu_clb.listen(lambda imu: imu_callback(imu))
#gnss_clb.listen(lambda gnss: gnss_callback(gnss))

# OpenCV window for camera\n",
#cv2.namedWindow('RGB Camera', cv2.WINDOW_AUTOSIZE)
#cv2.imshow('RGB Camera', camera_data['image'])
#cv2.waitKey(1)


# Open3D visualiser for LIDAR and RADAR\n",
#vis = o3d.visualization.Visualizer()
#vis.create_window(
#    window_name='Carla Lidar',
#    width=960,
#    height=540,
#    left=480,
#    top=270)
#vis.get_render_option().background_color = [0.05, 0.05, 0.05]
#vis.get_render_option().point_size = 1
#vis.get_render_option().show_coordinate_frame = True
#add_open3d_axis(vis)

# Update geometry and camera in game loop\n",

##try something

frame = 0
try:
    while True:

    #if frame == 2:
    #    vis.add_geometry(point_list)
        
    #vis.update_geometry(point_list)
    
        
    #vis.poll_events()
    #vis.update_renderer()
    # # This can fix Open3D jittering issues:\n",
    #time.sleep(0.005)
    #frame += 1
        lx,ly,lz = vehicle.get_location().x, vehicle.get_location().y ,vehicle.get_location().z
        location = [lx, ly, lz]
        #lid_frame = lidar.t
        print("[+] Car Location: (x y z)=(", location, ")")
        frame += 1

    #cv2.imshow('RGB Camera', camera_data['image'])
    #print("\n")
    # Break if user presses 'q'\n",
    
        #if cv2.waitKey(1) == ord('q'):
        #    break
    # Close displayws and stop sensors\n",
except KeyboardInterrupt:
        print("[~] Stopped simulation")
#cv2.destroyAllWindows()
#lidar.stop()
#lidar.destroy()
#camera.stop()
#camera.destroy()
#vis.destroy_window()
    
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for actor in world.get_actors().filter('*sensor*'):
    actor.destroy()