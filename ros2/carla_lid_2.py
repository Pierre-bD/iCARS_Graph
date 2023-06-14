import time
import sys
import os
import glob
from queue import Queue
from queue import Empty
import numpy as np

import carla
import random
import cv2
import open3d as o3d
from matplotlib import cm

def main():
    fps_sim = 100.0
    nb_frame = 5000
    nb_walkers = 10
    nb_vehicles = 10
    init_settings = None

    try : 
        client = carla.Client('localhost', 2000)
        init_settings = carla.WorldSettings()
        client.set_timeout(100.0)
        world = client.get_world()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 1.0/fps_sim
        settings.no_rendering_mode = True
        world.apply_settings(settings)

        ### Get the blueprint library
        bp_lib = world.get_blueprint_library() 
        spawn_points = world.get_map().get_spawn_points() #get spawn points

######
# Add vehicle
######
        vehicle_bp = bp_lib.find('vehicle.tesla.model3')
        vehicle_bp.set_attribute('role_name','ego_vehicle')
        vehicle = world.spawn_actor(vehicle_bp, spawn_points[79])

        # Move spectator to view ego vehicle
        spectator = world.get_spectator() 
        transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=-4,z=2.5)),vehicle.get_transform().rotation) 
        spectator.set_transform(transform)

        # Add traffic and set in motion with Traffic Manager
        #for i in range(2): 
        #    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
        #    vehicle_bp.set_attribute('role_name',f"vehicle{i}")
        #    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
        #for v in world.get_actors().filter('*vehicle*'): 
        #    v.set_autopilot(True) 


        ######
        # Set up Imu sensor
        ######
        imu_bp = bp_lib.find('sensor.other.imu')
        imu_bp.set_attribute('role_name','imu')
        #imu_bp.set_attribute("sensor_tick", '0.0')
        imu_init_trans = carla.Transform(carla.Location(x=1.0, y=+0, z=1.6), carla.Rotation(pitch=0.0,yaw=0.0, roll=0.0))
        imu_clb = world.spawn_actor(imu_bp, imu_init_trans, attach_to=vehicle)


        ######
        # Set up Gnss sensor
        ######
        gnss_bp = bp_lib.find('sensor.other.gnss')
        gnss_bp.set_attribute('role_name','gnss')
        gnss_bp.set_attribute('sensor_tick', '0.5')
        gnss_init_trans = carla.Transform(carla.Location(x=1.2,y=+0, z=1.6), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
        gnss_clb = world.spawn_actor(gnss_bp, gnss_init_trans, attach_to=vehicle)


        ######
        # Set up LIDAR, parameters are to assisst visualisation
        ######
        lidar_bp = bp_lib.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('role_name','lidar')
        lidar_bp.set_attribute('range', '50.0')
        lidar_bp.set_attribute('noise_stddev', '0.0')
        lidar_bp.set_attribute('upper_fov', '5.5')
        lidar_bp.set_attribute('lower_fov', '-20.5')
        lidar_bp.set_attribute('channels', '64.0')
        lidar_bp.set_attribute('rotation_frequency', '10.0')
        lidar_bp.set_attribute('points_per_second', '1200000')
        #lidar_bp.set_attribute("sensor_tick", '0.1')
        lidar_init_trans = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.6))
        lidar = world.spawn_actor(lidar_bp, lidar_init_trans, attach_to=vehicle)
        lidar.listen(lambda point_cloud: point_cloud.save_to_disk('/home/pierro/Programmation/tutorial/%.6d.ply' % point_cloud.frame))
        vehicle.set_autopilot(True)

        world.tick()

        print ("Starting simulation")
        time.sleep(2)

        frame = 0
        
        while (frame < nb_frame):

            lx,ly,lz = vehicle.get_location().x, vehicle.get_location().y ,vehicle.get_location().z
            location = [lx, ly, lz]
            #lid_frame = lidar.t
            print("[+] Car Location: (x y z)=(", location, ")")
                
            frame += 1
            world.tick()

    
        for actor in world.get_actors().filter('*vehicle*'):
            actor.destroy()
        for actor in world.get_actors().filter('*sensor*'):
            actor.destroy()


    finally:
        print("End of simulation")
        world.apply_settings(init_settings)
    
    time.sleep(2.0)

if __name__ == '__main__':
    main()