from IPython.display import HTML, display
import ipywidgets as widgets

carla_status = widgets.Output()
carla_status.layout = widgets.Layout(border= '4px solid gray')
carla_status.clear_output(wait=True)
with carla_status:
    print("CARLA Server status: ?")

display(carla_status)

import os
import carla
import random
import time

# Create config
config = {
    "ros_bridge_sync_mode": True,      # True if using ros bridge in sync mode otherwise False, start ros with passive:=True when this is False
    "traffic_simulation": True        # True if you want simulate traffic(npcs)
}
# Connect client to CARLA server.
start = time.time()
while True:
    # Retry until the CARLA server is ready
    try:
        print("Waiting for CARLA")
        client = carla.Client(os.environ.get('CARLA_HOSTNAME', 'localhost'), 2000)
        client.set_timeout(1.0)
        world = client.get_world()
        client.set_timeout(10.0)
        break
    except:
        time.sleep(5)
print(f"Connection established in {time.time()-start} seconds.")

# Setting synchronous mode
synchronous_master = False
settings = world.get_settings()
if not settings.synchronous_mode:
    print("Applying synchronous mode")
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
     
ego_vehicle_role_name = [f"hero{i}" for i in range(20)]

# add default role name
ego_vehicle_role_name.append("ego_vehicle")

def validate_rolename(role_name):
    available_rolenames = ego_vehicle_role_name.copy()
    actors = world.get_actors().filter('vehicle.*')
    for actor in actors:
        if actor.attributes['role_name'] in available_rolenames:
            available_rolenames.remove(actor.attributes['role_name'])
    
    # all role names are taken
    if not len(available_rolenames):
        return None
    if role_name not in available_rolenames:
        role_name = random.choice(available_rolenames)
    return role_name


# Spawn an ego-vehicle randomly.
spawn_points = world.get_map().get_spawn_points()
blueprints_vehicle = world.get_blueprint_library().find("vehicle.tesla.model3")
ego_transform = spawn_points[random.randint(0, len(spawn_points) - 1)]

# Let ROS bridge know this vehicle is the ego vehicle.
# Vehicles controlled by the user are commonly differenciated
# in CARLA by setting the attribute role_name to ego.
role_name = 'ego_vehicle'
role_name = validate_rolename(role_name)
if role_name == None:
    raise Exception("All available role names already exist in the simulation.")
print(f"Validated role_name: {role_name}")
blueprints_vehicle.set_attribute('role_name', role_name)


# Spawn ego vehicle at a randomly selected spawn point and set
# autopilot to True. This will register the vehicle to the Traffic
# Manager. It will roam around the city endlessly.
batch = [carla.command.SpawnActor(
    blueprints_vehicle, ego_transform).then(
        carla.command.SetAutopilot(carla.command.FutureActor, True))]
results = None

try:
    # try excpet block for the below mentioned exception:
    # trying to create rpc server for traffic manager; but the 
    # system failed to create because of bind error.
    results = client.apply_batch_sync(batch, False)
except Exception as e:
    pass

if results is not None and not results[0].error:
    ego_vehicle = world.get_actor(results[0].actor_id)
else:
    # get the actor by role_name
    actors = world.get_actors().filter('vehicle.*')
    for actor in actors:
        if actor.attributes['role_name'] == role_name:
            ego_vehicle = actor
            break

# attach rgb camera to ego vehicle
blueprint_camera = world.get_blueprint_library().find('sensor.camera.rgb')
blueprint_camera.set_attribute('role_name', 'rgb_front')
blueprint_camera.set_attribute('image_size_x', '800')
blueprint_camera.set_attribute('image_size_y', '600')
blueprint_camera.set_attribute('fov', '90')
transform_camera_front = carla.Transform(carla.Location(x=0.0, y=+0, z=1.6), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
camera_front = world.spawn_actor(blueprint_camera, transform_camera_front, attach_to=ego_vehicle)


# attach rgb camera to ego vehicle
blueprint_camera = world.get_blueprint_library().find('sensor.camera.rgb')
blueprint_camera.set_attribute('role_name', 'rgb_view')
blueprint_camera.set_attribute('image_size_x', '800')
blueprint_camera.set_attribute('image_size_y', '600')
blueprint_camera.set_attribute('fov', '90')
transform_camera_third = carla.Transform(carla.Location(x=-10, y=+0, z=2.4), carla.Rotation(pitch=20.0, yaw=0.0, roll=0.0))
camera_third = world.spawn_actor(blueprint_camera, transform_camera_third, attach_to=ego_vehicle)

# gnss
blueprint_gnss = world.get_blueprint_library().find('sensor.other.gnss')
blueprint_gnss.set_attribute('role_name', 'gnss')
transform_gnss = carla.Transform(carla.Location(x=2.0, y=+0, z=2.0), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
gnss = world.spawn_actor(blueprint_gnss, transform_gnss, attach_to=ego_vehicle)

# imu
blueprint_imu = world.get_blueprint_library().find('sensor.other.imu')
blueprint_imu.set_attribute('role_name', 'imu')
transform_imu = carla.Transform(carla.Location(x=1.0, y=+0, z=2.0), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))
imu = world.spawn_actor(blueprint_imu, transform_imu, attach_to=ego_vehicle)

# Attach a lidar to the ego vehicle.
blueprint_lidar = world.get_blueprint_library().find('sensor.lidar.ray_cast')
blueprint_lidar.set_attribute('role_name', 'lidar')
blueprint_lidar.set_attribute('range', '70')
blueprint_lidar.set_attribute('rotation_frequency', '20')
blueprint_lidar.set_attribute('channels', '64')
blueprint_lidar.set_attribute('lower_fov', '-24.8')
blueprint_lidar.set_attribute('upper_fov', '7.0')
blueprint_lidar.set_attribute('points_per_second', '1280000')
blueprint_lidar.set_attribute('noise_stddev', '0.0')
transform_lidar = carla.Transform(carla.Location(x=0.0, y=0.0, z=1.6))
lidar = world.spawn_actor(blueprint_lidar, transform_lidar, attach_to=ego_vehicle)
     
# Attach Semantic LiDAR to the ego vehicle.
blueprint_semantic_lidar = world.get_blueprint_library().find('sensor.lidar.ray_cast_semantic')
blueprint_semantic_lidar.set_attribute('role_name', 'semantic_lidar')
blueprint_semantic_lidar.set_attribute('range', '50')
blueprint_semantic_lidar.set_attribute('rotation_frequency', '20')
blueprint_semantic_lidar.set_attribute('channels', '32')
blueprint_semantic_lidar.set_attribute('lower_fov', '-26.8')
blueprint_semantic_lidar.set_attribute('upper_fov', '2.0')
blueprint_semantic_lidar.set_attribute('points_per_second', '320000')
transform_semantic_lidar = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.4))
semantic_lidar = world.spawn_actor(blueprint_semantic_lidar, transform_semantic_lidar, attach_to=ego_vehicle)
     
# Attach radar sensor to the ego vehicle.
blueprint_radar = world.get_blueprint_library().find('sensor.other.radar')
blueprint_radar.set_attribute('role_name', 'radar_front')
blueprint_radar.set_attribute('horizontal_fov', '30.0')
blueprint_radar.set_attribute('vertical_fov', '10.0')
blueprint_radar.set_attribute('range', '100.0')
blueprint_radar.set_attribute('points_per_second', '1500')
transform_radar = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0))
radar = world.spawn_actor(blueprint_radar, transform_radar, attach_to=ego_vehicle)
     
# Activate semantic segmentation camera to ego vehicle
sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
sem_bp.set_attribute('role_name', 'semantic_segmentation_front')
sem_bp.set_attribute("image_size_x", '400')
sem_bp.set_attribute("image_size_y", '70')
sem_bp.set_attribute("sensor_tick", '0.1')
sem_bp.set_attribute("fov", '90')
sem_location = carla.Location(x=+2.0, y=0.0, z=2.0)
sem_transform = carla.Transform(sem_location)
sem_cam = world.spawn_actor(sem_bp, sem_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
depth_bp.set_attribute('role_name', 'depth_front')
depth_bp.set_attribute("image_size_x", '400')
depth_bp.set_attribute("image_size_y", '70')
depth_bp.set_attribute("sensor_tick", '0.1')
depth_bp.set_attribute("fov", '90')
depth_location = carla.Location(x=+2.0, y=0.0, z=2.0)
depth_transform = carla.Transform(depth_location)
depth_cam = world.spawn_actor(depth_bp, depth_transform, attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
     
# Use world.wait_for_tick() for sync mode with carla_ros_bridge
if config["ros_bridge_sync_mode"]:
    _ = world.wait_for_tick()
else:
    world.tick()
     
def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation.
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations.
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except Exception as e:
        print(e)
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
     
def spawn_npcs(args_number_of_vehicles = 50, args_number_of_walkers = 70, args_car_lights_on = False):
    global synchronous_master
    vehicles_list = []
    walkers_list = []
    all_id = []

    # world settings
    settings = world.get_settings()

    # setup traffic manager
    traffic_manager = client.get_trafficmanager(8001)
    traffic_manager.set_global_distance_to_leading_vehicle(2.5)
    traffic_manager.set_hybrid_physics_mode(True)
    traffic_manager.set_hybrid_physics_radius(70.0)
    traffic_manager.set_synchronous_mode(True)

    if not settings.synchronous_mode:
        synchronous_master = True
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05

    world.apply_settings(settings)

    blueprints = get_actor_blueprints(world, 'vehicle.*', 'All')
    blueprintsWalkers = get_actor_blueprints(world, 'walker.pedestrian.*', 'All')
    blueprints = sorted(blueprints, key=lambda bp: bp.id)

    # Fetch spawn points.
    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)

    if args_number_of_vehicles < number_of_spawn_points:
        random.shuffle(spawn_points)
    elif args_number_of_vehicles > number_of_spawn_points:
        msg = 'requested %d vehicles, but could only find %d spawn points'
        print(msg, args_number_of_vehicles, number_of_spawn_points)
        args_number_of_vehicles = number_of_spawn_points
    
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    # --------------
    # Spawn vehicles
    # --------------
    batch = []
    hero = False
    for n, transform in enumerate(spawn_points):
        if n >= args_number_of_vehicles:
            break
        blueprint = random.choice(blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if hero:
            blueprint.set_attribute('role_name', 'hero')
            hero = False
        else:
            blueprint.set_attribute('role_name', 'autopilot')

        # Spawn the cars and set their autopilot and light state all together.
        batch.append(SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            print(response.error)
        else:
            vehicles_list.append(response.actor_id)

    # Set automatic vehicle lights update if specified.
    if args_car_lights_on:
        all_vehicle_actors = world.get_actors(vehicles_list)
        for actor in all_vehicle_actors:
            traffic_manager.update_vehicle_lights(actor, True)
    
    # -------------
    # Spawn Walkers
    # -------------
    # some settings
    percentagePedestriansRunning = 0.0      # How many pedestrians will run.
    percentagePedestriansCrossing = 0.0     # How many pedestrians will walk through the road.
    
    random.seed(0)
    
    # 1. Take all the random locations to spawn.
    spawn_points = []
    for i in range(args_number_of_walkers):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    # 2. We spawn the walker object.
    batch = []
    walker_speed = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        # set as not invincible
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        # set the max speed
        if walker_bp.has_attribute('speed'):
            if (random.random() > percentagePedestriansRunning):
                # walking
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
            else:
                # running
                walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
        else:
            print("Walker has no speed")
            walker_speed.append(0.0)
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    walker_speed2 = []
    for i in range(len(results)):
        if results[i].error:
            print(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
            walker_speed2.append(walker_speed[i])
    walker_speed = walker_speed2
    
    # 3. We spawn the walker controller.
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            print(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    
    # 4. We put together the walkers and controllers id to get the objects from their id.
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    all_actors = world.get_actors(all_id)
    
    # Wait for a tick to ensure client receives the last transform of the walkers we have just created.
    # use world.wait_for_tick() when using with carla_ros_bridge sync mode
    if config["ros_bridge_sync_mode"]:
        _ = world.wait_for_tick()
    else:
        world.tick()
    
    # 5. Initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...]).
    # Set how many pedestrians can cross the road.
    world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # max speed
        all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))
    
    print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))
    
    # Example of how to use Traffic Manager parameters.
    traffic_manager.global_percentage_speed_difference(30.0)
     
# spawn npcs
if config["traffic_simulation"]:
    spawn_npcs()
     
print("Ego-vehicle created!")
     
import threading

def monitor_carla():
    while True:
        try:
            world.wait_for_tick()
            carla_status.layout = widgets.Layout(border= '4px solid green')
            carla_status.clear_output(wait=True)
            with carla_status:
                print("CARLA Server is running")
            time.sleep(5)
        except:
            carla_status.layout = widgets.Layout(border= '4px solid red')
            carla_status.clear_output(wait=True)
            with carla_status:
                print("CARLA server has died")
            break
thd = threading.Thread(target=monitor_carla, daemon=True).start()
     
