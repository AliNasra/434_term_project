import time
import random
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import scipy as sp
import cmpe434_utils
import cmpe434_dungeon
import math
from visualizer import *
from controller_L2 import *
from controller_L1 import *


# Pressing SPACE key toggles the paused state. 
# You can define other keys for other actions here.
def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

paused    = False # Global variable to control the pause state.
obstaclex = []   # The x coordinates of the static obstacles are stored here
obstacley = []   # The y coordinates of the static obstacles are stored here
sx        = 0    # Start X coordinate of the robot
sy        = 0    # Start Y coordinate of the robot
gx        = 0    # Goal X coordinate of the robot
gy        = 0    # Goal Y coordinate of the robot
syaw      = None # Start Yaw of the robot

# A function to sample points on the exterior of the static triangular obstacles

def add_obstacles(pos_x,pos_y,size_x,size_y):
    increment_x       = 0
    increment_y       = 0
    obstacle_x        = []
    obstacle_y        = []
    while increment_x <= size_x:
		#Add 2 points on the right
        obstacle_x.append(pos_x+increment_x)
        obstacle_x.append(pos_x+increment_x)
        obstacle_y.append(pos_y+size_y)
        obstacle_y.append(pos_y-size_y)
        #Add 2 points on the left
        obstacle_x.append(pos_x-increment_x)
        obstacle_x.append(pos_x-increment_x)
        obstacle_y.append(pos_y+size_y)
        obstacle_y.append(pos_y-size_y)
        increment_x = increment_x + 0.05
    while increment_y <= size_y:
		#Add 2 points on the left
        obstacle_x.append(pos_x+size_x)
        obstacle_x.append(pos_x+size_x)
        obstacle_y.append(pos_y+increment_y)
        obstacle_y.append(pos_y-increment_y)
        #Add 2 points on the right
        obstacle_x.append(pos_x-size_x)
        obstacle_x.append(pos_x-size_x)
        obstacle_y.append(pos_y+increment_y)
        obstacle_y.append(pos_y-increment_y)
        increment_y = increment_y + 0.05
    obstaclex.extend(obstacle_x)
    obstacley.extend(obstacle_y)

# A function to sample points on the exterior of the dynaqmic circular obstacles

def add_obstacles_dynamic(pos_x,pos_y,radius):
    list_x  = np.array([])
    list_y  = np.array([])
    counter = 0
    while counter < 2*math.pi:
        x       = pos_x + radius*math.cos(counter)
        y       = pos_y + radius*math.sin(counter)
        list_x  = np.append(list_x, x)
        list_y  = np.append(list_y, y)
        counter += 0.3
    return list_x,list_y

# Creates the surrounding Mujoco environment

def create_scenario():
    global sx,sy,gx,gy,syaw
    scene, scene_assets = cmpe434_utils.get_model('scenes/empty_floor.xml')
    # robot, robot_assets = cmpe434_utils.get_model('models/mujoco_car/model.xml')
    # robot, robot_assets = cmpe434_utils.get_model('models/skydio_x2/x2.xml')

    tiles, rooms, connections = cmpe434_dungeon.generate(3, 2, 8)

    for index, r in enumerate(rooms):
        (xmin, ymin, xmax, ymax) = cmpe434_dungeon.find_room_corners(r)
        scene.worldbody.add('geom', name='R{}'.format(index), type='plane', size=[(xmax-xmin)+1, (ymax-ymin)+1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax), (ymin+ymax), 0])
        #add_obstacles((xmin+xmax),(ymin+ymax),(xmax-xmin)+1,(ymax-ymin)+1)
    for pos, tile in tiles.items():
        if tile == "#":
            scene.worldbody.add('geom', type='box', size=[1, 1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[pos[0]*2, pos[1]*2, 0])
            add_obstacles(pos[0]*2,pos[1]*2,1,1)
    
    # scene.worldbody.add('geom', type='plane', size=[(xmax-xmin)/2+0.1, (ymax-ymin)/2+0.1, 0.01], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax)/2, (ymin+ymax)/2, 0])

    # scene.worldbody.add('geom', type='box', size=[0.1, (ymax-ymin)/2+0.1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[xmin, (ymin+ymax)/2, 0.1])
    # scene.worldbody.add('geom', type='box', size=[0.1, (ymax-ymin)/2+0.1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[xmax, (ymin+ymax)/2, 0.1])
    # scene.worldbody.add('geom', type='box', size=[(xmax-xmin)/2+0.1, 0.1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax)/2, ymin, 0.1])
    # scene.worldbody.add('geom', type='box', size=[(xmax-xmin)/2+0.1, 0.1, 0.1], rgba=[0.8, 0.6, 0.4, 1],  pos=[(xmin+xmax)/2, ymax, 0.1])

    # Add the robot to the scene.
    robot, robot_assets = cmpe434_utils.get_model('models/mushr_car/model.xml')
    start_pos = random.choice([key for key in tiles.keys() if tiles[key] == "."])
    final_pos = random.choice([key for key in tiles.keys() if tiles[key] == "." and key != start_pos])

    scene.worldbody.add('site', name='start', type='box', size=[0.5, 0.5, 0.01], rgba=[0, 0, 1, 1],  pos=[start_pos[0]*2, start_pos[1]*2, 0])
    scene.worldbody.add('site', name='finish', type='box', size=[0.5, 0.5, 0.01], rgba=[1, 0, 0, 1],  pos=[final_pos[0]*2, final_pos[1]*2, 0])

    for i, room in enumerate(rooms):
        obs_pos = random.choice([tile for tile in room if tile != start_pos and tile != final_pos])
        scene.worldbody.add('geom', name='Z{}'.format(i), type='cylinder', size=[0.2, 0.05], rgba=[0.8, 0.0, 0.1, 1],  pos=[obs_pos[0]*2, obs_pos[1]*2, 0.08])

    start_yaw = random.randint(0, 359)
    robot.find("body", "buddy").set_attributes(pos=[start_pos[0]*2, start_pos[1]*2, 0.1], euler=[0, 0, start_yaw])
    # assigning the global variables given the randomly generated input
    sx   = start_pos[0]*2
    sy   = start_pos[1]*2
    gx   = final_pos[0]*2
    gy   = final_pos[1]*2
    syaw = start_yaw * math.pi/180
    scene.include_copy(robot)

    # Combine all assets into a single dictionary.
    all_assets = {**scene_assets, **robot_assets}

    return scene, all_assets

# Main Driver Function

def execute_scenario(obstacles,scene, ASSETS=dict()):
    global sx,sy,gx,gy,syaw
    # The main parameters for the simulation
    m              = mujoco.MjModel.from_xml_string(scene.to_xml_string(), assets=all_assets)
    d              = mujoco.MjData(m)
    max_v          = 2.5                      # Maximum value the velocity control variable can take
    max_w          = 12                       # Maximum value the steering control variable can take
    min_v          = 0.2                      # Minimum value the velocity control variable can take
    min_w          = -1*max_w                 # Minimum value the steering control variable can take
    gc             = 1                        # A parameter for calibrating the goal cost, currently ineffectual
    vc             = 1                        # A parameter for calibrating the velocity cost, currently ineffectual
    oc             = 1                        # A parameter for calibrating the obstacle cost, currently ineffectual
    time_window    = 0.5                      # Time Frame over which the dynamic trajectories are predicted
    time_step      = time_window*0.1          # The time step. We sample 10 points per trajectory
    ta             = max_v/(1.2*time_window)  # Translational acceleration involved in calculating the velocity range
    aa             = max_w/(0.8*time_window)  # Rotational acceleration involved in calculating the steering range
    rv             = 8                        # Number of velocity samples
    rw             = 8                        # Number of steering samples per velocity value 
    look_ahead_in  = 3                        # Setting the goal 3 indices ahead of the closest index
    vehicle_width  = 0.5                      # Robot's dimensions
    vehicle_height = 0.5                      
    rx,ry          = AStar(sx,sy,gx,gy,0.15,obstacles[:,0],obstacles[:,1]) #Route created by level_2 controller
    rx.reverse()
    ry.reverse()
    r_coordinates  = np.hstack((np.array(rx).reshape(-1, 1), np.array(ry).reshape(-1, 1)))
    
    rooms = [m.geom(i).id for i in range(m.ngeom) if m.geom(i).name.startswith("R")]
    dynamic_obstacles = [m.geom(i).id for i in range(m.ngeom) if m.geom(i).name.startswith("Z")]

    uniform_direction_dist = sp.stats.uniform_direction(2)
    obstacle_direction = [[x, y, 0] for x,y in uniform_direction_dist.rvs(len(dynamic_obstacles))]

    unused = np.zeros(1, dtype=np.int32)

    #print(r_coordinates)
    """
    plt.scatter(obstacles[:,0],obstacles[:,1])
    plt.scatter(rx,ry)
    plt.scatter(sx,sy)
    plt.scatter(gx,gy)
    print("Start Coordinates:",sx,sy)
    print("Goal Coordinates:",gx,gy)
    print("Start Yaw:",syaw)
    plt.show()
    """
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

        velocity       = d.actuator("throttle_velocity")
        steering       = d.actuator("steering")
        point          = np.array([d.body("wheel_fl").xpos[:2],d.body("wheel_fr").xpos[:2],d.body("wheel_bl").xpos[:2],d.body("wheel_br").xpos[:2]])
        point          = np.mean(point, axis=0)
        yaw            = 0
        tile_count     = visualize(r_coordinates,viewer)
        start_count    = tile_count
        tile_count     = initalize_route(int(time_window//time_step),viewer,start_count)
        dynamic_coordinates_x = np.array([])
        dynamic_coordinates_y = np.array([])
        dynamic_coordinates   = np.array([])
        # Generate dynamic obstacles
        for _, x in enumerate(dynamic_obstacles):
            px = m.geom_pos[x][0]
            py = m.geom_pos[x][1]
            x_l, y_l               = add_obstacles_dynamic(px,py,0.2)
            dynamic_coordinates_x  = np.append(dynamic_coordinates_x, x_l)
            dynamic_coordinates_y  = np.append(dynamic_coordinates_y, y_l)
        dynamic_coordinates = np.column_stack((dynamic_coordinates_x, dynamic_coordinates_y))
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 10000:
            step_start = time.time()

            if not paused:
                #print("Vel Value:",velocity.ctrl)
                complete_obstacles = np.concatenate((obstacles, dynamic_coordinates)) # combine static and dynamic obstacles
                #plt.scatter(complete_obstacles[:,0],complete_obstacles[:,1])
                #plt.show()
                goal          = []   # Dynamic goal
                point         = np.array([d.body("wheel_fl").xpos[:2],d.body("wheel_fr").xpos[:2],d.body("wheel_bl").xpos[:2],d.body("wheel_br").xpos[:2]])
                point         = np.mean(point, axis=0)
                distances     = np.linalg.norm(r_coordinates - point, axis=1)
                #  Find the index of the node with the minimum distance
                closest_index = np.argmin(distances)
                target_index  = 0
                # Set the next goal given the closest index
                if closest_index + look_ahead_in > len(r_coordinates)-1:
                    target_index = len(r_coordinates)-1
                else:
                    target_index = closest_index + look_ahead_in
                goal          = np.array(r_coordinates[target_index])
                squared_diff = (point - np.array([gx,gy])) ** 2
                # Sum the squared differences along the axis of the features (axis=1 for 2D)
                sum_squared_diff = np.sum(squared_diff)
                # Take the square root to get the Euclidean distance
                euclidean_distance = np.sqrt(sum_squared_diff)
                # Stop the simulation if the distance between the robot and the final target is less than 0.5
                if euclidean_distance < 0.5:
                    print("Goal Reached")
                    return
                velocity_val   = velocity.ctrl[0]
                angular_val    = steering.ctrl[0]
                if syaw != None:
                    yaw  = syaw %(2*math.pi)
                    syaw = None
                else:
                    inrotmat = np.array(d.body("buddy").xmat).reshape(3, 3)
                    euler = mat2euler(inrotmat)
                    yaw   = (euler[2] + 2*math.pi)%(2*math.pi)
                # Select a trajectory
                s,v,traj    = pick_trajectory(point,goal,complete_obstacles,velocity_val,angular_val,max_v,max_w,min_v,min_w,gc,vc,oc,ta,aa,time_window,time_step,rv,rw,vehicle_width,vehicle_height,yaw)
                vizualize_route(traj,viewer,start_count)
                print("Distance to Goal:",euclidean_distance," with index",target_index,"/",(len(r_coordinates)-1))
                print("Steering:",s)
                print("Velocity:",v)
                print("Coordinates: (",point[0],",",point[1],")")
                print("***************")
                velocity.ctrl = v # update velocity control value
                steering.ctrl = s # update steering control value
                tile_count    = addpoint(point,viewer,tile_count) # Visualize the robot's route
                dynamic_coordinates_x = np.array([])
                dynamic_coordinates_y = np.array([])
                dynamic_coordinates   = np.array([])
                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
                for i, x in enumerate(dynamic_obstacles):
                    dx = obstacle_direction[i][0]
                    dy = obstacle_direction[i][1]

                    px = m.geom_pos[x][0]
                    py = m.geom_pos[x][1]
                    pz = 0.02

                    nearest_dist = mujoco.mj_ray(m, d, [px, py, pz], obstacle_direction[i], None, 1, -1, unused)

                    if nearest_dist >= 0 and nearest_dist < 0.4:
                        obstacle_direction[i][0] = -dy
                        obstacle_direction[i][1] = dx

                    new_x                  = m.geom_pos[x][0]+dx*0.001
                    new_y                  = m.geom_pos[x][1]+dy*0.001
                    m.geom_pos[x][0]       = new_x
                    m.geom_pos[x][1]       = new_y
                    x_l, y_l               = add_obstacles_dynamic(new_x,new_y,0.2) # Update dynamic obstacles
                    dynamic_coordinates_x  = np.append(dynamic_coordinates_x, x_l)
                    dynamic_coordinates_y  = np.append(dynamic_coordinates_y, y_l)
                dynamic_coordinates = np.column_stack((dynamic_coordinates_x, dynamic_coordinates_y))
                mujoco.mj_step(m, d)
                
                # Pick up changes to the physics state, apply perturbations, update options from GUI.
                viewer.sync()
            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)

            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    return m, d

if __name__ == '__main__':
    scene, all_assets    = create_scenario()
    coordinates          = np.hstack((np.array(obstaclex).reshape(-1, 1), np.array(obstacley).reshape(-1, 1)))
    unique_points,counts = np.unique(coordinates,axis=0,return_counts=True)
    filtered_arr         = unique_points[counts == 1]
    execute_scenario(filtered_arr, scene, all_assets)