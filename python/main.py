import time
import random
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np
import cmpe434_utils
import cmpe434_dungeon
import math
from visualizer import *
from controller_L2 import *
from controller_L1 import *



# Start: [ 20 28 ]
# Goal : [ 42 22 ]

# Pressing SPACE key toggles the paused state. 
# You can define other keys for other actions here.
def key_callback(keycode):
    if chr(keycode) == ' ':
        global paused
        paused = not paused

paused    = False # Global variable to control the pause state.
obstaclex = []
obstacley = []
sx        = 0
sy        = 0
gx        = 0
gy        = 0
syaw      = None

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
        increment_x = increment_x + 0.1
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
        increment_y = increment_y + 0.1
    obstaclex.extend(obstacle_x)
    obstacley.extend(obstacle_y)

		

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
    final_pos = random.choice([key for key in tiles.keys() if tiles[key] == "."])

    scene.worldbody.add('site', name='start', type='box', size=[0.5, 0.5, 0.01], rgba=[0, 0, 1, 1],  pos=[start_pos[0]*2, start_pos[1]*2, 0])
    scene.worldbody.add('site', name='finish', type='box', size=[0.5, 0.5, 0.01], rgba=[1, 0, 0, 1],  pos=[final_pos[0]*2, final_pos[1]*2, 0])

    start_yaw = random.randint(0, 359)
    robot.find("body", "buddy").set_attributes(pos=[start_pos[0]*2, start_pos[1]*2, 0.1], euler=[0, 0, start_yaw])
    sx   = start_pos[0]*2
    sy   = start_pos[1]*2
    gx   = final_pos[0]*2
    gy   = final_pos[1]*2
    syaw = start_yaw * math.pi/180
    scene.include_copy(robot)

    # Combine all assets into a single dictionary.
    all_assets = {**scene_assets, **robot_assets}

    return scene, all_assets

def execute_scenario(obstacles,scene, ASSETS=dict()):
    global sx,sy,gx,gy,syaw
    m              = mujoco.MjModel.from_xml_string(scene.to_xml_string(), assets=all_assets)
    d              = mujoco.MjData(m)
    max_v          = 5
    max_w          = 6
    min_v          = -1
    min_w          = -1*max_w
    gc             = 0.15
    vc             = 1
    oc             = 1
    ta             = 1
    aa             = 1
    time_window    = 2
    time_step      = 0.2
    rv             = 5
    rw             = 5
    #vehicle_width  = 0.25
    #vehicle_height = 0.2965
    vehicle_width = 0.3
    vehicle_height= 0.3
    rx,ry          = AStar(sx,sy,gx,gy,0.15,obstacles[:,0],obstacles[:,1])
    rx.reverse()
    ry.reverse()
    r_coordinates  = np.hstack((np.array(rx).reshape(-1, 1), np.array(ry).reshape(-1, 1)))
    """
    print(r_coordinates)
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

        # velocity     = m.actuator("throttle_velocity")
        # steering     = m.actuator("steering")
        #prev_point    = np.array([d.xpos[3][0:2],d.xpos[4][0:2],d.xpos[5][0:2],d.xpos[6][0:2]])
        #prev_point    = np.mean(prev_point, axis=0)
        velocity       = d.actuator("throttle_velocity")
        steering       = d.actuator("steering")
        point          = np.array(d.body("buddy").xpos[:2])
        #prev_point     = np.array(d.body("buddy").xpos[:2])
        visualize(r_coordinates,viewer)
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 10000:
            step_start = time.time()

            if not paused:
                goal          = []
                point         = np.array(d.body("buddy").xpos[:2])
                distances     = np.linalg.norm(r_coordinates - point, axis=1)
                #  Find the index of the node with the minimum distance
                closest_index = np.argmin(distances)
                target_index  = 0
                if closest_index + 5 > len(r_coordinates)-1:
                    target_index = len(r_coordinates)-1
                else:
                    target_index = closest_index + 5
                goal          = np.array(r_coordinates[target_index])
                squared_diff = (goal - np.array([gx,gy])) ** 2
                # Sum the squared differences along the axis of the features (axis=1 for 2D)
                sum_squared_diff = np.sum(squared_diff)
                # Take the square root to get the Euclidean distance
                euclidean_distance = np.sqrt(sum_squared_diff)
                if euclidean_distance < 0.1:
                    print("Goal Reached")
                    return
                #if calculate_distance(goal,point) < 0.2:
                #    print(calculate_distance(goal,point))
                #    print("Goal Reached")
                #    return
                velocity_val  = np.mean(np.array(d.sensordata[0:3]))
                angular_val   = np.mean(np.array(d.sensordata[3:6]))
                #print("Translational velocity:",velocity_val)
                #print("Angular velocity:",angular_val)
                yaw           =  0 
                if syaw != None:
                    yaw  = syaw
                    syaw = None
                else:
                    yaw  = calculate_yaw(prev_point,point)
                _,s,v    = pick_trajectory(point,goal,obstacles,velocity_val,angular_val,max_v,max_w,min_v,min_w,gc,vc,oc,ta,aa,time_window,time_step,rv,rw,vehicle_width,vehicle_height,yaw)
                print("Final Target:",gx,gy)
                print("Goal:",goal)
                print("Point:",point)
                print("Yaw:",yaw)
                print("Distance to Goal:",euclidean_distance)
                print("Steering:",s)
                print("Velocity:",v)
                print("***************")
                velocity.ctrl = v # update velocity control value
                steering.ctrl = s # update steering control value
                prev_point    = point.copy()
                # mj_step can be replaced with code that also evaluates
                # a policy and applies a control signal before stepping the physics.
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
    #plt.scatter(filtered_arr[:,0],filtered_arr[:,1])
    #plt.show()
    execute_scenario(filtered_arr, scene, all_assets)