import time
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import cmpe434_utils
import numpy as np
from controller_L1 import *
import random
import math
"""
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "-3 6 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "4 0 0.05" density = "5000"/>
    <geom type="box" size="0.1 2.6 0.1" rgba="0.01 .3 1 1" pos= "4  2 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "0.8 2 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "2 2 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "0 3 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "3 1 0.05" density = "5000"/>
    <geom type="box" size="0.2 0.1 0.1" rgba="0.01 .3 1 1" pos= "2 0 0.05" density = "5000"/>
"""
scene, scene_assets = cmpe434_utils.get_model('scenes/racing_circuit.xml')
robot, robot_assets = cmpe434_utils.get_model('models/mushr_car/model.xml')
# robot, robot_assets = cmpe434_utils.get_model('models/mujoco_car/model.xml')
# robot, robot_assets = cmpe434_utils.get_model('models/skydio_x2/x2.xml')

# Add the robot to the scene.
scene.include_copy(robot)

# Combine all assets into a single dictionary.
all_assets = {**scene_assets, **robot_assets}

m = mujoco.MjModel.from_xml_string(scene.to_xml_string(), assets=all_assets)
d = mujoco.MjData(m) 



paused = False # Global variable to control the pause state.

obstaclesx,obstaclesy = get_obstacle_points(m)
obstacles = np.column_stack([obstaclesx, obstaclesy])
# Pressing SPACE key toggles the paused state. 
# You can define other keys for other actions here.
def key_callback(keycode):
  if chr(keycode) == ' ':
    global paused
    paused = not paused



with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:

  # velocity = m.actuator("throttle_velocity")
  # steering = m.actuator("steering")

  velocity       = d.actuator("throttle_velocity")
  steering       = d.actuator("steering")
  start          = time.time()
  prev_point     = np.array([d.xpos[3][0:2],d.xpos[4][0:2],d.xpos[5][0:2],d.xpos[6][0:2]])
  prev_point     = np.mean(prev_point, axis=0)
  max_v          = 5
  max_w          = 50.0 * math.pi / 180.0
  min_v          = -2.5
  min_w          = -1*max_w
  gc             = 0.15
  vc             = 1
  oc             = 1
  ta             = 2
  aa             = 50.0 * math.pi / 180.0
  time_window    = 2
  time_step      = 1
  rv             = 5
  rw             = 5
  vehicle_width  = 0.25
  vehicle_height = 0.2965
  #tile_count     = 0
  while viewer.is_running() and time.time() - start < 10000:
    step_start = time.time()

    if not paused:
        #list(viewer.user_scn.geoms).clear()
        #viewer.user_scn.ngeom   = 0
        #tile_count              = 0
        #print(d.xpos)
        point        = np.array([d.xpos[3][0:2],d.xpos[4][0:2],d.xpos[5][0:2],d.xpos[6][0:2]])
        point        = np.mean(point, axis=0)
        #print(point)
        velocity_val = np.mean(np.array(d.sensordata[0:3]))
        angular_val  = np.mean(np.array(d.sensordata[3:6]))
        goal_x = random.randint(-5, 6)
        goal_y = random.randint(-1, 8)
        #velocity_val = math.sqrt((v_measure[0]**2)+(v_measure[1]**2))
        #angular_val  = math.sqrt((w_measure[0]**2)+(w_measure[1]**2))
        goal         = np.array([goal_x,goal_y])
        #goal         = [random.uniform(-6,4),random.uniform(-2,8)]
        traj,s,v     = pick_trajectory(point,prev_point,goal,obstacles,velocity_val,angular_val,max_v,max_w,min_v,min_w,gc,vc,oc,ta,aa,time_window,time_step,rv,rw,vehicle_width,vehicle_height)
        #print("Goal:",goal)
        #print("Steering:",s," - ","Velocity:",v)
        """
        for point in traj:
            mujoco.mjv_initGeom(
                      viewer.user_scn.geoms[tile_count],
                      type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                      size = [0.05, 0.,0.],
                      pos =np.array([point[0], point[1], 0.05]),
                      mat =np.eye(3).flatten(),
                      rgba=np.array([0.89, 0.051, 0.051,0.4])
              )            
            tile_count += 1
        viewer.user_scn.ngeom = tile_count
        """
        #control_vals  = DWA(12,12,obstacles)
        #steering.ctrl = control_vals[0]
        #velocity.ctrl = control_vals[1]
        #print(s,v)
        steering.ctrl  =  s
        velocity.ctrl  =  v
        prev_point     = point.copy()
        mujoco.mj_step(m, d)
        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()
    else:
        break
    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = m.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)
