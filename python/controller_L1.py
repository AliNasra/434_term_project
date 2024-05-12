import math
import matplotlib.pyplot as plt
import numpy as np


# Predict the trajectory over the next time window
def calculate_circular_trajectory(pose,v,w,time_step,time_window):
	trajectory = np.array([[pose[0],pose[1],pose[2],v,w]])
	x          = pose[0]
	y          = pose[1]
	theta      = pose[2]
	time_count = 0
	while time_count < time_window:
		theta = theta + (w * time_step)                                # update theta according to the kinematics equations
		x = x + v * math.cos(theta) * time_step                        # Calculate the new x position
		y = y + v * math.sin(theta) * time_step                        # Calculate the new y position
		trajectory = np.vstack((trajectory, [x,y,theta,v,w])) 		   # Append pose info into the trajectory list	
		time_count = time_count + time_step                            # Increment the time counter until it reaches the time window limit
	return trajectory[1:]

# To conserve computational resources, only consider obstacles within radius of 3
def filter_obstacles(pose,obstacles):
	radius        = 3
	distances = np.linalg.norm(obstacles - np.array([pose[0], pose[1]]), axis=1)
	# Find the indices of points where the distance is less than 4
	indices = np.where(distances < radius)
	# Extract the points that meet the condition
	filtered_list = obstacles[indices]
	return filtered_list


def calculate_velocity_range(max_v,min_v,max_w,min_w,v,w,aa,ta,time_window):
	range_vel = []
	v_upper   = v + ta * time_window
	v_lower   = v - ta * time_window
	w_upper   = w + aa * time_window
	w_lower   = w - aa * time_window
	range_vel.append([max(v_lower,min_v),min(v_upper,max_v)])
	range_vel.append([max(w_lower,min_w),min(w_upper,max_w)])
	return range_vel

# Inf if the vehicle runs into an obstacle
def calculate_obstacle_cost(coefficient,trajectory,obstacles,vehicle_width,vehicle_height):
	ox = obstacles[:, 0]
	oy = obstacles[:, 1]
	dx = trajectory[:, 0] - ox[:, None]
	dy = trajectory[:, 1] - oy[:, None]
	r = np.hypot(dx, dy)
	yaw = trajectory[:, 2]
	rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
	rot = np.transpose(rot, [2, 0, 1])
	local_ob = obstacles[:, None] - trajectory[:, 0:2]
	local_ob = local_ob.reshape(-1, local_ob.shape[-1])
	local_ob = np.array([local_ob @ x for x in rot])
	local_ob = local_ob.reshape(-1, local_ob.shape[-1])
	upper_check = local_ob[:, 0] <= vehicle_height / 2
	right_check = local_ob[:, 1] <= vehicle_width / 2
	bottom_check = local_ob[:, 0] >= -vehicle_height / 2
	left_check = local_ob[:, 1] >= -vehicle_width /2
	if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
		return float("Inf")
	min_r = np.min(r)
	return coefficient / min_r  # OK

# Prioritize trajectories with yaw close to angle between the robot and the goal
def calculate_goal_cost(goal,point,yaw):
	dx            = goal[0] - point[0]
	dy            = goal[1] - point[1]
	error_angle   = (math.atan2(dy, dx) + 2*math.pi)%(2*math.pi)
	cost          = abs(yaw-error_angle)
	if cost > math.pi:
		cost = 2*math.pi - cost
	return cost

# Give precedence to routes with faster velocities 
def calculate_velocity_cost(coefficient, trajectory,max_v):
	cost = coefficient * (max_v - trajectory[0,3])
	return cost
# Filter routes based on 1- Obstacles 2- Velocity 3- Approximity to the Goal
def filter_routes(costs,routes,obstacles):
	ideal_controls   = np.array(costs)
	sorted_indices_0 = np.argsort(ideal_controls[:, 2])
	ideal_controls   = ideal_controls[sorted_indices_0]
	value_to_remove  = float('inf')
	mask             = ideal_controls[:, 2] != value_to_remove
	count_true       = np.sum(mask)
	print("Valid Routes Count:",count_true)
	if count_true > 0:
		ideal_controls   = ideal_controls[mask]
	#else:
	#	for i in range(len(routes)):
	#		plt.plot(routes[i][:,0],routes[i][:,1])
	#	plt.scatter(obstacles[:,0],obstacles[:,1])
	#	plt.show()
	limit            = int(math.ceil(len(ideal_controls)*0.3))
	sorted_indices   = np.argsort(ideal_controls[:, 0])
	# Sort the array using the sorted indices
	ideal_controls   = ideal_controls[sorted_indices]
	ideal_controls   = ideal_controls[:limit]
	sorted_indices_2 = np.argsort(ideal_controls[:, 1])
	# Sort the array using the sorted indices
	ideal_controls  = ideal_controls[sorted_indices_2]
	return ideal_controls[0,3:]


# Generate Trajectories and choose the most convenient
def pick_trajectory(point,goal,obstacles,v,w,max_v,max_w,min_v,min_w,gc,vc,oc,ta,aa,time_window,time_step,rv,rw,vehicle_width,vehicle_height,yaw):
	
	vel_range      = calculate_velocity_range(max_v,min_v,max_w,min_w,v,w,aa,ta,time_window)
	pose           = np.array([point[0],point[1],yaw])
	vel_counter    = vel_range[0][0]
	rot_counter    = vel_range[1][0]
	resolution_v   = (vel_range[0][1]-vel_range[0][0])/rv
	resolution_w   = (vel_range[1][1]-vel_range[1][0])/rw
	obstacles_list = filter_obstacles(point,obstacles)
	counter        = 0
	routes         = []
	costs          = []
	while vel_counter<=vel_range[0][1]:
		while rot_counter<=vel_range[1][1]:
			trajectory = calculate_circular_trajectory(pose,vel_counter,rot_counter,time_step,time_window)
			routes.append(trajectory)
			if len(trajectory) == 0:
				rot_counter = rot_counter + rw
				continue
			cost_1     = calculate_goal_cost(goal,point,trajectory[0,2])  
			cost_2	   = calculate_velocity_cost(vc,trajectory,max_v)
			cost_3     = 0
			if len(obstacles_list)>0:
				cost_3     = calculate_obstacle_cost(oc,trajectory,obstacles_list,vehicle_width,vehicle_height)
			costs.append([cost_1,cost_2,cost_3,vel_counter,rot_counter,counter])
			rot_counter = rot_counter + resolution_w
			counter = counter + 1
		rot_counter = vel_range[1][0]
		vel_counter = vel_counter + resolution_v
	ideal_controls = filter_routes(costs,routes,obstacles_list)

	return ideal_controls[1],ideal_controls[0],routes[int(ideal_controls[2])]

# Convert rotation matrix of a robot to euler-format orientation -> Necessary for calculating the Yaw
def mat2euler(mat):
	mat = np.asarray(mat, dtype=np.float64)
	_FLOAT_EPS = np.finfo(np.float64).eps
	_EPS4 = _FLOAT_EPS * 4.0
	assert mat.shape[-2:] == (3, 3), "Invalid shape matrix {}".format(mat)

	cy = np.sqrt(mat[..., 2, 2] * mat[..., 2, 2] + mat[..., 1, 2] * mat[..., 1, 2])
	condition = cy > _EPS4
	euler = np.empty(mat.shape[:-1], dtype=np.float64)
	euler[..., 2] = np.where(condition,
                             -np.arctan2(mat[..., 0, 1], mat[..., 0, 0]),
                             -np.arctan2(-mat[..., 1, 0], mat[..., 1, 1]))
	euler[..., 1] = np.where(condition,
                             -np.arctan2(-mat[..., 0, 2], cy),
                             -np.arctan2(-mat[..., 0, 2], cy))
	euler[..., 0] = np.where(condition,
                             -np.arctan2(mat[..., 1, 2], mat[..., 2, 2]),
                             0.0)
	return euler