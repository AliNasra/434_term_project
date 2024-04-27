import math
import matplotlib.pyplot as plt
import numpy as np


def calculate_distance(first_point,second_point):
	dist_sqr = ((first_point[0]-second_point[0])**2)+((first_point[1]-second_point[1])**2)
	return math.sqrt(dist_sqr)

def calculate_yaw(first_point,second_point):
	angle = math.atan2(second_point[1]-first_point[1],second_point[0]-first_point[0])
	return angle

def calculate_circular_trajectory(pose,v,w,aa,ta,time_step,time_window):
	trajectory = np.array([[0,0,0,0,0]])
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


def filter_obstacles(pose,obstacles):
	filtered_list = []
	radius        = 2
	for obstacle in obstacles:
		if calculate_distance(pose,obstacle) < radius:
			filtered_list.append(obstacle)
	return np.array(filtered_list)


def calculate_velocity_range(max_v,min_v,max_w,min_w,v,w,aa,ta,time_window):
	range_vel = []
	v_upper   = v + ta * time_window
	v_lower   = v - ta * time_window
	w_upper   = w + aa * time_window
	w_lower   = w - aa * time_window
	range_vel.append([max(v_lower,min_v),min(v_upper,max_v)])
	range_vel.append([max(w_lower,min_w),min(w_upper,max_w)])
	return range_vel


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
	left_check = local_ob[:, 1] >= -vehicle_width / 2
	if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
		return float("Inf")
	min_r = np.min(r)
	return coefficient / min_r  # OK


def calculate_goal_cost(coefficient,trajectory,goal):
	dx = goal[0] - trajectory[-1, 0]
	dy = goal[1] - trajectory[-1, 1]
	error_angle = math.atan2(dy, dx)
	cost_angle = error_angle - trajectory[-1, 2]
	cost = coefficient * abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
	return cost


def calculate_velocity_cost(coefficient, trajectory,max_v):
	cost = coefficient * (max_v - trajectory[0,3])
	return cost

def filter_routes(costs):
	ideal_controls  = np.array(costs)
	value_to_remove = float("Inf")
	mask = ideal_controls[:, 2] != value_to_remove
	filtered = ideal_controls[mask]
	sorted_indices = np.argsort(filtered[:, 0])
	# Sort the array using the sorted indices
	sorted_array = filtered[sorted_indices]
	sorted_indices_2 = np.argsort(sorted_array[:10, 1])
	# Sort the array using the sorted indices
	sorted_array_2 = sorted_array[sorted_indices_2]
	#print("Control Vals:",sorted_array_2[0,3:])
	return sorted_array_2[0,3:]



def pick_trajectory(point,goal,obstacles,v,w,max_v,max_w,min_v,min_w,gc,vc,oc,ta,aa,time_window,time_step,rv,rw,vehicle_width,vehicle_height,yaw):
	
	vel_range      = calculate_velocity_range(max_v,min_v,max_w,min_w,v,w,aa,ta,time_window)
	#print("vel_range:",vel_range)
	min_cost       = float("Inf")
	pose           = np.array([point[0],point[1],yaw])
	vel_counter    = vel_range[0][0]
	rot_counter    = vel_range[1][0]
	resolution_v   = (vel_range[0][1]-vel_range[0][0])/rv
	resolution_w   = (vel_range[1][1]-vel_range[1][0])/rw
	#ideal_traj     = []
	obstacles_list = filter_obstacles(point,obstacles)
	#counter        = 0
	#chosen_counter = 0
	#traj_x         = []
	#traj_y         = []
	costs          = []
	while vel_counter<=vel_range[0][1]:
		while rot_counter<=vel_range[1][1]:
			#counter = counter + 1
			trajectory = calculate_circular_trajectory(pose,vel_counter,rot_counter,aa,ta,time_step,time_window)
			#traj_x.extend(list(trajectory[:,0]))
			#traj_y.extend(list(trajectory[:,1]))
			if len(trajectory) == 0:
				rot_counter = rot_counter + rw
				continue
			cost_1     = calculate_goal_cost(gc,trajectory,goal)  
			cost_2	   = calculate_velocity_cost(vc,trajectory,max_v)
			cost_3     = 0
			if len(obstacles_list)>0:
				cost_3     = calculate_obstacle_cost(oc,trajectory,obstacles_list,vehicle_width,vehicle_height)
			costs.append([cost_1,cost_2,cost_3,vel_counter,rot_counter])
			#cost_total = cost_1 + cost_2 + cost_3
			#print("Angle:",trajectory[0,2],"Cost:",cost_total,"Coordinates: (",point[0],",",point[1],")")
			#if (cost_total<min_cost and cost_3 != float("Inf")):
				#print(cost_total,"vs", min_cost)
				#chosen_counter = counter
				#print("Better route with angle =",trajectory[0,2])
			#	ideal_traj = np.array(trajectory.copy())
			#	min_cost   = cost_total			
			rot_counter = rot_counter + resolution_w
		rot_counter = vel_range[1][0]
		vel_counter = vel_counter + resolution_v
	#print("Chosen Counter:",chosen_counter)
	#plt.scatter(traj_x,traj_y)
	#plt.show()
	#print("Chosen Trajectory:",ideal_traj[0,2])
	#print("*****************************")
	#plt.scatter(ideal_traj[:,0],ideal_traj[:,1])
	#plt.show()
	#return ideal_traj,(ideal_traj[0,4]),(ideal_traj[0,3])
	ideal_controls = filter_routes(costs)

	return ideal_controls,ideal_controls[1],ideal_controls[0]



def get_obstacle_points(m):
	obstacle_x  = []
	obstacle_y  = []
	obstacles_no = len(m.geom_pos)-12
	for i in range(1,obstacles_no,1):		
		pos_x             = m.geom_pos[i][0]
		pos_y             = m.geom_pos[i][1]
		size_x            = m.geom_size[i][0]
		size_y            = m.geom_size[i][1]
		increment_x       = 0
		increment_y       = 0
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
	#plt.scatter(obstacle_x,obstacle_y)
	#plt.show()
	return obstacle_x.copy(),obstacle_y.copy()
	

