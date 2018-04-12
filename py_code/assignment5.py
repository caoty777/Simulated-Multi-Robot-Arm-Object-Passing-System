import vrep
import time
import random
import numpy as np
from scipy.linalg import expm, logm
from pprint import pprint
from helper_functions import skew_3, skew_6, skew2twist, adjoint, revolute_S, euler2mat, deg2rad
from helper_functions import GetJointsHandle, rad2deg, euler_to_screw_axis, base_to_world_frame_a_q
from helper_functions import T_from_R_p, rot2euler, get_S, forward_kinematics, isMtxSame
from helper_functions import SetJointAngles, inverse_kinematics, get_S, OptimizeJointAngles
from helper_functions import StraightLineCollisionCheck, CollisionCheck, UpdateSphereP
from helper_functions import PathPlanner, PathSmoothing

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Get the handles for all six joints
joint_handles = GetJointsHandle(clientID)

# obtain the initial zero position angles for the six joints
joints_zero_pos = np.zeros(6)
for i in range(6):
	# Get the current value of the current joint
	result, theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint variable #{}'.format(i+1))
	joints_zero_pos[i] = theta


# Obtain the initial pose of the end effector (dummy7 frame)
# Get the handle of dummy7 object
result, dummy_handle = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy7 handle')

# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy7 orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
M = T_from_R_p(R_end,p_end)

# Get the R and p for the base frame (Jaco) w.r.t the world frame
# Get the handle of Jaco object
result, jaco_handle = vrep.simxGetObjectHandle(clientID, 'Jaco', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get Jaco handle')

# Get the position of base frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get Jaco position')

# Get the orientation of base w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jaco_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get Jaco orientation')

R_base_in_world = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
p_base_in_world = np.reshape(p,(3,1))

# get S - screw axes
S1,S2,S3,S4,S5,S6 = get_S(R_base_in_world,p_base_in_world)


################################ Path Planning Part #########################################

# get the handles of the spheres attached to the robot
robot_sphere_handles = []
result, sphere0_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 1 handle')
robot_sphere_handles.append(sphere0_handle)

result, sphere1_handle = vrep.simxGetObjectHandle(clientID, 'Dummy0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 2 handle')
robot_sphere_handles.append(sphere1_handle)

result, sphere2_handle = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 3 handle')
robot_sphere_handles.append(sphere2_handle)

result, sphere3_handle = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 4 handle')
robot_sphere_handles.append(sphere3_handle)

result, sphere4_handle = vrep.simxGetObjectHandle(clientID, 'Dummy3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 5 handle')
robot_sphere_handles.append(sphere4_handle)

result, sphere5_handle = vrep.simxGetObjectHandle(clientID, 'Dummy4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 6 handle')
robot_sphere_handles.append(sphere5_handle)

result, sphere6_handle = vrep.simxGetObjectHandle(clientID, 'Dummy5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 7 handle')
robot_sphere_handles.append(sphere6_handle)

result, sphere7_handle = vrep.simxGetObjectHandle(clientID, 'Dummy6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 8 handle')
robot_sphere_handles.append(sphere7_handle)


# get the handles of the spheres that represent obstacles in the environment
obstacle_sphere_handles = []

result, sphere_handle = vrep.simxGetObjectHandle(clientID, 'Dummy8', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere handle')
obstacle_sphere_handles.append(sphere_handle)

result, sphere_handle = vrep.simxGetObjectHandle(clientID, 'Dummy9', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere handle')
obstacle_sphere_handles.append(sphere_handle)

result, sphere_handle = vrep.simxGetObjectHandle(clientID, 'Dummy10', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere handle')
obstacle_sphere_handles.append(sphere_handle)

result, sphere_handle = vrep.simxGetObjectHandle(clientID, 'Dummy11', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere handle')
obstacle_sphere_handles.append(sphere_handle)


# get the positions of the robot spheres as well as obstacle spheres
p_robot = []
p_obstacle = []
for i in range(8):
	result, p = vrep.simxGetObjectPosition(clientID,robot_sphere_handles[i],-1,vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get sphere position')
	p_robot.append(p)

for i in range(len(obstacle_sphere_handles)):
	result, p = vrep.simxGetObjectPosition(clientID,obstacle_sphere_handles[i],-1,vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get sphere position')
	p_obstacle.append(p)

# get the radius of the robot spheres as well as obstacle spheres
r_robot = np.array([0.12,0.12,0.10,0.10,0.10,0.10,0.10,0.12])
r_robot = r_robot / 2
r_obstacle = np.array([0.15,0.15,0.15,0.15])
r_obstacle = r_obstacle / 2


# get the handles of the sphere that represent the goal pose of the tool frame
result, goal_sphere_handle = vrep.simxGetObjectHandle(clientID, 'Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere handle')


# Use IK to find a set of goal configuration, goal_angles

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,goal_sphere_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get goal position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, goal_sphere_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get goal orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
curr_goal_pose = T_from_R_p(R_end,p_end)


# obtain the current pose of the tool frame:
result, p = vrep.simxGetObjectPosition(clientID,dummy_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy7 position')

result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy7 orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
curr_pose = T_from_R_p(R_end,p_end)

error, goal_angles = inverse_kinematics(clientID,M,curr_goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
if error == 1:
	pprint('Cannot find an inverse kinematics solution within 100 iterations')
	pprint('Halting the program...')

else:
	goal_angles = OptimizeJointAngles(goal_angles)
	start_angles = np.zeros(6)

	# Path planner switch
	PP_on = 1

	# Path Smoothing switch
	path_smooth_on = 1

	if PP_on == 1:
		pprint('Path Planner is ON')
		pprint('--------------------------------------')
	########################## Implementation of Path Planning ##########################
		success, final_path = PathPlanner(start_angles,goal_angles,p_robot,p_obstacle,r_robot,r_obstacle,S1,S2,S3,S4,S5,S6)

		if success == 0:
			pprint('Path planner fails to find a collision-free path.')
			pprint('Halting the program...')
		else: 
			######################## Implementation of path smoothing #######################
			if path_smooth_on == 1:
				pprint('Path Smoothing is ON')
				pprint('The number of configurations in the unsmoothed path: ')
				pprint(len(final_path))

				final_path = PathSmoothing(final_path,p_robot,p_obstacle,r_robot,r_obstacle,S1,S2,S3,S4,S5,S6)
				pprint('The number of configurations in the smoothed path: ')
				pprint(len(final_path))

			# create a straight line path from start config to goal config, and check collision along the way
			s_value = np.linspace(0,1,21)

			for path_segment_idx in range(len(final_path) - 1):
				curr_start_angles = final_path[path_segment_idx]
				curr_goal_angles = final_path[path_segment_idx + 1]
				error = 0
				for i in range(1,21):
					curr_s = s_value[i]
					curr_theta = (1-curr_s)*curr_start_angles + curr_s*curr_goal_angles

					curr_theta = curr_theta + joints_zero_pos
					#curr_theta = curr_theta % (np.pi*2)
					
					if rad2deg(curr_theta[1]) < 47 or rad2deg(curr_theta[1]) > 313:
						error = 1
						pprint('Joint 2 out of valid range')
						pprint(curr_theta[1])
						pprint('Program halting...')
						break
					elif rad2deg(curr_theta[2]) < 19 or rad2deg(curr_theta[2]) > 341:
						error = 1
						pprint('Joint 3 out of valid range')
						pprint(curr_theta[2])
						pprint('Program halting...')
						break

					SetJointAngles(clientID,joint_handles,curr_theta)

					test_p_robot = []
					for i in range(8):
						result, p = vrep.simxGetObjectPosition(clientID,robot_sphere_handles[i],-1,vrep.simx_opmode_blocking)
						if result != vrep.simx_return_ok:
							raise Exception('could not get sphere position')
						test_p_robot.append(p)

					#pprint(curr_p_robot)
					#pprint(test_p_robot)

					if CollisionCheck(test_p_robot,p_obstacle,r_robot,r_obstacle) == 1:
						error = 1
						pprint('Collision Detected! Program halting...')
						break

				if error == 1:
					break
			if error == 0:
				pprint('Collision-free path completed! Program halting...')


	else:
	############################## Path Planning is turned off ################################
		pprint('Path Planner is OFF')
		pprint('--------------------------------------')
		s_value = np.linspace(0,1,21)
		error = 0
		for i in range(1,21):
			curr_s = s_value[i]
			curr_theta = (1-curr_s)*start_angles + curr_s*goal_angles

			curr_theta = curr_theta + joints_zero_pos
			#curr_theta = curr_theta % (np.pi*2)
			
			if rad2deg(curr_theta[1]) < 47 or rad2deg(curr_theta[1]) > 313:
				error = 1
				pprint('Joint 2 out of valid range')
				pprint(curr_theta[1])
				pprint('Program halting...')
				break
			elif rad2deg(curr_theta[2]) < 19 or rad2deg(curr_theta[2]) > 341:
				error = 1
				pprint('Joint 3 out of valid range')
				pprint(curr_theta[2])
				pprint('Program halting...')
				break

			SetJointAngles(clientID,joint_handles,curr_theta)

			curr_p_robot = []
			for i in range(8):
				result, p = vrep.simxGetObjectPosition(clientID,robot_sphere_handles[i],-1,vrep.simx_opmode_blocking)
				if result != vrep.simx_return_ok:
					raise Exception('could not get sphere position')
				curr_p_robot.append(p)

			if CollisionCheck(curr_p_robot,p_obstacle,r_robot,r_obstacle) == 1:
				error = 1
				pprint('Collision Detected! Program halting...')
				break

		if error == 0:
			pprint('Path Completed! Program halting...')



time.sleep(2)
# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)




