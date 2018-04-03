import vrep
import time
import random
import numpy as np
from scipy.linalg import expm, logm
from pprint import pprint
from helper_functions import skew_3, skew_6, skew2twist, adjoint, revolute_S, euler2mat, deg2rad
from helper_functions import GetJointsHandle, rad2deg, euler_to_screw_axis, base_to_world_frame_a_q
from helper_functions import T_from_R_p, rot2euler, get_S, forward_kinematics, isMtxSame
from helper_functions import inverse_kinematics, get_S, OptimizeJointAngles
from helper_functions import IsTwoBallCollision, CollisionCheck


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


# obtain the initial zero position angles for the six joints
joints_zero_pos = np.zeros(6)
for i in range(6):
	# Get the current value of the current joint
	result, theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint variable #{}'.format(i+1))
	joints_zero_pos[i] = theta


# get the handle of dummy spheres
sphere_handles = []
result, sphere0_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 1 handle')
sphere_handles.append(sphere0_handle)

result, sphere1_handle = vrep.simxGetObjectHandle(clientID, 'Dummy0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 2 handle')
sphere_handles.append(sphere1_handle)

result, sphere2_handle = vrep.simxGetObjectHandle(clientID, 'Dummy1', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 3 handle')
sphere_handles.append(sphere2_handle)

result, sphere3_handle = vrep.simxGetObjectHandle(clientID, 'Dummy2', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 4 handle')
sphere_handles.append(sphere3_handle)

result, sphere4_handle = vrep.simxGetObjectHandle(clientID, 'Dummy3', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 5 handle')
sphere_handles.append(sphere4_handle)

result, sphere5_handle = vrep.simxGetObjectHandle(clientID, 'Dummy4', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 6 handle')
sphere_handles.append(sphere5_handle)

result, sphere6_handle = vrep.simxGetObjectHandle(clientID, 'Dummy5', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 7 handle')
sphere_handles.append(sphere6_handle)

result, sphere7_handle = vrep.simxGetObjectHandle(clientID, 'Dummy6', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get sphere 8 handle')
sphere_handles.append(sphere7_handle)


# Now obtain the initial position of the six joints, which is the initial position of the spheres
sphere_init_pos = []

result, p = vrep.simxGetObjectPosition(clientID,jaco_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get base position')
sphere_init_pos.append(p)

for i in range(6):
	result, p = vrep.simxGetObjectPosition(clientID,joint_handles[i],-1,vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint position')
	sphere_init_pos.append(p)

result, p = vrep.simxGetObjectPosition(clientID,sphere_handles[7],-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get object position')
sphere_init_pos.append(p)

sphere_new_pos = sphere_init_pos
sphere_r = np.array([0.15,0.15,0.15,0.15,0.15,0.15,0.15,0.20])
sphere_r = sphere_r / 2

# Set a path for the end effector to take
step_size = 0.005
for iter_idx in range(21):
	# This path is a straight line along the x-axis in world frame
	curr_p_end = p_end
	if iter_idx < 7:
		curr_p_end[0][0] = curr_p_end[0][0] + iter_idx*step_size
		curr_p_end[1][0] = curr_p_end[1][0] + iter_idx*step_size
	elif iter_idx < 14:
		curr_p_end[0][0] = curr_p_end[0][0] - (iter_idx-7)*step_size
		curr_p_end[1][0] = curr_p_end[1][0] - (iter_idx-7)*step_size
	else:
		curr_p_end[2][0] = curr_p_end[2][0] - (iter_idx-14)*step_size

	# Find the next goal pose
	goal_pose = T_from_R_p(R_end,curr_p_end)

	# obtain the current pose of the tool frame:
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
	curr_pose = T_from_R_p(R_end,p_end)

	# Use the IK function to find the desired joint angles
	error, joint_angles = inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
	joint_angles = OptimizeJointAngles(joint_angles)
	joint_angles = joint_angles + joints_zero_pos
	joint_angles = joint_angles % (np.pi*2)

	if error == 1:
		pprint('Cannot find an inverse kinematics solution within 100 iterations')
	elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
		pprint('Joint 2 out of valid range')
		pprint(joint_angles[1])
	elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
		pprint('Joint 3 out of valid range')
		pprint(joint_angles[2])
	else:
		for i in range(6):
			vrep.simxSetJointTargetPosition(clientID, joint_handles[i], joint_angles[i], vrep.simx_opmode_oneshot)
			# Wait two seconds
			#time.sleep(0.1)


	for i in range(5):
		result, p = vrep.simxGetObjectPosition(clientID,joint_handles[i+1],-1,vrep.simx_opmode_blocking)
		sphere_new_pos[i+2] = p
	
	for idx in range(7):
		result = vrep.simxSetObjectPosition(clientID,sphere_handles[idx],-1,sphere_new_pos[idx],vrep.simx_opmode_oneshot)

	# Object Collision Detection
	if CollisionCheck(sphere_new_pos,sphere_r) == 1:
		pprint('Collision With Object')
		pprint('Robot Halting')
		break

	time.sleep(0.2)


time.sleep(2)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)


	









