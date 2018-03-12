import vrep
import time
import numpy as np
from scipy.linalg import expm
from helper_functions import skew_3, skew_6, skew2twist, adjoint, revolute_S, euler2mat, deg2rad, rad2deg, euler_to_screw_axis, base_to_world_frame_a_q, T_from_R_p, rot2euler
from get_handle import GetJointsHandle
from pprint import pprint

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Forward Kinematics
# A set of joint angles are given
joint_1_angle = np.pi/2
joint_2_angle = np.pi/2
joint_3_angle = np.pi/2
joint_4_angle = np.pi/2
joint_5_angle = np.pi/2
joint_6_angle = np.pi/2
joint_angles = np.array([joint_1_angle,joint_2_angle,joint_3_angle,joint_4_angle,joint_5_angle,joint_6_angle])

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

# First, find the screw axes for all 6 joints
# a,q without fk are w.r.t the base frame
# a_fk and q_fk are w.r.t the world frame
a1 = euler_to_screw_axis(deg2rad(-1.8000e+02),deg2rad(5.4277e-14),deg2rad(-3.4151e-06))
q1 = np.array([[-3.2723e-05],[-1.7315e-05],[7.8382e-02]])
a1_wf, q1_wf = base_to_world_frame_a_q(a1,q1,R_base_in_world,p_base_in_world)
S1 = revolute_S(a1_wf,q1_wf)
a2 = euler_to_screw_axis(deg2rad(-9.0000e+01),deg2rad(-7.2445e-06),deg2rad(2.4148e-06))
q2 = np.array([[-3.2693e-05],[-1.7308e-05],[1.9713e-01]])
a2_wf, q2_wf = base_to_world_frame_a_q(a2,q2,R_base_in_world,p_base_in_world)
S2 = revolute_S(a2_wf,q2_wf)
a3 = euler_to_screw_axis(deg2rad(9.0000e+01),deg2rad(9.6593e-06),deg2rad(9.6593e-06))
q3 = np.array([[-3.2663e-05],[-1.7196e-05],[6.0713e-01]])
a3_wf, q3_wf = base_to_world_frame_a_q(a3,q3,R_base_in_world,p_base_in_world)
S3 = revolute_S(a3_wf,q3_wf)
a4 = euler_to_screw_axis(deg2rad(1.8000e+02),deg2rad(1.6461e-05),deg2rad(-1.0245e-05))
q4 = np.array([[-3.2663e-05],[9.7829e-03],[8.1444e-01]])
a4_wf, q4_wf = base_to_world_frame_a_q(a4,q4,R_base_in_world,p_base_in_world)
S4 = revolute_S(a4_wf,q4_wf)
a5 = euler_to_screw_axis(deg2rad(1.2500e+02),deg2rad(1.5084e-05),deg2rad(1.1702e-05))
q5 = np.array([[-3.2663e-05],[4.4032e-02],[8.8027e-01]])
a5_wf, q5_wf = base_to_world_frame_a_q(a5,q5,R_base_in_world,p_base_in_world)
S5 = revolute_S(a5_wf,q5_wf)
a6 = euler_to_screw_axis(deg2rad(7.0000e+01),deg2rad(1.4016e-06),deg2rad(1.9864e-05))
q6 = np.array([[-3.2723e-05],[1.1769e-01],[8.9004e-01]])
a6_wf, q6_wf = base_to_world_frame_a_q(a6,q6,R_base_in_world,p_base_in_world)
S6 = revolute_S(a6_wf,q6_wf)

# Second, obtain the initial pose of the end effector (dummy7 frame)
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


# Third, use the matrix exponential mtehod to write down FK formulas
temp = np.matmul(expm(skew_6(S1)*joint_angles[0]), expm(skew_6(S2)*joint_angles[1]))
temp = np.matmul(temp, expm(skew_6(S3)*joint_angles[2]))
temp = np.matmul(temp, expm(skew_6(S4)*joint_angles[3]))
temp = np.matmul(temp, expm(skew_6(S5)*joint_angles[4]))
temp = np.matmul(temp, expm(skew_6(S6)*joint_angles[5]))
final_T = np.matmul(temp, M)

# Finally, print the expected pose into terminal
pprint(final_T)

# Put a dummy object to the predicted location to show the predicted final pose of the tool frame
# First, get the R and p from T
final_R = final_T[0:3,0:3]
final_p = final_T[0:3,3]
final_a,final_b,final_g = rot2euler(final_R)
final_euler = [final_a,final_b,final_g]

result, dummy_ball_handle = vrep.simxGetObjectHandle(clientID, 'Dummy', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get dummy ball handle')

time.sleep(1)

result = vrep.simxSetObjectOrientation(clientID,dummy_ball_handle,-1,final_euler,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not set dummy ball orientation')

time.sleep(1)

result = vrep.simxSetObjectPosition(clientID,dummy_ball_handle,-1,final_p,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not set dummy ball position')

time.sleep(1)
# Next, set all the joints to the given joint angles
# Obtain the handles of the six joints by calling the helper function GetJointsHandle in another file 'get_handle.py'
joint_one_handle,joint_two_handle,joint_three_handle,joint_four_handle,joint_five_handle,joint_six_handle = GetJointsHandle(clientID)

time.sleep(1)
# create a list to store the current joint angles of all six joints
current_theta = np.zeros(6)

# Iterate the following code block 6 times to move each individual joint one by one
for i in range(6):
	if i == 0:
		curr_handle = joint_one_handle
	elif i == 1:
		curr_handle = joint_two_handle
	elif i == 2:
		curr_handle = joint_three_handle
	elif i == 3:
		curr_handle = joint_four_handle
	elif i == 4:
		curr_handle = joint_five_handle
	elif i == 5:
		curr_handle = joint_six_handle

	# Get the current value of the current joint
	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint variable #{}'.format(i+1))

	# Wait two seconds
	time.sleep(1)

	# Set the desired value of the current joint variable
	vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + joint_angles[i], vrep.simx_opmode_oneshot)

	# Wait two seconds
	time.sleep(1)

	# Get the new value of the current joint
	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
	current_theta[i] = theta
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint variable #{}'.format(i+1))
	print('New value - joint #{}: theta = {:f}'.format(i+1,theta))



time.sleep(1)

# Print the actual pose of tool frame (dummy7) in the end
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
T_actual = T_from_R_p(R_end,p_end)
pprint(T_actual)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
