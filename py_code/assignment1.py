import vrep
import time
import numpy as np

from get_handle import GetJointsHandle

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Obtain the handles of the six joints by calling the helper function GetJointsHandle in another file 'get_handle.py'
joint_one_handle,joint_two_handle,joint_three_handle,joint_four_handle,joint_five_handle,joint_six_handle = GetJointsHandle(clientID)

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
	# Wait two seconds
	time.sleep(1)

	# Get the current value of the current joint variable
	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get  joint variable #{}'.format(i+1))
	print('Original value - joint #{}: theta = {:f}'.format(i+1,theta))

	# Set the desired value of the current joint variable
	vrep.simxSetJointTargetPosition(clientID, curr_handle, theta + (np.pi / 2), vrep.simx_opmode_oneshot)

	# Wait two seconds
	time.sleep(1)

	# Get the current value of the current joint variable
	result, theta = vrep.simxGetJointPosition(clientID, curr_handle, vrep.simx_opmode_blocking)
	current_theta[i] = theta
	if result != vrep.simx_return_ok:
		raise Exception('could not get joint variable #{}'.formay(i+1))
	print('New value - joint #{}: theta = {:f}'.format(i+1,theta))

# Now, make all six joints move at the same time, back to their original position
vrep.simxSetJointTargetPosition(clientID, joint_one_handle, current_theta[0] - (np.pi / 2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_two_handle, current_theta[1] - (np.pi / 2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, current_theta[2] - (np.pi / 2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_four_handle, current_theta[3] - (np.pi / 2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_five_handle, current_theta[4] - (np.pi / 2), vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_six_handle, current_theta[5] - (np.pi / 2), vrep.simx_opmode_oneshot)

# Wait for two seconds
time.sleep(1)

# Now do something extra: rotate joint 3 back and forth so that the arm looks like waving
result, current_theta[2] = vrep.simxGetJointPosition(clientID, joint_three_handle, vrep.simx_opmode_blocking)

center_theta = current_theta[2]
for i in range(3):
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, center_theta + (np.pi / 4), vrep.simx_opmode_oneshot)
	time.sleep(1.4)
	vrep.simxSetJointTargetPosition(clientID, joint_three_handle, center_theta - (np.pi / 4), vrep.simx_opmode_oneshot)
	time.sleep(1.4)
# Finally, move joint 3 back to its original position
vrep.simxSetJointTargetPosition(clientID, joint_three_handle, center_theta, vrep.simx_opmode_oneshot)

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
