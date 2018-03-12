import vrep
import time
import numpy as np

def GetJointsHandle(clientID):
	# Get "handle" to the first joint of robot
	result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint1', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for first joint')

	# Get "handle" to the second joint of robot
	result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint2', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for second joint')

	# Get "handle" to the third joint of robot
	result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint3', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for third joint')

	# Get "handle" to the fourth joint of robot
	result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint4', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for fourth joint')

	# Get "handle" to the fifth joint of robot
	result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint5', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for fifth joint')

	# Get "handle" to the sixth joint of robot
	result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint6', vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
		raise Exception('could not get object handle for sixth joint')

	return joint_one_handle,joint_two_handle,joint_three_handle,joint_four_handle,joint_five_handle,joint_six_handle

