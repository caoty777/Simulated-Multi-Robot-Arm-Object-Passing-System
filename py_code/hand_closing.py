import vrep
import time
import numpy as np
from pprint import pprint

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')


# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

result,j0 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor1',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 1')
result,j1 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor2',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 2')
result,j2 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor1',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 3')
result,j3 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor2',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 4')

closingVel = -0.04

for i in range(5):

    # Close the gripper
    result = vrep.simxSetJointTargetVelocity(clientID,j0,closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j1,closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j2,closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j3,closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')

    time.sleep(1.5)

    # Open the gripper
    result = vrep.simxSetJointTargetVelocity(clientID,j0,-closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j1,-closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j2,-closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')
    result = vrep.simxSetJointTargetVelocity(clientID,j3,-closingVel,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not set joint velocity')

    time.sleep(1.5)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
