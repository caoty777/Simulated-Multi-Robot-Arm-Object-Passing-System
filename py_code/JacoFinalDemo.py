import vrep
import time
import numpy as np
import random
from scipy.linalg import expm, logm
from pprint import pprint
from helper_functions import skew_3, skew_6, skew2twist, adjoint, revolute_S, euler2mat, deg2rad
from helper_functions import rad2deg, euler_to_screw_axis, base_to_world_frame_a_q
from helper_functions import T_from_R_p, rot2euler, get_S, forward_kinematics, isMtxSame, GetJointsHandle1
from helper_functions import inverse_kinematics, OptimizeJointAngles, GetJointsHandle, GetJointsHandle0

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# get joint handles for all the jaco arms
joint_handles = GetJointsHandle(clientID)
joint_handles_0 = GetJointsHandle0(clientID)


####### Initialization Process #######
result, connector = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachPoint', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get connector handle')

result, objectSensor = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachProxSensor', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get objectSensor handle')

result, connector = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachPoint#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get connector handle')

result, objectSensor = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachProxSensor#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get objectSensor handle')


# obtain the initial zero position angles for the six joints
joints_zero_pos = np.zeros(6)
for i in range(6):
    # Get the current value of the current joint
    result, theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get joint variable #{}'.format(i+1))
    joints_zero_pos[i] = theta


result, item_handle = vrep.simxGetObjectHandle(clientID, 'Item', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle handle')

# Obtain the initial pose of the end effector (dummy7 frame)
# Get the handle of dummy7 object
result, jacohand_dummy7_handle = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 handle')

# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_handle, -1, vrep.simx_opmode_blocking)
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


# Obtain the initial pose of the end effector (dummy7 frame)
# Get the handle of dummy7 object
result, jacohand_dummy7_0_handle = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy7#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 handle')

# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_0_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_0_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 orientation')

p_end_0 = np.reshape(p,(3,1))
R_end_0 = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
M_0 = T_from_R_p(R_end_0,p_end_0)

# Get the R and p for the base frame (Jaco) w.r.t the world frame
# Get the handle of Jaco object
result, jaco_handle_0 = vrep.simxGetObjectHandle(clientID, 'Jaco#0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco handle')

# Get the position of base frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jaco_handle_0,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco position')

# Get the orientation of base w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jaco_handle_0, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco orientation')

R_base_in_world_0 = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
p_base_in_world_0 = np.reshape(p,(3,1))

# get S - screw axes
S1_0,S2_0,S3_0,S4_0,S5_0,S6_0 = get_S(R_base_in_world_0,p_base_in_world_0)

result,j0_0 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor1#0',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 1')
result,j1_0 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor2#0',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 2')
result,j2_0 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor1#0',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 3')
result,j3_0 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor2#0',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 4')

closingVel = -0.05


####### Step 1 #########
####### Make the Jaco#0 hand move to dummy12 pose (intial receiving pose) #######
result, dummy12_0_handle = vrep.simxGetObjectHandle(clientID, 'Dummy12', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle handle')

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,dummy12_0_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy12_0_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal orientation')

p_goal = np.reshape(p,(3,1))
R_goal = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
goal_pose = T_from_R_p(R_goal,p_goal)

error, joint_angles_0 = inverse_kinematics(clientID,M_0,goal_pose,M_0,joints_zero_pos,joint_handles_0,S1_0,S2_0,S3_0,S4_0,S5_0,S6_0)
joint_angles_0 = OptimizeJointAngles(joint_angles_0)
joint_angles_0 = joint_angles_0 + joints_zero_pos
#joint_angles = joint_angles % (np.pi*2)
if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles_0[1]) < 47 or rad2deg(joint_angles_0[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles_0[1])
elif rad2deg(joint_angles_0[2]) < 19 or rad2deg(joint_angles_0[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles_0[2])
else:
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[0], joint_angles_0[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[1], joint_angles_0[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[2], joint_angles_0[2], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[3], joint_angles_0[3], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[4], joint_angles_0[4], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[5], joint_angles_0[5], vrep.simx_opmode_oneshot)


# Open the gripper of Jaco #0
result = vrep.simxSetJointTargetVelocity(clientID,j0_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
   raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j1_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
   raise Exception('could not set joint velocity')
# result = vrep.simxSetJointTargetVelocity(clientID,j2_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#    raise Exception('could not set joint velocity')
# result = vrep.simxSetJointTargetVelocity(clientID,j3_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#    raise Exception('could not set joint velocity')

# # Open the gripper of Jaco #0
# result = vrep.simxSetJointForce(clientID,j0_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#     raise Exception('could not set joint velocity')
# result = vrep.simxSetJointForce(clientID,j1_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#     raise Exception('could not set joint velocity')
# result = vrep.simxSetJointForce(clientID,j2_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#     raise Exception('could not set joint velocity')
# result = vrep.simxSetJointForce(clientID,j3_0,-closingVel,vrep.simx_opmode_blocking)
# if result != vrep.simx_return_ok:
#     raise Exception('could not set joint velocity')



####### Step 2 #######
####### Jaco hand goes to a pose right above the item, then lower down #######
result, p = vrep.simxGetObjectPosition(clientID,item_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle position')

# Get the orientation of item_handle w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, item_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle orientation')

item_p = np.reshape(p,(3,1))
item_R = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])


item_p[2][0] = item_p[2][0] + 0.10+0.048
for i in range(3):
    # obtain the current pose of the tool frame:
    # Get the position of dummy7 frame w.r.t the world frame
    result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_handle,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')

    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_handle, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')

    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)

    item_p[2][0] = item_p[2][0] - 0.048
    goal_pose = T_from_R_p(item_R,item_p)

    # Use the IK function to find the desired joint angles
    error, joint_angles = inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos

    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[5], joint_angles[5], vrep.simx_opmode_oneshot)
        if i == 0:
            time.sleep(4)
        else:
            time.sleep(0.1)

time.sleep(1)

# Close the gripper of Jaco
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

time.sleep(1)


####### Step 3 #######
####### Jaco hand moves to Dummy7 pose #######
result, dummy7_handle = vrep.simxGetObjectHandle(clientID, 'Dummy7', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Dummy7 handle')

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,dummy7_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Dummy7 position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy7_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Dummy7 orientation')

p_goal = np.reshape(p,(3,1))
p_goal[2][0] = p_goal[2][0] + 0.08
R_goal = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])

for i in range(4):
    p_goal[2][0] = p_goal[2][0] - 0.02
    goal_pose = T_from_R_p(R_goal,p_goal)

    result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_handle,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')

    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_handle, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')

    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)

    error, joint_angles = inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos

    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[5], joint_angles[5], vrep.simx_opmode_oneshot)
        if i == 0:
            time.sleep(7)
        else:
            time.sleep(0.1)

time.sleep(2)


####### Step 4 #######
####### Item transfer between Jaco and Jaco#0 #######
# Close the gripper of Jaco #0
result = vrep.simxSetJointTargetVelocity(clientID,j0_0,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j1_0,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j2_0,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j3_0,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

time.sleep(2)

# Open the gripper of Jaco
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

time.sleep(5)



####### Step 5 #######
####### Jaco Arm goes back to its zero pose #######

for i in range(3):
    p_goal[2][0] = p_goal[2][0] + 0.02
    goal_pose = T_from_R_p(R_goal,p_goal)
    
    result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_handle,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')
    
    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_handle, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')
    
    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)
    
    error, joint_angles = inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos
    
    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles[5], joint_angles[5], vrep.simx_opmode_oneshot)
        time.sleep(0.2)

vrep.simxSetJointTargetPosition(clientID, joint_handles[0], joints_zero_pos[0], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles[1], joints_zero_pos[1], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joints_zero_pos[2], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles[3], joints_zero_pos[3], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles[4], joints_zero_pos[4], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles[5], joints_zero_pos[5], vrep.simx_opmode_oneshot)

time.sleep(4)


####### Step 6 #######
####### Jaco hand #0 goes to Dummy12_0 pose #######
result, dummy12_0_handle = vrep.simxGetObjectHandle(clientID, 'Dummy12_0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle handle')

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,dummy12_0_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy12_0_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal orientation')

p_goal = np.reshape(p,(3,1))
R_goal = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
goal_pose = T_from_R_p(R_goal,p_goal)

# Get the current jacohand#0 pose
result, p = vrep.simxGetObjectPosition(clientID,jacohand_dummy7_0_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jacohand_dummy7_0_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 orientation')

p_end = np.reshape(p,(3,1))
R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
curr_pose = T_from_R_p(R_end,p_end)

error, joint_angles_0 = inverse_kinematics(clientID,M_0,goal_pose,curr_pose,joints_zero_pos,joint_handles_0,S1_0,S2_0,S3_0,S4_0,S5_0,S6_0)
joint_angles_0 = OptimizeJointAngles(joint_angles_0)
joint_angles_0 = joint_angles_0 + joints_zero_pos

if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles_0[1]) < 47 or rad2deg(joint_angles_0[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles_0[1])
elif rad2deg(joint_angles_0[2]) < 19 or rad2deg(joint_angles_0[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles_0[2])
else:
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[0], joint_angles_0[0], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[1], joint_angles_0[1], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[2], joint_angles_0[2], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[3], joint_angles_0[3], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[4], joint_angles_0[4], vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[5], joint_angles_0[5], vrep.simx_opmode_oneshot)

time.sleep(7)



# Jaco0 comes to play
joint_handles0 = GetJointsHandle1(clientID)

result, connector = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachPoint0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get connector handle')

result, objectSensor = vrep.simxGetObjectHandle(clientID, 'JacoHand_attachProxSensor0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get objectSensor handle')

result, dummy_handle0 = vrep.simxGetObjectHandle(clientID, 'JacoHand_Dummy15', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 handle')

# Get the position of dummy7 frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,dummy_handle0,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 position')

# Get the orientation of dummy7 w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle0, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get dummy7 orientation')

p_end0 = np.reshape(p,(3,1))
R_end0 = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
M0 = T_from_R_p(R_end0,p_end0)

result, jaco_handle0 = vrep.simxGetObjectHandle(clientID, 'Jaco0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco handle')

# Get the position of base frame w.r.t the world frame
result, p = vrep.simxGetObjectPosition(clientID,jaco_handle0,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco position')

# Get the orientation of base w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, jaco_handle0, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get Jaco orientation')

R_base_in_world0 = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
p_base_in_world0 = np.reshape(p,(3,1))

S10,S20,S30,S40,S50,S60 = get_S(R_base_in_world0,p_base_in_world0)

result,j00 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor3',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 1')
result,j10 = vrep.simxGetObjectHandle(clientID,'JacoHand_fingers12_motor4',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 2')
result,j20 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor3',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 3')
result,j30 = vrep.simxGetObjectHandle(clientID,'JacoHand_finger3_motor4',vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get handle 4')


# Open the gripper of Jaco0
result = vrep.simxSetJointTargetVelocity(clientID,j00,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j10,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j20,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j30,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')



result, goal_item_handle = vrep.simxGetObjectHandle(clientID, 'Dummy7_0', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get item handle')

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,goal_item_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get goal position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, goal_item_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
	raise Exception('could not get goal orientation')

p_goal = np.reshape(p,(3,1))
p_goal[2][0] = p_goal[2][0] + 0.007*10
R_goal = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
goal_pose = T_from_R_p(R_goal,p_goal)

error, joint_angles = inverse_kinematics(clientID,M0,goal_pose,M0,joints_zero_pos,joint_handles0,S10,S20,S30,S40,S50,S60)
joint_angles = OptimizeJointAngles(joint_angles)
joint_angles = joint_angles + joints_zero_pos

if error == 1:
    pprint('Cannot find an inverse kinematics solution within 100 iterations')
elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
    pprint('Joint 2 out of valid range')
    pprint(joint_angles[1])
elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
    pprint('Joint 3 out of valid range')
    pprint(joint_angles[2])
else:
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joint_angles[0], vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joint_angles[1], vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joint_angles[2], vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joint_angles[3], vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joint_angles[4], vrep.simx_opmode_oneshot)
	vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joint_angles[5], vrep.simx_opmode_oneshot)

time.sleep(5)

for i in range(4):
	p_goal[2][0] = p_goal[2][0] -0.02
	goal_pose = T_from_R_p(R_goal,p_goal)

	result, p = vrep.simxGetObjectPosition(clientID,dummy_handle0,-1,vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get dummy7 position')

	# Get the orientation of dummy7 w.r.t the world frame
	result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle0, -1, vrep.simx_opmode_blocking)
	if result != vrep.simx_return_ok:
	    raise Exception('could not get dummy7 orientation')

	p_end = np.reshape(p,(3,1))
	R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
	curr_pose = T_from_R_p(R_end,p_end)

	error, joint_angles = inverse_kinematics(clientID,M0,goal_pose,curr_pose,joints_zero_pos,joint_handles0,S10,S20,S30,S40,S50,S60)
	joint_angles = OptimizeJointAngles(joint_angles)
	joint_angles = joint_angles + joints_zero_pos

	if error == 1:
	    pprint('Cannot find an inverse kinematics solution within 100 iterations')
	elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
	    pprint('Joint 2 out of valid range')
	    pprint(joint_angles[1])
	elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
	    pprint('Joint 3 out of valid range')
	    pprint(joint_angles[2])
	else:
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joint_angles[0], vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joint_angles[1], vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joint_angles[2], vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joint_angles[3], vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joint_angles[4], vrep.simx_opmode_oneshot)
		vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joint_angles[5], vrep.simx_opmode_oneshot)
		time.sleep(0.1)

time.sleep(1)

# Close the gripper of Jaco0
result = vrep.simxSetJointTargetVelocity(clientID,j00,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j10,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j20,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j30,closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

time.sleep(1)

# Open the gripper of Jaco #0
result = vrep.simxSetJointTargetVelocity(clientID,j0_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j1_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j2_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j3_0,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

time.sleep(5)


# Jaco0 hand goes straight up for several steps
for i in range(3):
    p_goal[2][0] = p_goal[2][0] + 0.02
    goal_pose = T_from_R_p(R_goal,p_goal)
    
    result, p = vrep.simxGetObjectPosition(clientID,dummy_handle0,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')
    
    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle0, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')
    
    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)
    
    error, joint_angles = inverse_kinematics(clientID,M0,goal_pose,curr_pose,joints_zero_pos,joint_handles0,S10,S20,S30,S40,S50,S60)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos
    
    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joint_angles[5], vrep.simx_opmode_oneshot)
        time.sleep(0.1)

time.sleep(1)


# Jaco0 hand goes back to its zero position
vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joints_zero_pos[0], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joints_zero_pos[1], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joints_zero_pos[2], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joints_zero_pos[3], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joints_zero_pos[4], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joints_zero_pos[5], vrep.simx_opmode_oneshot)

time.sleep(5)

# Jaco#0 hand goes back to its zero position
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[0], joints_zero_pos[0], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[1], joints_zero_pos[1], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[2], joints_zero_pos[2], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[3], joints_zero_pos[3], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[4], joints_zero_pos[4], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles_0[5], joints_zero_pos[5], vrep.simx_opmode_oneshot)

time.sleep(5)

####### Step Final #######
####### Jaco0 hand goes to a pose right above the FinalPoint, then lower down #######

result, dummy12_0_handle = vrep.simxGetObjectHandle(clientID, 'FinalPoint', vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get item_handle handle')

# Get the current goal pose
result, p = vrep.simxGetObjectPosition(clientID,dummy12_0_handle,-1,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal position')

# Get the orientation of dummy ball w.r.t the world frame
result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy12_0_handle, -1, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get goal orientation')

p_goal = np.reshape(p,(3,1))
R_goal = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])



p_goal[2][0] = p_goal[2][0] + 0.12
for i in range(3):
    # obtain the current pose of the tool frame:
    # Get the position of dummy7 frame w.r.t the world frame
    result, p = vrep.simxGetObjectPosition(clientID,dummy_handle0,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')
    
    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle0, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')
    
    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)
    
    p_goal[2][0] = p_goal[2][0] - 0.04
    goal_pose = T_from_R_p(R_goal,p_goal)

    # Use the IK function to find the desired joint angles
    error, joint_angles = inverse_kinematics(clientID,M0,goal_pose,curr_pose,joints_zero_pos,joint_handles0,S10,S20,S30,S40,S50,S60)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos
    
    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joint_angles[5], vrep.simx_opmode_oneshot)
        if i == 0:
            time.sleep(8)
        else:
            time.sleep(0.1)


# Open the gripper of Jaco0
result = vrep.simxSetJointTargetVelocity(clientID,j00,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j10,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j20,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')
result = vrep.simxSetJointTargetVelocity(clientID,j30,-closingVel,vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not set joint velocity')

time.sleep(3)


for i in range(3):
    # obtain the current pose of the tool frame:
    # Get the position of dummy7 frame w.r.t the world frame
    result, p = vrep.simxGetObjectPosition(clientID,dummy_handle0,-1,vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 position')
    
    # Get the orientation of dummy7 w.r.t the world frame
    result , euler_angles = vrep.simxGetObjectOrientation(clientID, dummy_handle0, -1, vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get dummy7 orientation')
    
    p_end = np.reshape(p,(3,1))
    R_end = euler2mat(euler_angles[0],euler_angles[1],euler_angles[2])
    curr_pose = T_from_R_p(R_end,p_end)
    
    p_goal[2][0] = p_goal[2][0] + 0.04
    goal_pose = T_from_R_p(R_goal,p_goal)

    # Use the IK function to find the desired joint angles
    error, joint_angles = inverse_kinematics(clientID,M0,goal_pose,curr_pose,joints_zero_pos,joint_handles0,S10,S20,S30,S40,S50,S60)
    joint_angles = OptimizeJointAngles(joint_angles)
    joint_angles = joint_angles + joints_zero_pos
    
    if error == 1:
        pprint('Cannot find an inverse kinematics solution within 100 iterations')
    elif rad2deg(joint_angles[1]) < 47 or rad2deg(joint_angles[1]) > 313:
        pprint('Joint 2 out of valid range')
        pprint(joint_angles[1])
    elif rad2deg(joint_angles[2]) < 19 or rad2deg(joint_angles[2]) > 341:
        pprint('Joint 3 out of valid range')
        pprint(joint_angles[2])
    else:
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joint_angles[0], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joint_angles[1], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joint_angles[2], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joint_angles[3], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joint_angles[4], vrep.simx_opmode_oneshot)
        vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joint_angles[5], vrep.simx_opmode_oneshot)
        time.sleep(0.1)

# Jaco0 hand goes back to its zero position
vrep.simxSetJointTargetPosition(clientID, joint_handles0[0], joints_zero_pos[0], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[1], joints_zero_pos[1], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joints_zero_pos[2], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[3], joints_zero_pos[3], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[4], joints_zero_pos[4], vrep.simx_opmode_oneshot)
vrep.simxSetJointTargetPosition(clientID, joint_handles0[5], joints_zero_pos[5], vrep.simx_opmode_oneshot)

time.sleep(4)

for i in range(3):
    vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joints_zero_pos[2] + (np.pi/4), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[2], joints_zero_pos[2] + (np.pi/4), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joints_zero_pos[2] + (np.pi/4), vrep.simx_opmode_oneshot)
    if i == 0:
        time.sleep(0.7)
    else:
        time.sleep(1.4)
    vrep.simxSetJointTargetPosition(clientID, joint_handles[2], joints_zero_pos[2] - (np.pi/4), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles_0[2], joints_zero_pos[2] - (np.pi/4), vrep.simx_opmode_oneshot)
    vrep.simxSetJointTargetPosition(clientID, joint_handles0[2], joints_zero_pos[2] - (np.pi/4), vrep.simx_opmode_oneshot)
    time.sleep(1.4)


# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)

