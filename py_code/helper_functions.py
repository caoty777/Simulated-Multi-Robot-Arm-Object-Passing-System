import numpy as np
import vrep
import time
import random
from scipy.linalg import expm, logm
from pprint import pprint


def skew_6(x):
    result = np.array([[0,-x[2][0],x[1][0],x[3][0]],[x[2][0],0,-x[0][0],x[4][0]],[-x[1][0],x[0][0],0,x[5][0]],[0,0,0,0]])
    return result

def skew_3(x):
    result = np.array([[0,-x[2][0],x[1][0]],[x[2][0],0,-x[0][0]],[-x[1][0],x[0][0],0]])
    return result

def deskew_3(M):
    c = M[1][0]
    b = -M[2][0]
    a = M[2][1]
    result = np.array([[a],[b],[c]])
    return result

def deskew_6(M):
    w_bracket = M[0:3,0:3]
    v = np.reshape(M[0:3,3],(3,1))
    w = deskew_3(w_bracket)
    V = np.concatenate((w,v))
    return V

def skew2twist(M):
    w_bracket = M[0:3,0:3]
    w = deskew_3(w_bracket)
    v = np.reshape(M[0:3,3],(3,1))
    V = np.concatenate((w,v))
    return V

def adjoint(t):
    R = t[0:3,0:3]
    p = np.resize(t[0:3,3],(3,1))
    temp = np.concatenate((R,np.zeros((3,3))),axis=1)
    a = np.matmul(skew_3(p),R)
    temp2 = np.concatenate((a,R),axis=1)
    result = np.concatenate((temp,temp2))
    return result

def revolute_S(a,q):
    temp = np.matmul(-skew_3(a),q)
    S = np.concatenate((a,temp))
    return S

def euler2mat(a,b,g):
    Rx = np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
    Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
    Rz = np.array([[np.cos(g),-np.sin(g),0],[np.sin(g),np.cos(g),0],[0,0,1]])
    temp = np.matmul(Ry,Rz)
    R = np.matmul(Rx,temp)
    return R

def rot2euler(R):
    r00 = R[0][0]
    r01 = R[0][1]
    r02 = R[0][2]
    r12 = R[1][2]
    r22 = R[2][2]
    r10 = R[1][0]
    r11 = R[1][1]
    
    if r02 < 1:
        if r02 > -1:
            b = np.arcsin(r02)
            a = np.arctan2(-r12,r22)
            g = np.arctan2(-r01,r00)
        else:
            b = -np.pi/2
            a = np.arctan2(r10,r11)
            g = 0
    else:
        b = np.pi/2
        a = np.arctan2(r10,r11)
        g = 0

    return a,b,g

def deg2rad(deg):
    rad = deg * np.pi / 180
    return rad

def rad2deg(rad):
    deg = rad * 180 / np.pi
    return deg

def euler_to_screw_axis(a,b,g):
    R = euler2mat(a,b,g)
    a = np.reshape(R[:,2],(3,1))
    return a

def base_to_world_frame_a_q(a,q,R_base_in_world,p_base_in_world):
    a_wf = np.matmul(R_base_in_world, a)
    temp = np.matmul(R_base_in_world, q)
    q_wf = temp + p_base_in_world
    return a_wf,q_wf

def T_from_R_p(R,p):
    up = np.concatenate((R,p),axis=1)
    down = np.array([[0,0,0,1]])
    T = np.concatenate((up,down))
    return T

def get_S(R_base_in_world,p_base_in_world):
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
    return S1,S2,S3,S4,S5,S6

def forward_kinematics(joint_angles,S1,S2,S3,S4,S5,S6,M):
    #pprint(joint_angles[0])
    #pprint(joint_angles[1])
    temp = np.matmul(expm(skew_6(S1)*joint_angles[0]), expm(skew_6(S2)*joint_angles[1]))
    temp = np.matmul(temp, expm(skew_6(S3)*joint_angles[2]))
    temp = np.matmul(temp, expm(skew_6(S4)*joint_angles[3]))
    temp = np.matmul(temp, expm(skew_6(S5)*joint_angles[4]))
    temp = np.matmul(temp, expm(skew_6(S6)*joint_angles[5]))
    final_T = np.matmul(temp, M)
    return final_T

def isMtxSame(M1,M2):
    num_rows = M1.shape[0]
    num_cols = M1.shape[1]
    flag = 1
    for x in range(num_rows):
        if flag == 0:
            break
        for y in range(num_cols):
            if M1[x][y] != M2[x][y]:
                flag = 0
                break
    return flag

def jacobian(S1,S2,S3,S4,S5,S6,joint_angles):
    S = np.concatenate((S1,S2,S3,S4,S5,S6),axis=1)
    J = S1
    temp = np.identity(4)
    for i in range(1,6):
        temp = np.matmul(temp,expm(skew_6(np.reshape(S[:,i-1],(6,1)))*joint_angles[i-1]))
        J_entry = np.matmul(adjoint(temp),np.reshape(S[:,i],(6,1)))
        J = np.concatenate((J,J_entry),axis=1)
    return J


def inverse_kinematics(clientID,M,goal_pose,curr_pose,joints_zero_pos,joint_handles,S1,S2,S3,S4,S5,S6):
    # initialize six joint angles:
    # If the current tool frame is already close to the goal pose, use the current joint angles
    # as the initial guess. Otherwise, randomize six angles
    error = 1
    for idx in range(5):
        if error == 0:
            break
        temp = np.matmul(goal_pose,np.linalg.inv(curr_pose))
        V_bracket = logm(temp)
        V = deskew_6(V_bracket)
        if np.linalg.norm(V) < 1:
            joint_angles = GetAllJointAngles(clientID,joints_zero_pos,joint_handles)
        else:
            theta1 = deg2rad(random.randint(-180,180))
            theta2 = deg2rad(random.randint(47-180,313-180))
            theta3 = deg2rad(random.randint(19-180,341-180))
            theta4 = deg2rad(random.randint(-180,180))
            theta5 = deg2rad(random.randint(-180,180))
            theta6 = deg2rad(random.randint(-180,180))
            joint_angles = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

        # Now, start the numeric IK algorithm
        for i in range(100):
            curr_T = forward_kinematics(joint_angles,S1,S2,S3,S4,S5,S6,M)
            temp = np.matmul(goal_pose,np.linalg.inv(curr_T))
            V_bracket = logm(temp)
            V = deskew_6(V_bracket)
            if np.linalg.norm(V) < 0.01:
                error = 0
                break
            # now find the Jacobian matrix
            J = jacobian(S1,S2,S3,S4,S5,S6,joint_angles)
            tem = np.matmul(J.T,J) + 0.01*np.identity(6)
            tem = np.linalg.inv(tem)
            tem2 = np.matmul(J.T,V)
            thetadot = np.matmul(tem,tem2)
            #thetadot = np.matmul(np.linalg.inv(J),V)
            # update theta
            joint_angles[0] = joint_angles[0] + thetadot[0][0]
            joint_angles[1] = joint_angles[1] + thetadot[1][0]
            joint_angles[2] = joint_angles[2] + thetadot[2][0]
            joint_angles[3] = joint_angles[3] + thetadot[3][0]
            joint_angles[4] = joint_angles[4] + thetadot[4][0]
            joint_angles[5] = joint_angles[5] + thetadot[5][0]

    return error,joint_angles

def GetAllJointAngles(clientID,joints_zero_pos,joint_handles):
    thetas = np.zeros(6)
    for i in range(6):
        # Get the current value of the current joint
        result, theta = vrep.simxGetJointPosition(clientID, joint_handles[i], vrep.simx_opmode_blocking)
        if result != vrep.simx_return_ok:
            raise Exception('could not get joint variable #{}'.format(i+1))
        thetas[i] = theta - joints_zero_pos[i]
    return thetas

def OptimizeJointAngles(thetas):
    for i in range(6):
        if i != 1 and i != 2:
            if thetas[i] >= np.pi and thetas[i] <= 2*np.pi:
                thetas[i] = thetas[i] - 2*np.pi
            elif thetas[i] >= -2*np.pi and thetas[i] <= -np.pi:
                thetas[i] = thetas[i] + 2*np.pi
    return thetas

#def UpdateSpherePos(sphere_init_pos,thetas,S1,S2,S3,S4,S5,S6):
    #sphere_new_pos = sphere_init_pos
    #tem_init_pos_2 = np.concatenate((sphere_init_pos[2],np.array([1])))
    #temp = expm(skew_6(S1)*thetas[0])
    #tem_new_pos_2 = np.matmul(temp,tem_init_pos_2)
    #sphere_new_pos[2] = tem_new_pos_2[0:3]

    #tem_init_pos_3 = np.concatenate((sphere_init_pos[3],np.array([1])))
    #temp = np.matmul(temp,expm(skew_6(S2)*thetas[1]))
    #tem_new_pos_3 = np.matmul(temp,tem_init_pos_3)
    #sphere_new_pos[3] = tem_new_pos_3[0:3]

    #tem_init_pos_4 = np.concatenate((sphere_init_pos[4],np.array([1])))
    #temp = np.matmul(temp,expm(skew_6(S3)*thetas[2]))
    #tem_new_pos_4 = np.matmul(temp,tem_init_pos_4)
    #sphere_new_pos[4] = tem_new_pos_4[0:3]

    #tem_init_pos_5 = np.concatenate((sphere_init_pos[5],np.array([1])))
    #temp = np.matmul(temp,expm(skew_6(S4)*thetas[3]))
    #tem_new_pos_5 = np.matmul(temp,tem_init_pos_5)
    #sphere_new_pos[5] = tem_new_pos_5[0:3]

    #tem_init_pos_6 = np.concatenate((sphere_init_pos[6],np.array([1])))
    #temp = np.matmul(temp,expm(skew_6(S5)*thetas[4]))
    #tem_new_pos_6 = np.matmul(temp,tem_init_pos_6)
    #sphere_new_pos[6] = tem_new_pos_6[0:3]



    #return sphere_new_pos

def IsTwoBallCollision(p1,p2,r1,r2):
    result = 1
    diff = np.array([0.0,0.0,0.0])
    diff[0] = p1[0] - p2[0]
    diff[1] = p1[1] - p2[1]
    diff[2] = p1[2] - p2[2]
    if np.linalg.norm(diff) > (r1 + r2):
        result = 0
    return result

def CollisionCheck(sphere_p,sphere_r):
    result = 0
    for i in range(7):
        if IsTwoBallCollision(sphere_p[i],sphere_p[7],sphere_r[i],sphere_r[7]) == 1:
            result = 1
            break
            
    return result



def GetJointsHandle(clientID):
    joint_handles = []
    # Get "handle" to the first joint of robot
    result, joint_one_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint1', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for first joint')
    else:
        joint_handles.append(joint_one_handle)

    # Get "handle" to the second joint of robot
    result, joint_two_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint2', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for second joint')
    else:
        joint_handles.append(joint_two_handle)

    # Get "handle" to the third joint of robot
    result, joint_three_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint3', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for third joint')
    else:
        joint_handles.append(joint_three_handle)

    # Get "handle" to the fourth joint of robot
    result, joint_four_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint4', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for fourth joint')
    else:
        joint_handles.append(joint_four_handle)

    # Get "handle" to the fifth joint of robot
    result, joint_five_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint5', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for fifth joint')
    else:
        joint_handles.append(joint_five_handle)

    # Get "handle" to the sixth joint of robot
    result, joint_six_handle = vrep.simxGetObjectHandle(clientID, 'Jaco_joint6', vrep.simx_opmode_blocking)
    if result != vrep.simx_return_ok:
        raise Exception('could not get object handle for sixth joint')
    else:
        joint_handles.append(joint_six_handle)

    return joint_handles

