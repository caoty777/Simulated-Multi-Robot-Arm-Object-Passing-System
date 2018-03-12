import numpy as np
from scipy.linalg import expm
from scipy.linalg import logm

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


