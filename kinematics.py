import numpy as np
import math
#expm is a matrix exponential function

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""
L1=118.2
L2=99.8
L3=110
L4=102   


def FK_dh(joint_angles, link):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm using DH convention

    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    a=np.zeros(7)
    alpha=np.zeros(7)
    d=np.zeros(7)
    theta=np.zeros(7)
    
    alpha[0]=math.pi/2
    d[0]=L1
    theta[0]=joint_angles[0]
    
    a[1]=L2
    theta[1]=joint_angles[1]+np.pi/2
    
    a[2]=L3
    theta[2]=joint_angles[2]
    
    alpha[3]=np.pi/2
    theta[3]=np.pi/2
    
    theta[4]=joint_angles[3]
    alpha[4]=-np.pi/2
    
    alpha[5]=np.pi/2
    theta[5]=joint_angles[4]
    
    d[6]=L4
    theta[6]=joint_angles[5]
    
    T=np.identity(4)
    
    # Add tamp
    if link<3:
        num_A=link
    else:
        num_A=link+1
           
    for j in range(num_A):
        A = np.array([[np.cos(theta[j]), -np.sin(theta[j])*np.cos(alpha[j]), np.sin(theta[j])*np.sin(alpha[j]), a[j]*np.cos(theta[j])],
                      [np.sin(theta[j]), np.cos(theta[j])*np.cos(alpha[j]), -np.cos(theta[j])*np.sin(alpha[j]), a[j]*np.sin(theta[j])],
                      [0, np.sin(alpha[j]), np.cos(alpha[j]), d[j]],
                      [0, 0, 0, 1] ])
        T=T.dot(A)

    return T
        
def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    w=np.zeros((6,3))
    v=np.zeros((6,3))
    w[0,:]=[0,0,1]
    w[1,:]=[0,1,0]
    w[2,:]=[0,1,0]
    w[3,:]=[0,0,1]
    w[4,:]=[0,1,0]
    w[5,:]=[0,0,1]
    v[1,:]=[-L1,0,0]
    v[2,:]=[-L1-L2,0,0]
    v[3,:]=[0,0,0]
    v[4,:]=[-L1-L2-L3,0,0]

    M=[[1,0,0,0],[0,1,0,0],[0,0,1,L1+L2+L3+L4],[0,0,0,1]]
    T=M
    for i in range(6):
        s=to_s_matrix(w[i,:],v[i,:])
        w_hat=np.delete(s,(3),axis=0)
        w_hat=np.delete(w_hat,(3),axis=1)
        ew=np.identity(3)+w_hat*math.sin(joint_angles[i])+np.power(w_hat,2)*(1-math.cos(joint_angles[i]))
        t=np.dot((np.identity(3)-ew),(np.cross(np.transpose(w[i,:]),(np.transpose(v[i,:])))))+\
          np.dot(np.dot(np.transpose(w),w),np.transpose(v[i,:]))*joint_angles[i]
        t=np.reshape(t,(3,1))
        es=np.append(ew,t,axis=1)
        e=np.reshape([0,0,0,1],(1,4))
        es=np.append(es,e,axis=0)
        T=np.dot(es,T)
    
    return T
    
def IK(pose,block_angle):
    theta=np.zeros(6)
    '''R='''
    R=np.zeros([3,3])
    if block_angle<-np.pi/4:
        block_angle=block_angle+np.pi/4
    c_b = np.cos(-block_angle)
    s_b = np.sin(-block_angle)
    if np.sqrt(pose[0]**2+pose[1]**2)<=219 and pose[2]<=100:
        L_4=L4
        R = [[c_b,s_b,0],[s_b,-c_b,0],[0,0,-1]]
    else:
        pose[2] = pose[2]-19
        L_4=L4+19
        if pose[0]>=0:
            R= [[0,s_b,c_b],[0,-c_b,s_b],[1,0,0]]
        else:
            R= [[0,-s_b,-c_b],[0,c_b,-s_b],[1,0,0]]


    o_end = [pose[0],pose[1],pose[2]]
    o_c = o_end-L_4*np.matmul(R,np.array([0,0,1]))
    theta[0] = np.arctan2(o_c[1],o_c[0])
    r1 = np.sqrt(o_c[0]**2+o_c[1]**2)
    r2 = o_c[2]-L1
    r3 = np.sqrt(r1**2+r2**2)
    D = -(r3**2-L2**2-L3**2)/(2*L2*L3);
    if D<-1 or D>1:
        theta[2]=0
        theta[1]=np.arctan2(r2, r1)-np.pi/2
    else:
        theta1_t=np.arctan2(r2,r1)
        theta2_t=np.arccos((L2**2 + L3**2 - r3**2)/(2*L2*L3))
        
        theta[1] = np.arcsin(np.sin(theta2_t)/r3*L3)+theta1_t - np.pi/2
        theta[2] = theta2_t - np.pi
    
    joint_angles = [theta[0],theta[1],theta[2],0,0,0]
    T30 = FK_dh(joint_angles, 3)
    R30 = np.delete(T30,(3), axis=0)
    R30 = np.delete(R30,(3),axis=1)
    R63 = np.matmul(np.transpose(R30),R)

    r33 = R63[2][2]
    r13 = R63[0][2]
    r23 = R63[1][2]
    r31 = R63[2][0]
    r32 = R63[2][1]
    r21 = R63[1][0]
    r12 = R63[0][1]
    r11 = R63[0][0]
    theta[4] = np.arctan2(-np.sqrt(1-r33**2),r33)
    if abs(np.sin(theta[4]))>0.05:
        theta[3] = np.arctan2(-r23,-r13)
        theta[5] = np.arctan2(-r32,r31)
    else:
        theta[3] = 0
        if np.cos(theta[4])>=0.95:
            theta[5] = np.arctan2(r21,r11)
        else:
            theta[5] = -np.arctan2(-r12,-r11)
    return theta

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pose1 = np.matmul(T, np.array([0,0,0,1]))
    pose = (pose1[0],pose1[1],pose1[2])

    return pose

def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    s=np.zeros((4,4))
    s[0,1]=-w[2]
    s[0,2]=w[1]
    s[1,0]=w[2]
    s[1,2]=-w[0]
    s[2,0]=-w[1]
    s[2,1]=w[0]
    s[0,3]=v[0]
    s[1,3]=v[1]
    s[2,3]=v[2]
    return s