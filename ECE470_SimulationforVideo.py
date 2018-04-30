import vrep
import math
import time
import numpy as np
import scipy.linalg as la

""" Matrix Calculations """ #Syntax is similar to public class repository
def create_skewsym(input_matrix): #input_matrix must be 3 elements
    ss_matrix = np.zeros((3, 3))
    ss_matrix[0][1] = -1 * input_matrix[2]
    ss_matrix[0][2] = input_matrix[1]
    ss_matrix[1][0] = input_matrix[2]
    ss_matrix[1][2] = -1 * input_matrix[0]
    ss_matrix[2][0] = -1 * input_matrix[1]
    ss_matrix[2][1] = input_matrix[0]

    return ss_matrix

def get_skewval(matrix):
    ss_val = np.array([[matrix[2][1]], [matrix[0][2]], [matrix[1][0]]])
    return ss_val

def create_bracket(v_screw): #input should be 6 element list
    bracket_matrix = np.zeros((4,4))
    bracket_matrix[0:3, 0:3] = create_skewsym(v_screw[0:3])
    bracket_matrix[0:3, 3] = np.transpose(v_screw[3:])

    return bracket_matrix

def create_adjoint(t_matrix): #input is a 4x4 matrix
    rot, pos = create_rottransmatrix(t_matrix)
    pos = [pos[0] for pos in pos]
    ss_m = create_skewsym(pos)
    ss_m = np.multiply(-1, ss_m)
    adj_matrix = np.block([[ rot, np.zeros((3,3))],
                           [np.multiply(ss_m, rot), rot]])
    return adj_matrix

def create_posematrix(rot, pos):
    T = np.block([[rot, pos],
                 [[0, 0, 0, 1]]])
    return T

def create_rottransmatrix(t_matrix): #Input is a 4x4 matrix
    T = np.asarray(t_matrix)
    rot_matrix = T[:3, :3]
    trans_matrix = T[:3, 3:4]

    return rot_matrix, trans_matrix

def val2screw(val_list):
    rot_axis = [i for i in val_list[0:3]]
    screw_matrix = np.zeros(6)
    screw_matrix[0:3] = np.array([val_list[0], val_list[1], val_list[2]])
    screw_matrix[3:] = -1 * np.dot(create_skewsym(rot_axis), np.array([val_list[3], val_list[4], val_list[5]]))
    return screw_matrix

def mat2screw(input_matrix): #input is a 4x4 matrix
    V_array = [input_matrix[2][1], input_matrix[0][2], input_matrix[1][0], input_matrix[0][3], input_matrix[1][3], input_matrix[2][3]]
    return V_array

def rotMatrix2Euler(R_matrix): #find general euler angles from rot matrix
    x = math.atan2(-R_matrix[1, 2], R_matrix[2, 2])
    y = math.asin(R_matrix[0, 2])
    z = math.atan2(-R_matrix[0, 1], R_matrix[0,0])
    angles = np.array([x, y, z])
    return angles

""" Kinematics Equations"""
def get_initialpose():
    t_originallist = []
    #Baxter's left
    t_originallist.append(np.array([[-1, 0, 0, 0.1055],
                              [0, 0, -1, 1.3301],
                              [0, 1, 0, 1.25193],
                              [0, 0, 0, 1]]))
    #Baxter's right
    t_originallist.append(np.array([[0, 0, -1, 0.1059],
                              [1, 0, 0, -1.3196],
                              [0, -1, 0, 1.252953],
                              [0, 0, 0, 1]]))
    #Baxter's monitor
    t_originallist.append([[0, 1, 0, 0.21212],
                              [0, 0, 1, 0.0025],
                              [1, 0, 0, 1.582553],
                              [0, 0, 0, 1]])

    return t_originallist




""" V-REP Functions """
def initialize_sim():
    # Close all open connections (just in case)
    vrep.simxFinish(-1)
    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')

    return clientID


def getCollisionlib(clientID):
    collision_list = ['CL', 'CR']

    collision_lib = {}

    for item in collision_list:
        res, collision = vrep.simxGetCollisionHandle(clientID, item, vrep.simx_opmode_blocking)
        collision_lib[item] = collision

    return collision_lib


def checkjointThetas(theta_list):
    joint_upperbound = [97.494, 60, 174.987, 150, 175.25, 120, 175.25]
    joint_lowerbound = [-97.494, -123, -174.987, -2.864, -175.25, -90, -175.25]

    for i in range(0, len(theta_list) - 1):
        if joint_upperbound[i] > theta_list[i] > joint_lowerbound[i]:
            continue
        else:
            print "Theta " + theta_list[i] + " does not fit within the bounds: " + joint_lowerbound[i] + ", " + joint_upperbound[i]
            theta_list[i] = 0

    return theta_list

def declarejointvar(clientID):
    joint_library = {}
    body_joints = {}
    Rarm_joints = {}
    Larm_joints = {}

    joint_bodynames = ['Baxter_verticalJoint', 'Baxter_rotationJoint', 'Baxter_monitorJoint']
    joint_Rarm = ['Baxter_rightArm_joint1', 'Baxter_rightArm_joint2', 'Baxter_rightArm_joint3', 'Baxter_rightArm_joint4', 'Baxter_rightArm_joint5', 'Baxter_rightArm_joint6', 'Baxter_rightArm_joint7']
    joint_Larm = ['Baxter_leftArm_joint1', 'Baxter_leftArm_joint2', 'Baxter_leftArm_joint3', 'Baxter_leftArm_joint4', 'Baxter_leftArm_joint5', 'Baxter_leftArm_joint6', 'Baxter_leftArm_joint7']
    joint_names = ['Baxter_leftArm_joint1', 'Baxter_rightArm_joint1', 'Baxter_leftArm_joint2', 'Baxter_rightArm_joint2',
                   'Baxter_leftArm_joint3', 'Baxter_rightArm_joint3', 'Baxter_leftArm_joint4', 'Baxter_rightArm_joint4',
                   'Baxter_leftArm_joint5', 'Baxter_rightArm_joint5', 'Baxter_leftArm_joint6', 'Baxter_rightArm_joint6',
                   'Baxter_leftArm_joint7', 'Baxter_rightArm_joint7', 'Baxter_verticalJoint', 'Baxter_rotationJoint',
                   'Baxter_monitorJoint']

    for joint in joint_names:
        result, joint_handle = vrep.simxGetObjectHandle(clientID, joint, vrep.simx_opmode_blocking)
        if joint in joint_bodynames:
            body_joints[joint] = {}
            body_joints[joint]['Result'] = result
            body_joints[joint]['Joint Handler'] = joint_handle
        elif joint in joint_Rarm:
            Rarm_joints[joint] = {}
            Rarm_joints[joint]['Result'] = result
            Rarm_joints[joint]['Joint Handler'] = joint_handle
        elif joint in joint_Larm:
            Larm_joints[joint] = {}
            Larm_joints[joint]['Result'] = result
            Larm_joints[joint]['Joint Handler'] = joint_handle
        joint_library[joint] = {}
        joint_library[joint]['Result'] = result
        joint_library[joint]['Joint Handler'] = joint_handle

    return joint_library, Larm_joints, Rarm_joints, body_joints, joint_bodynames, joint_Rarm, joint_Larm


def main():
    Larm_theta = []
    Rarm_theta = []
    Larm_flag = False
    Rarm_flag = False

    print "This is a path planning simulation for the Baxter robot"

    #for user reference
    #joint_upperbound = [97.494, 60, 174.987, 150, 175.25, 120, 175.25]
    #joint_lowerbound = [-97.494, -123, -174.987, -2.864, -175.25, -90, -175.25]

    if arm_response1 == 'Y':
        Larm_flag = True
        print "Input Theta in degrees"
        theta1 = raw_input("Theta1 = ")
        Larm_theta.append(float(theta1))
        theta2 = raw_input("Theta2 = ")
        Larm_theta.append(float(theta2))
        theta3 = raw_input("Theta3 = ")
        Larm_theta.append(float(theta3))
        theta4 = raw_input("Theta4 = ")
        Larm_theta.append(float(theta4))
        theta5 = raw_input("Theta5 = ")
        Larm_theta.append(float(theta5))
        theta6 = raw_input("Theta6 = ")
        Larm_theta.append(float(theta6))
        theta7 = raw_input("Theta7 = ")
        Larm_theta.append(float(theta7))

    arm_response2 = raw_input("Would you like to move the right arm? (Y or N)")
    if arm_response2 == 'Y':
        Rarm_flag = True
        print "Input Theta in degrees"
        theta1 = raw_input("Theta1 = ")
        Rarm_theta.append(float(theta1))
        theta2 = raw_input("Theta2 = ")
        Rarm_theta.append(float(theta2))
        theta3 = raw_input("Theta3 = ")
        Rarm_theta.append(float(theta3))
        theta4 = raw_input("Theta4 = ")
        Rarm_theta.append(float(theta4))
        theta5 = raw_input("Theta5 = ")
        Rarm_theta.append(float(theta5))
        theta6 = raw_input("Theta6 = ")
        Rarm_theta.append(float(theta6))
        theta7 = raw_input("Theta7 = ")
        Rarm_theta.append(float(theta7))

    if Larm_flag == False:
        Larm_theta = [0, 0, 0, 0, 0, 0, 0]

    if Rarm_flag == False:
        Rarm_theta = [0, 0, 0, 0, 0, 0, 0]

    #check if thetas are valid
    Rarm_theta = checkjointThetas(Rarm_theta)
    Larm_theta = checkjointThetas(Larm_theta)

    clientID = initialize_sim()
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    joint_library, Larm_jointsdict, Rarm_jointsdict, body_jointsdict, joint_bodynames, joint_Rarm, joint_Larm = declarejointvar(clientID)
    collision_library = getCollisionlib(clientID)

    #generate path
    #Initial thetas are 0, desired goal thetas are user-input

    #path plan for L_joints
    executemovement(clientID, Larm_jointsdict, joint_Larm, collision_library, Larm_theta, 'L')

    #path plan for R_joints
    #executemovement(clientID, Rarm_jointsdict, joint_Rarm, collision_library, Rarm_theta, 'R')

    # stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)


if __name__ == "__main__": main()