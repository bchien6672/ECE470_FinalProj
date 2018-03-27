import vrep
import math
import time
import numpy as np
import scipy as sp

""" Matrix Calculations """ #Syntax is similar to public class repository
def create_skewsym(input_matrix): #input_matrix must be 3 elements
    ss_matrix = np.zeros(3, 3)
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
    bracket_matrix = np.zeros(4,4)
    bracket_matrix[0:3, 0:3] = create_skewsym(v_screw[0:3])
    bracket_matrix[0:3, 3] = np.transpose(v_screw[3:])

    return bracket_matrix

def create_adjoint(t_matrix): #input is a 4x4 matrix
    rot, pos = create_rottransmatrix(t_matrix)
    input = pos.tolist()
    ss_m = create_skewsym(input)
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
    rot_axis = [i for i in val_list[0:2]]
    screw_matrix = np.zeros(6)
    screw_matrix[0:3] = np.array([val_list[0], val_list[1], val_list[2]])
    screw_matrix[3:] = -1 * np.dot(create_skewsym(rot_axis), np.array([val_list[3], val_list[4], val_list[5]]))
    return screw_matrix

def rotMatrix2Euler(R_matrix): #find general euler angles from rot matrix
    x = math.atan2(-R_matrix[1, 2], R_matrix[2, 2])
    y = math.asin(R_matrix[0, 2])
    z = math.atan2(-R_matrix[0, 1], R_matrix[0,0])
    angles = np.array([x, y, z])
    return angles

""" Kinematics Equations"""
def get_initialpose(frame_selection):
    if frame_selection == 1: #monitor frame
        t_original = np.array([[0, 0, 1, 0.4113],
                              [1, 0, 0, 0.4751],
                              [0, 1, 0, 1.6004],
                              [0, 0, 0, 1]])
    elif frame_selection == 2: #left arm frame
        t_original = np.array([[1, 0, 0, 1.0363],
                              [0, 0, 1, 1.456],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    elif frame_selection == 3: #right arm frame
        t_original = np.array([[1, 0, 0, 1.0363],
                              [0, 0, 1, -0.5065],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    return t_original

def get_screwmatrix(frame_selection):
    if frame_selection == 1: #monitorframe
        s_matrix = np.zeros(6, 3)
        s_matrix = np.array([[0, 0, 1, 0.4113],
                              [1, 0, 0, 0.4751],
                              [0, 1, 0, 1.6004],
                              [0, 0, 0, 1]])
    elif frame_selection == 2: #left arm frame
        s_matrix = np.zeros(6, 7)
        s_matrix = np.array([[1, 0, 0, 1.0363],
                              [0, 0, 1, 1.456],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    elif frame_selection == 3: #right arm frame
        s_matrix = np.zeros(6, 7)
        s_matrix = np.array([[1, 0, 0, 1.0363],
                              [0, 0, 1, -0.5065],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    return s_matrix

""" V-REP Functions """
def initialize_sim():
    # Close all open connections (just in case)
    vrep.simxFinish(-1)
    # Connect to V-REP (raise exception on failure)
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID == -1:
        raise Exception('Failed connecting to remote API server')

    return clientID

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
        #compose Master joint dictionary.....
        joint_library[joint] = {}
        joint_library[joint]['Result'] = result
        joint_library[joint]['Joint Handler'] = joint_handle

    return joint_library, Larm_joints, Rarm_joints, body_joints, joint_bodynames, joint_Rarm, joint_Larm

def movebody():
    return

def move_rightarm():
    return

def move_leftarm():
    return

def main():
    Larm_theta = []
    Rarm_theta = []
    body_theta = []
    body_flag = False
    Larm_flag = False
    Rarm_flag = False
    frame_selection = 0

    print "This is an inverse kinematics simulation for the Baxter robot. As of now, only one frame can be calculated."
    body_response = raw_input("Would you like to move the body, left arm, or right arm:  ")

    if str(body_response) == "body": #3 joints
        body_flag = True
        frame_selection = 1
        print "The body will be moved"
    elif str(body_response) == "left arm": #7 joints
        Larm_flag = True
        frame_selection = 2
        print "The left arm will be moved"
    elif str(body_response) == "right arm": #7 joints
        Rarm_flag = True
        frame_selection = 3
        print "The right arm will be moved"
    else:
        print "That is not a valid frame to be moved"

    if body_flag and Larm_flag and Rarm_flag == False:
        print "Program end"
        quit()

    #run inverse/forward kinematics calculations
    M = get_initialpose(frame_selection)
    S = get_screwmatrix(frame_selection)


    clientID = initialize_sim()

    #declare joint parameters for interacting with V-REP
    joint_library, Larm_joints, Rarm_joints, body_joints, joint_bodynames, joint_Rarm, joint_Larm = declarejointvar(clientID)

    #stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)

if __name__ == "__main__": main()