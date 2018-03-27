import vrep
import math
import time
import numpy as np
import scipy as sp

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
    rot_axis = [i for i in val_list[0:3]]
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
                              [0, 0, -1, -0.5065],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    return t_original

def get_screwmatrix(frame_selection): #in the future I will be implementing this into a screw matrix library, again for simplicity...
    if frame_selection == 1: #monitorframe
        s_matrix = np.zeros((6, 3))
        prismatic_s = np.zeros(6)
        prismatic_s[0:3] = np.array([0,0,0])
        prismatic_s[3:] = np.array([0,0,1])
        s_matrix[:,0] = prismatic_s
        s_vals = [[0, 0, 1, 0.25, 0.475, 0.8777],[0, 0, 1, 0.3101, 0.4751, 1.6104]]
        for vals in s_vals:
            ind = s_vals.index(vals) + 1
            s_matrix[:,ind] = val2screw(vals)
    elif frame_selection == 2: #left arm frame
        s_matrix = np.zeros((6, 7))
        s_vals = [[0, 0, 1, 0.3138, 0.7341, 1.054], [1, 0, 0, 0.3626, 0.7828, 1.3244], [0, 1, 0, 0.4347, 0.855, 1.3244], [1, 0, 0, 0.6203, 1.0405, 1.2554], [0, 1, 0, 0.6935, 1.1138, 1.2554], [1, 0, 0, 0.8849, 1.3052, 1.2454], [0, 1, 0, 0.9669, 1.3872, 1.2454]]
        for vals in s_vals:
            ind = s_vals.index(vals)
            s_matrix[:,ind] = val2screw(vals)
    elif frame_selection == 3: #right arm frame
        s_matrix = np.zeros((6, 7))
        s_vals = [[0, 0, 1, 0.3142, 0.216, 1.054], [-1, 0, 0, 0.3629, 0.1672, 1.3244], [0, -1, 0, 0.4351, 0.0951, 1.3244], [-1, 0, 0, 0.6206, -0.0904, 1.2554], [0, -1, 0, 0.6939, -0.1637, 1.2554], [-1, 0, 0, 0.8853, -0.3551, 1.2454], [0, -1, 0, 0.9673, -0.4371, 1.2454]]
        for vals in s_vals:
            ind = s_vals.index(vals)
            s_matrix[:, ind] = val2screw(vals)
    return s_matrix

def get_desiredposematrix(param_list):
    t_matrix = np.zeros((4,4))
    x = param_list[0]
    y = param_list[1]
    z = param_list[2]
    alpha = param_list[3]
    beta = param_list[4]
    gamma = param_list[5]

    t_matrix[0, 3] = x
    t_matrix[1, 3] = y
    t_matrix[2, 3] = z
    t_matrix[3, 3] = 1

    #Use generalized rotation matrix formula
    R_x = np.array([[1,               0,                    0],
                    [0, math.cos(alpha), -1 * math.sin(alpha)],
                    [0, math.sin(alpha), math.cos(alpha)]])

    R_y = np.array([[math.cos(beta), 0, math.sin(beta)],
                    [0,              1,              0],
                    [-1 * math.sin(beta), 0, math.cos(beta)]])

    R_z = np.array([[math.cos(gamma), -1 * math.sin(gamma), 0],
                    [math.sin(gamma),      math.cos(gamma),              0],
                    [0,                                  0,              1]])

    R_matrix = np.linalg.multi_dot([R_x, R_y, R_z])

    t_matrix[0:3,0:3] = R_matrix

    return t_matrix

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

    final_params = []
    print "Please input the desired position and orientation of your frame"
    x = float(raw_input("X-position: "))
    final_params.append(x)
    y = float(raw_input("Y-position: "))
    final_params.append(y)
    z = float(raw_input("Z-position: "))
    final_params.append(z)
    alpha = float(raw_input("X-orientation (deg): "))
    alpha = alpha * (math.pi/180) #convert to radians
    final_params.append(alpha)
    beta = float(raw_input("Y-orientation (deg): "))
    beta = beta * (math.pi/180)
    final_params.append(beta)
    gamma = float(raw_input("Z-orientation (deg): "))
    gamma = gamma * (math.pi/180)
    final_params.append(gamma)

    desired_T = get_desiredposematrix(final_params)
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