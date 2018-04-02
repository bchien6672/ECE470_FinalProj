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
def get_initialpose(frame_selection):
    if frame_selection == 1: #monitor frame
        t_original = np.array([[1, 0, 0, 0.4113],
                              [0, 1, 0, 0.4751],
                              [0, 0, 1, 1.6004],
                              [0, 0, 0, 1]])
    elif frame_selection == 2: #left arm frame
        t_original = np.array([[-1, 0, 0, 1.0363],
                              [0, 0, 1, 1.456],
                              [0, 1, 0, 1.237],
                              [0, 0, 0, 1]])
    elif frame_selection == 3: #right arm frame
        t_original = np.array([[0, 0, 1, 1.0363],
                              [1, 0, 0, -0.5065],
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
        s_vals = [[0, 0, 1, 0.3138, 0.7341, 1.054], [-1, 0, 0, 0.3626, 0.7828, 1.3244], [0, 1, 0, 0.4347, 0.855, 1.3244], [-1, 0, 0, 0.6203, 1.0405, 1.2554], [0, 1, 0, 0.6935, 1.1138, 1.2554], [-1, 0, 0, 0.8849, 1.3052, 1.2454], [0, 1, 0, 0.9669, 1.3872, 1.2454]]
        for vals in s_vals:
            ind = s_vals.index(vals)
            s_matrix[:,ind] = val2screw(vals)
    elif frame_selection == 3: #right arm frame
        s_matrix = np.zeros((6, 7))
        s_vals = [[0, 0, 1, 0.3142, 0.216, 1.054], [0, 1, 0, 0.3629, 0.1672, 1.3244], [1, 0, 0, 0.4351, 0.0951, 1.3244], [0, 1, 0, 0.6206, -0.0904, 1.2554], [1, 0, 0, 0.6939, -0.1637, 1.2554], [0, 1, 0, 0.8853, -0.3551, 1.2454], [1, 0, 0, 0.9673, -0.4371, 1.2454]]
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

def get_Jacobian(S_explist, S_list):
    S_ascending = []
    adj_list = []
    J = []
    num_adj = len(S_explist) - 2

    S_val = 1
    iter = 0
    for item in S_explist:
        if iter <= num_adj:
            S_val = np.multiply(S_val, item)
            S_ascending.append(S_val)
            iter += 1

    for item in S_ascending:
        adj_matrix = create_adjoint(item)
        adj_list.append(adj_matrix)

    for i in range(0, len(adj_list) + 1):
        if i == 0:
            J.append(S_list[:,0])
        else:
            S_f = np.dot(adj_list[i - 1], S_list[:,i])
            J.append(S_f)

    J = np.asarray(J)

    return J

def perform_FK(S_list, theta_list, M):
    S_explist = []
    for item, theta in zip(S_list, theta_list):
        S_mul = np.dot(item, theta)
        S_exp = la.expm(S_mul)
        S_explist.append(S_exp)

    S_whole = np.linalg.multi_dot(S_explist)
    final_pose = np.dot(S_whole, M)

    return S_explist, final_pose

def perform_IK(T_1, M, S, dof, frame_selection, Larm_joints, Rarm_joints, body_joints, joint_bodynames, joint_Rarm, joint_Larm, clientID): #syntax follows homework syntax
    theta = np.random.rand(dof, 1).tolist()
    theta = [theta[0] for theta in theta]
    samples = 10
    V_threshold = 0.1
    theta_threshold = 0.01
    theta_err = 10

    S_bracketlist = []

    rot, pos = create_rottransmatrix(T_1)
    rot_angles = rotMatrix2Euler(rot)
    rot_angles = [rot_angles[0] for angle in rot_angles]

    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    # Wait for simulation to settle down
    time.sleep(2)

    res, dummy = vrep.simxCreateDummy(clientID, 0.25, [], vrep.simx_opmode_blocking)
    res, base_handler = vrep.simxGetObjectHandle(clientID, 'Baxter_base_visible', vrep.simx_opmode_blocking)

    vrep.simxSetObjectPosition(clientID, dummy, base_handler, pos, vrep.simx_opmode_blocking)
    vrep.simxSetObjectOrientation(clientID, dummy, base_handler, rot_angles, vrep.simx_opmode_blocking)
    time.sleep(2)

    for vals in S.T:
        s_bracket = create_bracket(vals)
        S_bracketlist.append(s_bracket)

    for i in range(1, samples):
        if frame_selection == 1:
            movebody(clientID, body_joints, joint_bodynames, theta)
        elif frame_selection == 2:
            move_leftarm(clientID, Larm_joints, joint_Larm, theta)
        elif frame_selection == 3:
            move_rightarm(clientID, Rarm_joints, joint_Rarm, theta)

        S_explist, T_2 = perform_FK(S_bracketlist, theta, M)

        combined_T = np.dot(T_2, np.linalg.inv(T_1))
        V = la.logm(combined_T)
        V_screw = mat2screw(V)
        V_screw = np.asarray(V_screw)
        V_err = la.norm(V_screw)

        if V_err < V_threshold or theta_err < theta_threshold:
            rot, pos = create_rottransmatrix(T_2)
            rot_angles = rotMatrix2Euler(rot)
            rot_angles = [rot_angles[0] for angle in rot_angles]
            res, dummy2 = vrep.simxCreateDummy(clientID, 0.25, [0, 255, 0], vrep.simx_opmode_blocking)
            res, base_handler = vrep.simxGetObjectHandle(clientID, 'Baxter_base_visible', vrep.simx_opmode_blocking)

            vrep.simxSetObjectPosition(clientID, dummy2, base_handler, pos, vrep.simx_opmode_blocking)
            vrep.simxSetObjectOrientation(clientID, dummy2, base_handler, rot_angles, vrep.simx_opmode_blocking)
            return theta

        J = get_Jacobian(S_explist, S)
        theta_dot = np.dot(la.pinv(J.T), V_screw)
        theta_err = np.linalg.norm(theta_dot)
        theta = theta - (theta_dot)

    print "Simulation did not converge"

    return theta

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

def movebody(clientID, joint_dict, joint_name, theta):
    for joint, theta_val in zip(joint_name, theta):
        joint_obj = joint_dict[joint]['Joint Handler']
        vrep.simxSetJointTargetPosition(clientID, joint_obj, theta_val, vrep.simx_opmode_oneshot)
        time.sleep(0.5)
    return

def move_rightarm(clientID, joint_dict, joint_name, theta):
    for joint, theta_val in zip(joint_name, theta):
        joint_obj = joint_dict[joint]['Joint Handler']
        vrep.simxSetJointTargetPosition(clientID, joint_obj, theta_val, vrep.simx_opmode_oneshot)
        time.sleep(0.5)
    return

def move_leftarm(clientID, joint_dict, joint_name, theta):
    for joint, theta_val in zip(joint_name, theta):
        joint_obj = joint_dict[joint]['Joint Handler']
        vrep.simxSetJointTargetPosition(clientID, joint_obj, theta_val, vrep.simx_opmode_oneshot)
        time.sleep(0.5)
    return