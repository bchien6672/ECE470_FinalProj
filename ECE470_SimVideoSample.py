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
    t_originallist.append(np.array([[-1, 0, 0, 0.7384],
                              [0, 0, -1, 0.446],
                              [0, 1, 0, 1.665853],
                              [0, 0, 0, 1]]))
    #Baxter's right
    t_originallist.append(np.array([[0, 0, -1, 0.8301],
                              [1, 0, 0, -0.6108],
                              [0, -1, 0, 0.745753],
                              [0, 0, 0, 1]]))
    #Baxter's monitor
    t_originallist.append(np.array([[0, 1, 0, 0.21212],
                              [0, 0, 1, 0.0025],
                              [1, 0, 0, 1.582553],
                              [0, 0, 0, 1]]))

    return t_originallist

def get_screwmatrix():
    s_matrixlist = []
    #Baxter's left
    s_matrix = np.zeros((6, 7))
    s_vals = [[0, 0, 1, 0.1055, 0.2651, 1.053953], [-1, 0, 0, 0.1499, 0.3143, 1.324], [0, 1, 0, 0.2093, 0.3851, 1.367453],
              [-1, 0, 0, 0.3337, 0.6157, 1.438153], [0, 1, 0, 0.4074, 0.6241, 1.510453], [-1, 0, 0, 0.5959, 0.6548, 1.702553],
              [0, 1, 0, 0.6626, 0.561, 1.688553]]
    for vals in s_vals:
        ind = s_vals.index(vals)
        s_matrix[:, ind] = val2screw(vals)

    s_matrixlist.append(s_matrix)

    #Baxter's right
    s_matrix = np.zeros((6, 7))
    s_vals = [[0, 0, 1, 0.1059, -0.2566, 1.053953], [1, 0, 0, 0.1656, -0.2911, 1.324353], [0, -1, 0, 0.2281, -0.3271, 1.252153],
              [1, 0, 0, 0.339, -0.4118, 1.019453], [0, -1, 0, 0.4087, -0.4494, 0.952653],
              [1, 0, 0, 0.5838, -0.5482, 0.771153], [0, -1, 0, 0.695, -0.5776, 0.756353]]
    for vals in s_vals:
        ind = s_vals.index(vals)
        s_matrix[:, ind] = val2screw(vals)

    s_matrixlist.append(s_matrix)

    return s_matrixlist

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
   # print theta_list
    S_whole = np.linalg.multi_dot(S_explist)
    final_pose = np.dot(S_whole, M)

    return S_explist, final_pose

def perform_IK(T_1, M, S, dof): #syntax follows homework syntax
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

    for vals in S.T:
        s_bracket = create_bracket(vals)
        S_bracketlist.append(s_bracket)

    for i in range(1, samples):
        S_explist, T_2 = perform_FK(S_bracketlist, theta, M)
        combined_T = np.dot(T_2, np.linalg.inv(T_1))
        V = la.logm(combined_T)
        V_screw = mat2screw(V)
        V_screw = np.asarray(V_screw)
        V_err = la.norm(V_screw)

        if V_err < V_threshold or theta_err < theta_threshold:
            print theta
            return theta

        J = get_Jacobian(S_explist, S)
        theta_dot = np.dot(la.pinv(J.T), V_screw)
        theta_err = np.linalg.norm(theta_dot)
        theta = theta - (theta_dot)

    print "Simulation did not converge"

    return theta

""" Video Functions """
def move_arm(clientID, joint_dict, joint_list, collision_set, arm):
    #get joint initial position

    init_theta = []
    for joint in joint_list:
        joint_handle = joint_dict[joint]['Joint Handler']
        res, theta = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)
        init_theta.append(theta)

    T_set = get_initialpose()
    S_list = get_screwmatrix()

    if arm == "left":
        M = T_set[0]
        S = S_list[0]
    elif arm == "right":
        M = T_set[1]
        S = S_list[0]

    if arm == "left":
        res, object = vrep.simxGetObjectHandle(clientID, "Cup", vrep.simx_opmode_blocking)
        res, parent = vrep.simxGetObjectParent(clientID, object, vrep.simx_opmode_blocking)
    elif arm == "right":
        res, object = vrep.simxGetObjectHandle(clientID, "Cup0", vrep.simx_opmode_blocking)
        res, parent = vrep.simxGetObjectParent(clientID, object, vrep.simx_opmode_blocking)

    res, pos = vrep.simxGetObjectPosition(clientID, object, parent, vrep.simx_opmode_blocking)

    desired_T = np.array([[1, 0, 0, pos[0]],
                              [0, 1, 0, pos[1]],
                              [0, 0, 1, pos[2]],
                              [0, 0, 0, 1]])

    perform_IK(desired_T, M, S, 7)

    return

def grab_cup(clientID):
    return

def drop_cup(clientID):
    return

def final_move(clientID, jointsdict, joint_list, collision_library, arm):
    if arm == "left":
        handle = jointsdict['Baxter_leftArm_joint1']['Joint Handler']
        res, theta_i = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    elif arm == "right":
        handle = jointsdict['Baxter_rightArm_joint1']['Joint Handler']
        res, theta_i = vrep.simxGetJointPosition(clientID, handle, vrep.simx_opmode_blocking)
    return

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
    print "Brian H Chien, ME445/ECE470 Final Project"

    #for user reference
    #joint_upperbound = [97.494, 60, 174.987, 150, 175.25, 120, 175.25]
    #joint_lowerbound = [-97.494, -123, -174.987, -2.864, -175.25, -90, -175.25]

    clientID = initialize_sim()
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    joint_library, Larm_jointsdict, Rarm_jointsdict, body_jointsdict, joint_bodynames, joint_Rarm, joint_Larm = declarejointvar(clientID)
    collision_library = getCollisionlib(clientID)

    #generate path
    #Initial thetas are 0, desired goal thetas are user-input

    left_cup = np.array([[1, 0, 0, 0.8416],
                              [0, 1, 0, 0.6025],
                              [0, 0, 1, 1.013553],
                              [0, 0, 0, 1]])

    right_cup = np.array([[1, 0, 0, 1.075],
                              [0, 1, 0, 0.05],
                              [0, 0, 1, 1.0117],
                              [0, 0, 0, 1]])

    vrep.simxSetIntegerSignal(clientID, "BaxterGripperL_close", 1, vrep.simx_opmode_oneshot)
    time.sleep(40)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_leftArm_joint5']['Joint Handler'], vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_leftArm_joint5']['Joint Handler'], theta - np.pi, vrep.simx_opmode_oneshot)
    time.sleep(5)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_leftArm_joint1']['Joint Handler'], vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_leftArm_joint1']['Joint Handler'], theta + ((np.pi/2) - 0.35),vrep.simx_opmode_oneshot)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_leftArm_joint2']['Joint Handler'],
                                           vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_leftArm_joint2']['Joint Handler'],
                                    theta - (np.pi / 8), vrep.simx_opmode_oneshot)

    time.sleep(3)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_leftArm_joint5']['Joint Handler'], vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_leftArm_joint5']['Joint Handler'],theta + (np.pi), vrep.simx_opmode_oneshot)


    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_rightArm_joint5']['Joint Handler'], vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_rightArm_joint5']['Joint Handler'],theta - (np.pi/2), vrep.simx_opmode_oneshot)

    time.sleep(3)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_rightArm_joint1']['Joint Handler'],vrep.simx_opmode_blocking)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_rightArm_joint1']['Joint Handler'],theta + (np.pi / 6), vrep.simx_opmode_oneshot)

    res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_rightArm_joint2']['Joint Handler'],
                                           vrep.simx_opmode_blocking)

    time.sleep(3)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_rightArm_joint2']['Joint Handler'],
                                    theta + (np.pi / 4), vrep.simx_opmode_oneshot)

    vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_rightArm_joint5']['Joint Handler'],
                                    theta + (np.pi / 2), vrep.simx_opmode_oneshot)

    #res, theta = vrep.simxGetJointPosition(clientID, joint_library['Baxter_leftArm_joint4']['Joint Handler'], vrep.simx_opmode_blocking)

    #vrep.simxSetJointTargetPosition(clientID, joint_library['Baxter_leftArm_joint4']['Joint Handler'], theta + (np.pi/3), vrep.simx_opmode_oneshot)

    #move left arm to position
    #move_arm(clientID, Larm_jointsdict, joint_Larm, collision_library, "left")

    #grab_cup(clientID, "left")

    #move_arm(clientID, Larm_jointsdict, joint_Larm, collision_library, "left")

    #drop_cup(clientID, "left")

    #move_arm(clientID, Larm_jointsdict, joint_Larm, collision_library, "left")

    #final_move(clientID, Larm_jointsdict, joint_Larm, collision_library, "right")

    #grab_cup(clientID, "right")

    #move_arm(clientID, Larm_jointsdict, joint_Larm, collision_library, "right")

    #drop_cup(clientID, "right")

    #final_move(clientID, Larm_jointsdict, joint_Larm, collision_library, "right")

    #stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)


if __name__ == "__main__": main()