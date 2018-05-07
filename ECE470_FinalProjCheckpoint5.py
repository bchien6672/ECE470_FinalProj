import vrep
import math
import time
import numpy as np
import scipy.linalg as la


""" Node Class for Path Planning"""
class Node:
    def __init__(self, theta, parent):
        self.theta = theta
        self.parent = parent

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

""" Collision Detection Package and Path Planning """

def getCollisionlib(clientID):
     collision_list = ['CL', 'CR']

     collision_lib = {}

     for item in collision_list:
         res, collision = vrep.simxGetCollisionHandle(clientID, item, vrep.simx_opmode_blocking)
         collision_lib[item] = collision

     return collision_lib

def perform_collisiondetect(clientID, collision):
    res, in_collision = vrep.simxReadCollision(clientID, collision, vrep.simx_opmode_streaming)
    print "Collision Status: " + str(in_collision)
    return in_collision

def movebody(clientID, joint_dict, joint_name, theta, collision):
    for joint, theta_val in zip(joint_name, theta):
        joint_obj = joint_dict[joint]['Joint Handler']
        res, theta_0 = vrep.simxGetJointPosition(clientID, joint_obj, vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID, joint_obj, theta_0 + theta_val, vrep.simx_opmode_oneshot)
        time.sleep(1)
        collision_state = perform_collisiondetect(clientID, collision)
        if collision_state == True:
            vrep.simxSetJointTargetPosition(clientID, joint_obj, 0, vrep.simx_opmode_oneshot)
            time.sleep(0.5)
    return

def movejoint(clientID, joint_dict, joint_name, theta, left_right, collision_lib):
    collision_bool = False

    joint_obj = joint_dict[joint_name]['Joint Handler']
    res, theta_init = vrep.simxGetJointPosition(clientID, joint_obj, vrep.simx_opmode_blocking)
    vrep.simxSetJointTargetPosition(clientID, joint_obj, theta_init + theta, vrep.simx_opmode_oneshot)
    time.sleep(1)

    if left_right == 'L':
        lr_key = 'CL'
    elif left_right == 'R':
        lr_key = 'CR'
    collision_handle = collision_lib[lr_key]
    collision_bool = perform_collisiondetect(clientID, collision_handle)

    return collision_bool

def findpath(clientID, joint_dict, joint_name, collision_lib, theta_goal, left_right):
    initial_theta = [0, 0, 0, 0, 0, 0, 0]
    joint_upperbound = [1.7016, 1.047, 3.0541, 2.618, 3.059, 2.094, 3.059]
    joint_lowerbound = [-1.7016, -2.147, -3.0541, -0.05, -3.059, -1.5707, -3.059]

    theta_goal_rad = []

    #convert to rad
    for theta in theta_goal:
        theta_rad = theta * (np.pi/180)
        theta_goal_rad.append(theta_rad)

    #initialize tree
    theta_0 = Node(initial_theta, None)
    theta_1 = Node(theta_goal_rad, None)

    forward = [theta_0]
    backward = [theta_1]

    iter = 0
    while iter <= 5:
        print "Iter: " + str(iter)
        theta_comp = np.zeros(7)
        theta_comp[0] = (joint_upperbound[0] - joint_lowerbound[0]) * np.random.random_sample() + joint_lowerbound[0]
        theta_comp[1] = (joint_upperbound[1] - joint_lowerbound[1]) * np.random.random_sample() + joint_lowerbound[1]
        theta_comp[2] = (joint_upperbound[2] - joint_lowerbound[2]) * np.random.random_sample() + joint_lowerbound[2]
        theta_comp[3] = (joint_upperbound[3] - joint_lowerbound[3]) * np.random.random_sample() + joint_lowerbound[3]
        theta_comp[4] = (joint_upperbound[4] - joint_lowerbound[4]) * np.random.random_sample() + joint_lowerbound[4]
        theta_comp[5] = (joint_upperbound[5] - joint_lowerbound[5]) * np.random.random_sample() + joint_lowerbound[5]
        theta_comp[6] = (joint_upperbound[6] - joint_lowerbound[6]) * np.random.random_sample() + joint_lowerbound[6]

        theta_comp = theta_comp.tolist()
        inForward = False
        inBackward = False

        collision_bool = False

        for forward_theta in forward:
            close_node = forward_theta

        for theta, joint in zip(theta_comp, joint_name):
            collision_bool = movejoint(clientID, joint_dict, joint, theta, left_right, collision_lib)
            if collision_bool == True:
                break

            if collision_bool == False:
                continue

        if collision_bool == True:
            iter += 1
            continue

        iter += 1

        #This is assuming collision_bool is True...
        theta_newpos = Node(theta_comp, close_node)
        forward.append(theta_comp)
        inForward = True

        for backward_theta in backward:
            close_node = backward_theta


        theta_back = Node(theta_comp, close_node)
        backward.append(theta_comp)
        inBackward = True

        if inBackward and inForward == True:
            theta_path = [theta_newpos.theta]
            parent = theta_newpos.parent
            while parent != None:
                theta_path = [parent.theta] + theta_path
                parent = parent.parent

            parent = theta_back.parent
            while parent != None:
                theta_path = theta_path + [parent.theta]
                parent = parent.parent

            return theta_path
    return False

def executemovement(clientID, joint_dict, joint_name, collision_lib, theta_goal, left_right):
    path = findpath(clientID, joint_dict, joint_name, collision_lib, theta_goal, left_right)

    if path == False:
        print "Valid path not found!"
    else:
        print "Path: " + str(path)

        for set in path:
            for angle in set:
                ind = set.index(angle)
                angle = angle * (180/np.pi)
                set[ind] = angle

        print "Path in deg: " + str(path)

    return

def main():
    Larm_theta = []
    Rarm_theta = []
    Larm_flag = False
    Rarm_flag = False

    print "This is a path planning simulation for the Baxter robot"

    arm_response1 = raw_input("Would you like to move the left arm? (Y or N)")

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