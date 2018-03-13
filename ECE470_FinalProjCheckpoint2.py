import vrep
import time
import numpy as np
import scipy as sp


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

def deriveFK(clientID, Larm_theta, Rarm_theta, body_theta, Larm_joints, Rarm_joints, body_joints):
    #convert each theta into radians
    Larm_thetarad = []
    Rarm_thetarad = []
    body_thetarad = []

    if Larm_theta != []:
        for theta in Larm_theta:
            theta = int(theta) * (np.pi/180)
            Larm_thetarad.append(theta)
    if Rarm_theta != []:
        for theta in Rarm_theta:
            theta = int(theta) * (np.pi/180)
            Rarm_thetarad.append(theta)

    if body_theta != []:
        for theta in body_theta:
            theta = int(theta) * (np.pi/180)
            body_thetarad.append(theta)

    dummy_handler = []
    dummyres = []

    #declare handler parent
    res, base_handler = vrep.simxGetObjectHandle(clientID, 'Baxter_base_visible', vrep.simx_opmode_blocking)
    res, Larm_handler = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_tip', vrep.simx_opmode_blocking)
    res, Rarm_handler = vrep.simxGetObjectHandle(clientID, 'BaxterGripper', vrep.simx_opmode_blocking)
    res, monitor_handler = vrep.simxGetObjectHandle(clientID, 'Baxter_monitor', vrep.simx_opmode_blocking)


    #Display Coordinate frame via dummy
    res, dummy_monitor = vrep.simxCreateDummy(clientID, 0.1,[],vrep.simx_opmode_blocking)
    dummyres.append(res)
    dummy_handler.append(dummy_monitor)
    res2, dummy_Rarm = vrep.simxCreateDummy(clientID, 0.1,[],vrep.simx_opmode_blocking)
    dummyres.append(res2)
    dummy_handler.append(dummy_Rarm)
    res3, dummy_Larm = vrep.simxCreateDummy(clientID, 0.1,[],vrep.simx_opmode_blocking)
    dummyres.append(res3)
    dummy_handler.append(dummy_Larm)
    res4, dummy_base = vrep.simxCreateDummy(clientID, 0.25,[],vrep.simx_opmode_blocking)
    #Will clean up later (place into loops)

    positions = []
    orientations = []

    result, posB = vrep.simxGetObjectPosition(clientID, base_handler, base_handler, vrep.simx_opmode_blocking)
    result, orientationB = vrep.simxGetObjectOrientation(clientID, base_handler, base_handler, vrep.simx_opmode_blocking)

    result, posL = vrep.simxGetObjectPosition(clientID, Larm_handler, base_handler, vrep.simx_opmode_blocking)
    result, orientationL = vrep.simxGetObjectOrientation(clientID, Larm_handler, base_handler, vrep.simx_opmode_blocking)
    positions.append(posL)
    orientations.append(orientationL)

    result, posR = vrep.simxGetObjectPosition(clientID, Rarm_handler, base_handler, vrep.simx_opmode_blocking)
    result, orientationR = vrep.simxGetObjectOrientation(clientID, Rarm_handler, base_handler, vrep.simx_opmode_blocking)
    positions.append(posR)
    orientations.append(orientationR)

    result, posM = vrep.simxGetObjectPosition(clientID, monitor_handler, base_handler, vrep.simx_opmode_blocking)
    result, orientationM = vrep.simxGetObjectOrientation(clientID, monitor_handler, base_handler, vrep.simx_opmode_blocking)
    positions.append(posM)
    orientations.append(orientationM)

    #Set Original Tool Frame positions
    vrep.simxSetObjectPosition(clientID, dummy_base, base_handler, posB,vrep.simx_opmode_blocking)
    vrep.simxSetObjectOrientation(clientID, dummy_base, base_handler, orientationB, vrep.simx_opmode_blocking)

    vrep.simxSetObjectPosition(clientID, dummy_Larm, base_handler, posL,vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, dummy_Rarm, base_handler, posR, vrep.simx_opmode_blocking)
    vrep.simxSetObjectPosition(clientID, dummy_monitor, base_handler, posM, vrep.simx_opmode_blocking)

    vrep.simxSetObjectOrientation(clientID, dummy_Larm, base_handler, orientationL, vrep.simx_opmode_blocking)
    vrep.simxSetObjectOrientation(clientID, dummy_Rarm, base_handler, orientationR, vrep.simx_opmode_blocking)
    vrep.simxSetObjectOrientation(clientID, dummy_monitor, base_handler, orientationM, vrep.simx_opmode_blocking)

    #T_matrixlist = obtain_homogeneousTmatrix(clientID, positions, orientations)

    for result in dummyres:
        if result != vrep.simx_return_ok:
            raise Exception('Could not get Dummy')

    return dummy_handler, Larm_thetarad, Rarm_thetarad, body_thetarad

def obtain_homogeneousTmatrix(clientID, positions, orientations):
    #declare new fram positions

    T_matrix_list = []
    rot_rows = [0] * 4
    for i in range(0, 3):
        T_matrix = []
        for k in range(0, 4):
            T_matrix.append(rot_rows)
        T_matrix = populateTmatrix(T_matrix, positions, orientations)

    return T_matrix_list

def populateTmatrix(empty_Tmatrix, positions, orientations):

    for position in positions:
        print '22222', position
        for coordinate in position:
            for row, i in zip(empty_Tmatrix, [0, 1, 2, 3]):
                row[3] = position[i]

    empty_Tmatrix.append([0, 0, 0, 1])
    print empty_Tmatrix
    return empty_Tmatrix

def runsimulation(clientID, joint_library, Larm_joints, Rarm_joints, body_joints, Larm_theta, Rarm_theta, body_theta, joint_Rarm, joint_Larm, dummy_handler, joint_bodynames):
    # Start simulation
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

    # Wait for simulation to settle down
    time.sleep(2)

    # Get the current value of the first joint variable
    for joint in joint_library:
        joint_handle = joint_library[joint]['Joint Handler']
        result, theta = vrep.simxGetJointPosition(clientID, joint_handle, vrep.simx_opmode_blocking)

        if result != vrep.simx_return_ok:
            raise Exception('could not get first joint variable')
        #print 'current value of ' + joint + ': theta = {:f}'.format(theta)

    if body_theta != []:
        for joint, theta in zip(joint_bodynames, body_theta):
            joint_obj = body_joints[joint]['Joint Handler']
            vrep.simxSetJointTargetPosition(clientID, joint_obj, theta, vrep.simx_opmode_oneshot)
            time.sleep(2)

    if Rarm_theta != []:
        for joint, theta in zip(joint_Rarm, Rarm_theta):
            joint_obj = Rarm_joints[joint]['Joint Handler']
            vrep.simxSetJointTargetPosition(clientID, joint_obj, theta, vrep.simx_opmode_oneshot)
            time.sleep(2)

    if Larm_theta != []:
        for joint, theta in zip(joint_Larm, Larm_theta):
            joint_obj = Larm_joints[joint]['Joint Handler']
            vrep.simxSetJointTargetPosition(clientID, joint_obj, theta, vrep.simx_opmode_oneshot)
            time.sleep(2)

    time.sleep(5)
    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)

def main():
    Larm_theta = []
    Rarm_theta = []
    body_theta = []
    body_flag = False
    Larm_flag = False
    Rarm_flag = False

    print "This is a simulation for the Baxter robot"
    body_response = raw_input("Would you like to move the body? (Y or N)")

    if body_response == 'Y':
        body_flag = True
        print "Input Theta in degrees"
        theta_vert = raw_input("Vert. Joint = ")
        body_theta.append(float(theta_vert))
        theta_body_rotation = raw_input("Body rot. Joint = ")
        body_theta.append(theta_body_rotation)
        monitor_theta = raw_input("Monitor Theta = ")
        body_theta.append(monitor_theta)

    arm_response1 = raw_input("Would you like to move the left arm? (Y or N)")

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

    if body_flag or Rarm_flag or Larm_flag != False:
        clientID = initialize_sim()
        joint_library, Larm_joints, Rarm_joints, body_joints, joint_bodynames, joint_Rarm, joint_Larm = declarejointvar(clientID)
        dummy_handler, Larm_thetarad, Rarm_thetarad, body_thetarad = deriveFK(clientID, Larm_theta, Rarm_theta, body_theta, Larm_joints, Rarm_joints, body_joints)
        runsimulation(clientID, joint_library, Larm_joints, Rarm_joints, body_joints, Larm_thetarad, Rarm_thetarad, body_thetarad, joint_Rarm, joint_Larm, dummy_handler, joint_bodynames)


if __name__ == "__main__": main()