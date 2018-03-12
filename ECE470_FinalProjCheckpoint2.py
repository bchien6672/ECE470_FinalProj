import vrep
import time
import numpy as np



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

    print body_joints, Rarm_joints, Larm_joints

    return joint_library

    #print '111111111111', joint_library
    #for joint in joint_library.keys():
        #item = joint_library[joint]['Result']
        #if item != vrep.simx_return_ok:
            #raise Exception('Cannot get object handle of one of the joints. Check again')

    # Declare joint handles for each joint in the arms
    #result, joint_one_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint1', vrep.simx_opmode_blocking)
    #result2, joint_one_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint1', vrep.simx_opmode_blocking)
    #result3, joint_two_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint2', vrep.simx_opmode_blocking)
    #result4, joint_two_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint2', vrep.simx_opmode_blocking)
    #result5, joint_three_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint3',
                                                            #vrep.simx_opmode_blocking)
    #result6, joint_three_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint3',
                                                            #vrep.simx_opmode_blocking)
    #result7, joint_four_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint4', vrep.simx_opmode_blocking)
    #result8, joint_four_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint4',
                                                           #vrep.simx_opmode_blocking)
    #result9, joint_five_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint5', vrep.simx_opmode_blocking)
    #result10, joint_five_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint5',
                                                            #vrep.simx_opmode_blocking)
    #result11, joint_six_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint6', vrep.simx_opmode_blocking)
    #result12, joint_six_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint6',
                                                           #vrep.simx_opmode_blocking)
    #result13, joint_seven_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint7',
                                                             #vrep.simx_opmode_blocking)
    #result14, joint_seven_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint7',
                                                             #vrep.simx_opmode_blocking)

    # every other joint
    #result_vert, joint_vert = vrep.simxGetObjectHandle(clientID, 'Baxter_verticalJoint', vrep.simx_opmode_blocking)
    #result_base, joint_base = vrep.simxGetObjectHandle(clientID, 'Baxter_rotationJoint', vrep.simx_opmode_blocking)
    #result_monitor, joint_monitor = vrep.simxGetObjectHandle(clientID, 'Baxter_monitorJoint', vrep.simx_opmode_blocking)

    # arm joint library
    #result_list = [result, result2, result3, result4, result5, result6, result7, result8, result9, result10, result11,
                   #result12, result13, result14]
    #joint_handles = [joint_one_handleL, joint_one_handleR, joint_two_handleL, joint_two_handleR, joint_three_handleL,
                     #joint_three_handleR, joint_four_handleL, joint_four_handleR, joint_five_handleL,
                     #joint_five_handleR, joint_six_handleL, joint_six_handleR, joint_seven_handleL, joint_seven_handleR]

    # In the future, a dictionary will be used to call each individual joint, with the keys being each joint, and the values being the result/joint handlers

def deriveFK():
    return

def runsimulation(clientID, joint_library):
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
        print 'current value of ' + joint + ': theta = {:f}'.format(theta)

    # Stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)

def main():
    theta = []
    body_theta = []

    print "This is a simulation for the Baxter robot"
    body_response = raw_input("Would you like to move the body? (Y or N)")

    if body_response == 'Y':
        print "Input Theta in degrees"
        theta_vert = raw_input("Vert. Joint = ")
        body_theta.append(float(theta_vert))
        theta_body_rotation = raw_input("Body rot. Joint = ")
        body_theta.append(theta_body_rotation)
        monitor_theta = raw_input("Monitor Theta = ")
        body_theta.append(monitor_theta)

    arm_response = raw_input("Would you like to move the arms? (Y or N)")

    if arm_response == 'Y':
        print "Input Theta in degrees"
        theta1 = raw_input("Theta1 = ")
        theta.append(float(theta1))
        theta2 = raw_input("Theta2 = ")
        theta.append(float(theta2))
        theta3 = raw_input("Theta3 = ")
        theta.append(float(theta3))
        theta4 = raw_input("Theta4 = ")
        theta.append(float(theta4))
        theta5 = raw_input("Theta5 = ")
        theta.append(float(theta5))
        theta6 = raw_input("Theta6 = ")
        theta.append(float(theta6))
        theta7 = raw_input("Theta7 = ")
        theta.append(float(theta7))

    if body_response or arm_response != 'N':
        print '111111'
        clientID = initialize_sim()
        joint_library = declarejointvar(clientID)
        deriveFK()
        runsimulation(clientID, joint_library)


if __name__ == "__main__": main()