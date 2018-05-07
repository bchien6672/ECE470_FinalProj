import vrep
import math
import time
import numpy as np
import scipy.linalg as la

""" V-REP Functions """
def getCollisionlib(clientID):
    collision_list = ['CL', 'CR']

    collision_lib = {}

    for item in collision_list:
        res, collision = vrep.simxGetCollisionHandle(clientID, item, vrep.simx_opmode_blocking)
        collision_lib[item] = collision

    return collision_lib


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

def main():
    Larm_theta = []
    Rarm_theta = []
    Larm_flag = False
    Rarm_flag = False

    print "This is a gripping test simulation for the Baxter robot"

    clientID = initialize_sim()
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)
    joint_library, Larm_jointsdict, Rarm_jointsdict, body_jointsdict, joint_bodynames, joint_Rarm, joint_Larm = declarejointvar(clientID)
    collision_library = getCollisionlib(clientID)

    time.sleep(5)

    # stop simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

    # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID)

    # Close the connection to V-REP
    vrep.simxFinish(clientID)


if __name__ == "__main__": main()