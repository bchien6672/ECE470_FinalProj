import vrep
import time
import numpy as np

# Close all open connections (just in case)
vrep.simxFinish(-1)

# Connect to V-REP (raise exception on failure)
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID == -1:
    raise Exception('Failed connecting to remote API server')

joint_names = ['Baxter_leftArm_joint1', 'Baxter_rightArm_joint1', 'Baxter_leftArm_joint2', 'Baxter_rightArm_joint2', 'Baxter_leftArm_joint3','Baxter_rightArm_joint3', 'Baxter_leftArm_joint4', 'Baxter_rightArm_joint4', 'Baxter_leftArm_joint5', 'Baxter_rightArm_joint5', 'Baxter_leftArm_joint6','Baxter_rightArm_joint6', 'Baxter_leftArm_joint7', 'Baxter_rightArm_joint7', 'Baxter_verticalJoint', 'Baxter_rotationJoint', 'Baxter_monitorJoint' ]

# Declare joint handles for each joint in the arms
result, joint_one_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint1', vrep.simx_opmode_blocking)
result2, joint_one_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint1', vrep.simx_opmode_blocking)
result3, joint_two_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint2', vrep.simx_opmode_blocking)
result4, joint_two_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint2', vrep.simx_opmode_blocking)
result5, joint_three_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint3', vrep.simx_opmode_blocking)
result6, joint_three_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint3', vrep.simx_opmode_blocking)
result7, joint_four_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint4', vrep.simx_opmode_blocking)
result8, joint_four_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint4', vrep.simx_opmode_blocking)
result9, joint_five_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint5', vrep.simx_opmode_blocking)
result10, joint_five_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint5', vrep.simx_opmode_blocking)
result11, joint_six_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint6', vrep.simx_opmode_blocking)
result12, joint_six_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint6', vrep.simx_opmode_blocking)
result13, joint_seven_handleL = vrep.simxGetObjectHandle(clientID, 'Baxter_leftArm_joint7', vrep.simx_opmode_blocking)
result14, joint_seven_handleR = vrep.simxGetObjectHandle(clientID, 'Baxter_rightArm_joint7', vrep.simx_opmode_blocking)

#every other joint
result_vert, joint_vert = vrep.simxGetObjectHandle(clientID, 'Baxter_verticalJoint', vrep.simx_opmode_blocking)
result_base, joint_base = vrep.simxGetObjectHandle(clientID, 'Baxter_rotationJoint', vrep.simx_opmode_blocking)
result_monitor, joint_monitor = vrep.simxGetObjectHandle(clientID, 'Baxter_monitorJoint', vrep.simx_opmode_blocking)

#arm joint library
result_list = [result, result2, result3, result4, result5, result6, result7, result8, result9, result10, result11, result12, result13, result14]
joint_handles = [joint_one_handleL, joint_one_handleR, joint_two_handleL, joint_two_handleR, joint_three_handleL, joint_three_handleR , joint_four_handleL, joint_four_handleR, joint_five_handleL, joint_five_handleR, joint_six_handleL, joint_six_handleR, joint_seven_handleL, joint_seven_handleR]

#In the future, a dictionary will be used to call each individual joint, with the keys being each joint, and the values being the result/joint handlers
joint_library = {}
for joint in joint_names:
    joint_library[joint] = None

for item in result_list:
    if item != vrep.simx_return_ok:
        raise Exception('Cannot get object handle of one of the joints. Check again')

# Start simulation
vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot)

# Wait for simulation to settle down
time.sleep(2)

# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handleL, vrep.simx_opmode_blocking)


if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
print('current value of first joint variable: theta = {:f}'.format(theta))

#initial motion
for joint in joint_handles:
    vrep.simxSetJointTargetPosition(clientID, joint, theta + (np.pi / 2), vrep.simx_opmode_oneshot)
    time.sleep(2)


for joint in joint_handles:
    vrep.simxSetJointTargetPosition(clientID, joint, theta - (np.pi / 4), vrep.simx_opmode_oneshot)
    time.sleep(2)
# Set the desired values of the joint variables
#vrep.simxSetJointTargetPosition(clientID, joint_one_handleL, theta + (3 * np.pi / 2), vrep.simx_opmode_oneshot)
#vrep.simxSetJointTargetPosition(clientID, joint_one_handleR, theta + (np.pi / 2), vrep.simx_opmode_oneshot)
for joint in joint_handles:
    vrep.simxSetJointTargetPosition(clientID, joint, 0, vrep.simx_opmode_oneshot)
# Wait two seconds
time.sleep(2)

#moving base...
vrep.simxSetJointTargetPosition(clientID, joint_vert, theta - (np.pi / 6), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_vert, theta + (np.pi / 6), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_vert, 0, vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_base, theta + (2 * np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_base, theta - (4 * np.pi), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_base, 0, vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_monitor, theta + (np.pi/4), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_monitor, theta - (np.pi/2), vrep.simx_opmode_oneshot)
time.sleep(2)
vrep.simxSetJointTargetPosition(clientID, joint_monitor, 0, vrep.simx_opmode_oneshot)
time.sleep(2)
# Get the current value of the first joint variable
result, theta = vrep.simxGetJointPosition(clientID, joint_one_handleL, vrep.simx_opmode_blocking)
if result != vrep.simx_return_ok:
    raise Exception('could not get first joint variable')
#print('current value of first joint variable: theta = {:f}'.format(theta))

# Stop simulation
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)

# Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
vrep.simxGetPingTime(clientID)

# Close the connection to V-REP
vrep.simxFinish(clientID)
