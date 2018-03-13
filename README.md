### ECE470_FinalProj
ECE470 Final Project SP18 (Will be updating throughout the semester)

Checkpoint 1 (3/5)
User log

In order to begin this endeavor, I first downloaded the V-REP executable file cited by a student for Windows 64bit. After following the prompt instructions...

1. Download V-REP from Piazza link
2. (I already have Anaconda Python 2.7 from past projects, so I will forego the step of obtaining Anaconda)
3. Played with the V-REP environment first by placing in UR-3 arms
4. Removed the arms and placed in Baxter
5. Added gripper and suction cup and mated them with the robot body
6. Added environment objects table and cup, with the latter being translated onto the table
7. Copied over vrep.py and vrepConst.py to my workspace folder that I had created in the V-REP scenes folder
8. Saved scene file with .ttt ext
9. Played with initial code by replacing UR3_armjoint name with Baxter_leftArm_joint1
10. Used different theta values to see the range of motion
11. Put all of them together without the wait (realize it is quite important!!)
12. Place all joint handlers into a list so that they can be called for future reference (will use a dictionary in the future)
13. Using a for loop, iterate over all joints to move a specified amount of motion
14. Same as 13, going the other direction
15. Move the main base and monitor joints

Future Plans: Migrate to GitHub, implement full joint dictionary so that things aren't hard code, calibrate robot

Checkpoint 2 (3/12)

This week involved implementing Forward Kinematics to the Baxter Robot

1. Consolidate all joint handlers into one joint library
2. Separate the joint library into three separate joint dictionaries for each set of joints
3. Draw up generalized schematic of Baxter robot using unit distances for each robot (assumed each arm was on the same plane)
4. Wrote down initial T_01(0) homogeneous transformation matrices for each set of joints
5. Calculated initial spatial screw axes (farthest we shall go on paper)
6. Since the transformation matrix can be described as the sequential exponentials and the homogeneous transformation matrix, only one set of transformation matrices was written down (base to monitor).
7. Organized code so that each part of the things setting up and running the simulation were in separate functions.
8. Wrote down user input commands that would take in the theta positions for each robot joint depending on which joints the user picked.
9. Added basic dummy to represent tool frame that will be target.
10. Placed dummy frames in tool_frame positions, as well as placed base frame
11.
