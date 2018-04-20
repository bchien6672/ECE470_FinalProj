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
(Stopped here for deadline, will update later in the week for full forward kinematics module)

Note: I will not be moviong forward in implementing a full forward kinematics module, but a lot of the concepts that are used to derive forward kinematics are included in the following derivations for inverse kinematics, so a lot of the process is to interpret the various axis and dimensioning within the simulator and putting that into valid matrices for kinematics.

Checkpoint 3 (3/26)

To make up for lack of forward kinematics derivations, the main functions that could be used are in the following Checkpoint, as they are needed for inverse kinematics implementations...
1. Implemented into new Checkpoint 3 python code a library of matrix operations that will be useful 
2. Create a spreadsheet for joint x,y,z distances
3. Rewrite user prompt info for what frames the user wants to move
4. Added initial screw calculations derived from joint spreadsheet
5. Created desired transformation pose algorithm
6. Implemented FK module for test transformation matrix
7. Implemented full IK module
8. Implemented joint movement logic that would enable it to determine where it will go while looking for the goal pose
9. Added code for showing where the goal pose is, as well as having it change color when the arm reaches the correct pose
10. bugfixing....

Current bugs:
I would like to check over the screw matrix and initial transformation matrix derivations, as the relative logic for the IK and FK functions are relatively correct.

Checkpoint 4 (4/2)
1. Looked into deciding what type of collision detector to use. (Currently going to work directly with V-REP)
2. Ported existing code into the new module.
3. Decided on algorithm to understand how robot should handle collision (decide on having the joint go back to original position and move on)
4. Compiling set of joint thetas for the robot to perform, currently have 4 sets of thetas that will yield at least 30 configurations, with additional configurations that will occur when the robot has been in collision)
5. Uploaded Robot Scene

Current Issues:
The simulation may sometimes be wonky with the robot returning to the original position, as it will get stuck with the table.

CheckPoint 5 (4/11)
1. Uploaded framework that will be used for path planning
2. Added in valid theta checking function
3. Implemented tree structure for nodal placement
4. Added in iterations for selecting thetas
5. put in logic for moving the joints and using collision checking at the same time.

Note: with additional dynamic script due to debugging being in the scene, the robot may start off awkwardly. Generated path to be found in the youtube description.

Final Video (5/1)
1. Redo ALL of major issues from the past five weeks.
2. 

