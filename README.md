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
