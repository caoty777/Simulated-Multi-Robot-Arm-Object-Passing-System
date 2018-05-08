# ECE470 Introduction to Robotics
## Checkpoint 1

Checkpoint 1 describes the necessary steps to obtain remote api comunication with a desired robot in V-REP using python or matlab.
In this specific tutorial, we use the Jaco arm.

1. Download V-REP PRO EDU from the Coppelia Robotics website: [here](http://www.coppeliarobotics.com)


2. from the folder run "./vrep.app/Contents/MacOS/vrep"

3. Drag a robot into the GUI, and remove associated scripts.
![alt text](https://github.com/axander89/ECE470/blob/master/imgs/assignmet1_1.png " remove child scripts")


4. save the scene "File -> Save Scene As..."

5. Install python. [Conda](https://www.anaconda.com) is highly recommended (Products -> Download -> Python 3.6 version)  

6. create a workspace for your robot. (i.e. create a new "my_workspace" folder)

7. copy these files into "my_workspace" folder.

##### Python
vrep/programming/remoteApiBindings/python/python/vrep.py
vrep/programming/remoteApiBindings/python/python/vrepConst.py
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib

##### Matlab
vrep/programming/remoteApiBindings/matlab/matlab/remMpi.m
vrep/programming/remoteApiBindings/matlab/matlab/remoteApiProto.m
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib

8. copy this test code into "my_workspace"

##### Python
(vrep/programming/remoteApiBindings/python/python/simpleTest.py) 

##### Matlab
(vrep/programming/remoteApiBindings/matlab/matlab/simpleTest.m)

9. modify this code like the code we provide, rename it "assignment1.py" for python or "assignment1.m" for matlab.

10. If you are using python, from terminal run "python assignment1.py".  If you are using matlab, from matlab run the code assignment1.m.

You should see your robot move, and in the terminal, you should see the joint angle values. 

Find the demo [here.](https://www.youtube.com/watch?v=sbDnvnmbf_Q)


More information can be found in the documentation (e.g., look at "Writing code in and around V-REP"): [here](http://www.coppeliarobotics.com/helpFiles/index.html)


Commands available to you in the python/Matlab remote API: [python](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
[matlab](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm)


## Checkpoint 2

Checkpoint 2 describes a way to derive and implement the forward kinematics of the Jaco arm. 

1. First we obtain the schematics of the Jaco arm. ![alt text](https://github.com/axander89/ECE470/blob/master/imgs/assignment2_1.png "schematics")

2. We define a set of desired joint angles, theta_desired.

3. We obtain T<sub>1</sub><sup>0</sup>, where T<sub>1</sub><sup>0</sup> is the homogeneous configuration of the base {b} of the Jaco arm w.r.t. the world.

4. For each joint we manually obtain a<sub>i</sub>, p<sub>i</sub>, and S<sub>i</sub> w.r.t. the base {b}.

5. We convert S<sub>i</sub> to be w.r.t. the world.

6. We obtain M<sub>1</sub><sup>0</sup>, where M<sub>1</sub><sup>0</sup> is the homogeneous configuration of the end-effector w.r.t. the world.

7. We predict and print the resulting configuration M<sub>2</sub><sup>0</sup> = e<sup>[S<sub>1</sub>]theta_desired<sub>1</sub></sup> . . .e<sup>[S<sub>6</sub>]theta_desired<sub>6</sub></sup>M<sub>1</sub><sup>0</sup>.

8. We position a 'dummy' frame at M<sub>2</sub><sup>0</sup>.

9. We position our Jaco arm at the desired angles theta_desired<sub>i</sub>.

10. We retrieve and print the resulting pose of the end-effector.

11. We compare the printed results and the coordinates of the 'dummy' and the end-effector.

12. For the bonus point, we implement the inverse kinematics, which will be next week's checkpoint.

Find the demo [here](https://youtu.be/UyVPlmozkV8).


## Checkpoint 3

Checkpoint 3 describes a way to derive and implement the inverse kinematics of the Jaco arm. 

1. The goal pose of the tool frame is represented by a yellow dummy ball in the scene. During a simulation, the user can drag the dummy ball to any place he/she wants and the jaco arm will try to reach the new goal pose.

2. From last week's work, we already have the screw axes S<sub>i</sub> of all six joints and the forward kinematics algorithm.

3. Now we begin the inverse kinematics algorithm. First, we randomly choose a set of joint angles theta_1, theta_2...theta_6 as the initial joint angles of the jaco arm.

4. Using the forward kinematics function with the six joint angles as input, we obtain the predicted pose of the tool frame T<sub>curr</sub> w.r.t the world frame.

5. We obtain the current goal pose T<sub>goal</sub> w.r.t the world frame, as specified by the user input.

6. From the formula T<sub>goal</sub>=e<sup>[V]</sup>M, we derive that [V]=logm(T<sub>goal</sub>M<sup>-1</sup>).

7. We extract the spatial twist V from its matrix form [V].

8. We check if the norm of V is less than 0.01. If so, we stop the algorithm since the current pose is already close enough to the goal pose, and the current set of joint angles is the desired set of joint angles. So we skip to step 13. If not, we continue to the next step.

9. We call the helper function to calculate the space jacobian matrix J associated with the current set of joint angles.

10. From the formula V=J*theta_dot, we derive that theta_dot=J<sup>-1</sup>V.

11. Finally, we update the joint angles by theta_new = theta + theta_dot, where theta is the current set of joint angles and theta_new is the updated set of joint angles.

12. We go back to step 4 with the updated set of joint angles and iterate again.

13. We feed the desired set of joint angles to the six joints of the jaco arm. In simulation, the arm should move to align the tool frame with the goal frame indicated by the dummy ball.

14. For the bonus point, we created our own code to get the handle of the JACO arm. This provides us with the ability to open and close the JACO hand at any time we want. 

Find the demo [here](https://youtu.be/3RZ7FuynWso).



## Checkpoint 4

Checkpoint 4 describes a way to detect self-collision and collision with other objects in the environment.

1. We detect collision by assigning points along our robot, assigning a volume to the points, we use spheres in this case, and then we determine that these spheres do not come into contact among each other, or with other objects along the path.

2. We obtained the position of each point/sphere and compute the distances to  to all other points in the robot and objects in the environment. We have collision if:

    || P<sub>1</sub> - P<sub>2</sub> || <= radius<sub>i</sub> + radius<sub>j</sub>
    
    
    ![alt text](https://github.com/axander89/ECE470/blob/master/imgs/Collision1.png  "Collision1")
    ![alt text](https://github.com/axander89/ECE470/blob/master/imgs/Collision2.png "Collision2")
    
Find the demo [here](https://www.youtube.com/watch?v=TOIL0BH2obo&feature=youtu.be).

For the bonus, we implemented hand graspping. Find the bonus [here](https://www.youtube.com/watch?v=PDSB02fIcXc&feature=youtu.be).


## Checkpoint 5

Checkpoint 5 implements path planning algorithm. Building up from checkpoint 4 with collision detection, we implement a sampling base path planning algorithm that generates a set of desired joint angles for the Jaco to move  along a  non-collision path to achieve a desired pose. More specific, we implement the RRT algorithm for generating a non-collision path.

1. Given theta<sub>initial</sub> and theta<sub>goal</sub>, find theta<sub>t</sub>:[0,1] -> R<sup>N</sup>:[0,1] 

    such that:
    1. theta is continuous
    2. theta(0) = theta<sub>start</sub>
    3. theta(1) = theta<sub>goal</sub>
    4. theta<sub>t</sub> is not in collision for 0 < t < 1
    
    ![alt text](https://github.com/axander89/ECE470/blob/master/imgs/PathPlanning.png "PathPlanning")

2. Following is an illustration of the RRT path planning algorithm including path smoothing for optimization.

    ![alt text](https://github.com/axander89/ECE470/blob/master/imgs/PathPlanning2.png "PathPlanning1")
    
Find the demo [here](https://www.youtube.com/watch?v=ouvhs-biBL8&feature=youtu.be).


## Checkpoint 6 - Final Demo

The goal of the demo for the final project is to have 3 Jaco arms with grippers translate an object from an initial position to a final destination. For our scene, we placed the target object and Jaco arm 1 on table 1, the Jaco arm 3 and final destination dummy on table 2, and Jaco arm 2 in between to make the transfer between Jaco1 and Jaco 2. Also, we placed 2 dummy points between the Jaco arms to set the delivery and pick up points for each Jaco arm at the time of each transfer. These dummy points were used for the inverse kinematics for the transfer tasks.

![alt text](https://github.com/axander89/ECE470/blob/master/imgs/FinalScene.png "V-REP scene for the final project demo.")

We perform the final demo as follows:

1. At start Jaco arm 1 will go to pick up the target object by 

    1. Moving Jaco 1 slightly above the target and moving Jaco 2 to pick-up location 1
    2. Moving incrementally Jaco 1 down to the location of the target
    3. Closing gripper of Jaco 1

2. Then,  we do transfer 1 by,

    1. Moving Jaco 1 slightly above the drop-off location 1
    2. Moving incrementally Jaco 1 down to the drop-off location 1
    3. Closing gripper of Jaco 2
    4. Opening gripper of Jaco 1
    5. Moving incrementally Jaco 1 above the drop-off location 1
    6. Moving Jaco 1 to initial position

3. Then we do transfer 2 by,

    1. Moving Jaco 2 to the drop-off location 2
    2. Moving Jaco 3 slightly above the pick-up location 2
    3. Moving incrementally Jaco 3 down to the pick-up location 2
    4. Closing gripper of Jaco 3
    5. Opening gripper of Jaco 2
    6. Moving incrementally Jaco 3 above the pick-up location 2
    7. Moving Jaco 3 and Jaco 2 to initial position

4. Then we move the target to final destination by;

    1. Moving Jaco 3 slightly above the destination 
    2. Moving incrementally Jaco 3 down to the destination
    3. Opening gripper of Jaco 1
    4. Moving incrementally Jaco 3 above the destination
    5. Moving Jaco 3 to initial position

5. And last but not least we wave by moving only joints 3 of the Jaco arms from -pi/4 to pi/4 3 consecutive times.

The code for the final demo is [JacoFinalDemo](https://github.com/axander89/ECE470/blob/master/py_code/JacoFinalDemo.py), and our final demo is [here](https://www.youtube.com/watch?v=lUvLlf-LwAg&t=4s).

