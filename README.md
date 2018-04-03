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

1. Given theta<sub>initial</sub> and theta<sub>goal</sub>, find theta:[0,1] -> R<sup>n</sup> such thta:

    1. theta is continuous
    2. theta(0) = theta<sub>initial</sub>
    3. theta(1) = theta<sub>goal</sub>
    4. theta is not in collision for 0 < t < 1

2. We detect collision by assigning points along our robot, assigning a volume to the points, we use spheres in this case, and then we determine that these spheres do not come into contact among each other, or with other objects along the path.

3. We obtained the position of each point/sphere and compute the distances to  to all other points in the robot and objects in the environment. We have collision if:

    || P<sub>1</sub> - P<sub>2</sub> || <= radius<sub>i</sub> + radius<sub>j</sub>
    
    
Find the demo [here](https://www.youtube.com/watch?v=TOIL0BH2obo&feature=youtu.be).
