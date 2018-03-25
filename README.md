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

Find our demo [here.](https://www.youtube.com/watch?v=sbDnvnmbf_Q)


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

Find our demo [here](https://youtu.be/UyVPlmozkV8). As a bonus, we included the inverse kinematics, which will be next week's checkpoint.


## Checkpoint 3

Checkpoint 3 describes a way to derive and implement the inverse kinematics of the Jaco arm. 

Find a [dummy demo](https://youtu.be/UyVPlmozkV8)





