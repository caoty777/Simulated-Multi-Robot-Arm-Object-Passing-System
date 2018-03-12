# ECE470 Introduction to Robotics
## Assignment 1

This portion describes the necessary steps to carry out assignment 1 with python remote API

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
vrep/programming/remoteApiBindings/matlab/matlab/vrep.m
vrep/programming/remoteApiBindings/matlab/matlab/vrepConst.m
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


## Assignment 2

