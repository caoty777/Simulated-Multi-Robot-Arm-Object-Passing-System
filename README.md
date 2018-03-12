# ECE470 Introduction to Robotics
## Assignment 1

This portion describes the necessary steps to carry out assignment 1 with python remote API

1. Download V-REP PRO EDU from the Coppelia Robotics website: [here](http://www.coppeliarobotics.com)


2. from the folder run "./vrep.app/Contents/MacOS/vrep"

3. Drag a robot into the GUI, and remove associated scripts.
![alt text](https://github.com/axander89/ECE470/blob/master/imgs/assignmet1_1.png " remove child scripts")


4. save the scene "File -> Save Scene As..."

5. Install python. [Conda](https://www.anaconda.com) is highly recommended (Products -> Download -> Python 3.6 version)  

6. create a workspace for your robot: create a new "my_workspace" folder.

7. copy these files into "my_workspace" folder

vrep/programming/remoteApiBindings/python/python/vrep.py
vrep/programming/remoteApiBindings/python/python/vrepConst.py
vrep/programming/remoteApiBindings/lib/lib/remoteApi.dylib

8. copy this python test code (vrep/programming/remoteApiBindings/python/python/simpleTest.py) into "my_workspace"

9. modify this code like the code we provide, rename it "assignment1.py"

10. from terminal run "python assignment1.py"

You should see your robot move, and in the terminal, you should see the joint angle values. 


More information can be found in the documentation (e.g., look at "Writing code in and around V-REP"): [here](http://www.coppeliarobotics.com/helpFiles/index.html)


Commands available to you in the python/Matlab remote API: [python](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
[matlab](http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm)


## Assignment 2

