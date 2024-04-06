# YouBot-Mobile-Robot-Controller
To run this code, first download the Modern Robotics repository from https://github.com/NxRLab/ModernRobotics. 
To simulate the results, install CoppeliaSim. (https://coppeliarobotics.com/)

This repo demonstrates the Modern Robotics Capstone Project.
This project demonstrates YouBot's control in CoppeliaSim. The project is divided into 4 major milestones. 

# milestone 1
Run WrapperTrajGen.m to generate the reference trajectory of the end effector. It will generate a wrapper_generated_trajectory.csv
file, which contains the EE pose. Scene 8 in CoppeliaSim will be used to showcase the reference trajectory of the EE.

# milestone 2
Run WrapperNextState.m to generate the next state of the robot. This program generates the wrapper_next_state.csv file. Use
scene 6 to showcase the reference motion of the YouBot.

# milestone 3
Run WrapperFeedbackControl.m to obtain the joint velocities and the pose error. Use the given set of input to validate the results
of the function.

# milestone 4
This is a wrapper function FinalProject.m, which uses previous functions to accomplish the task of pick-n-place the cube. It
generates trajectory (generated_trajectory.csv), subsequently joint velocities, and the robot's next state  (next_state1.csv), which 
can be simulated in CoppeliaSim Scene 6. 
This program is designed for three tasks: Best, Overshooting, and the New Task. The initial and final locations of the cube are user-defined. Kp & Ki matrices for Best, Overshoot, and Next task are given in the program and must be chosen for each task.

# results
Best: https://www.youtube.com/watch?v=x05bqU1cqyI
Overshoot: https://www.youtube.com/watch?v=_bFjHEhTV6I
New Task: https://www.youtube.com/watch?v=vGOI-ic2gas
 
