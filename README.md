# IEEE Robotics Club Competition
•	Implemented PID control for a two-wheel differential drive robot, with feedback data collected from optical encoders

•	Coordinated with path planning team to calculate real-time optimal setpoint sequences using spline interpolation technique

File Description:
1. Motor.h & Motor.cpp - C++ code to implement DC motor PID PWM control with measurement data from optical encoders

2. pid_duo_motors.ino - Arduino sketch to implement the duo-motor control system of the robot

3. Spline.java - Java code to implement the spline interpolation method to obtain a closely spaced sequence of setpoints based on the more sparsely spaced setpoints provided by path planning team

4. Trajectory.java - Java code to convert the setpoint sequence to a sequence of individual wheel velocity commands for the motor controllers
