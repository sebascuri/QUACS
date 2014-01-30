# QUACS: A ROS controller for AR.Drone

## Table of Contents

- [Introduction](#introduction)
- [Dependencies](#dependencies)
- [Modules](#odules)
	- [State Estimation](#state-estimation)
	- [Controller](#controller)
	- [State Handling](#state-handling)
- [Usage](#usage)
	- [Keyboard Command](#keyboard-command)
	- [Joystick Command](#joystick-command)
	- [Text File Command](#text-file-command)
- [Modifications](#modifications)
- [ToDo](#todo)

## Introduction
This is a repository of a ROS package that controls the parrot AR.DRONE.

## Dependencies
It depends primarily on "ardrone_autonomy" and "tum_simulator" packages. It runs in ROS FUERTE. 

## Modules
### State Estimation
This module estimates quadrotor state using Digital Low-Pass (Butterworth) Filtering of Sensors, Kalman Filtering to fusion sensors and Odometry to predict positions. 

Publishes results in message topic 'ardrone/sensorfusion/navdata' with type Odometry

### Controller
There is a PD controller implemented to control the position but further control strategies are planned. 

Commands Drone using message topic 'cmd_vel' with type Twist

PID parameters can be modified in PID_parameters.yaml inside /parameters directory. The I parameter is set to zero as it makes the system unstable. 


### State Handling
Both the controller and the drone are finite-state machines and this node handles such properties. (read https://github.com/AutonomyLab/ardrone_autonomy)
When executed this module A GUI window will open, while active:

* Press 'Enter' for Drone to take-off
* Press 'Backspace' for Drone to land
* Press 'Space bar' for Drone to Reset
* Press Right (Left) arrow to increase (decrease) Y-coordinate goal
* Press Up (Down) arrow to increase (decrease) X-coordinate goal
* Press W (X) arrow to increase (decrease) Z-coordinate goal
* Press A (D) arrow to increase (decrease) Yaw-coordinate goal

It publishes goal in message topic 'ardrone/trajectory' with type Odometry
It publishes take-off, land and reset messages to handle Drone State

## Usage
### Keyboard Command
```bash
$ roslaunch ardrone_control ardrone_PIDcontrol_keyboard.launch
```
### Joystick Command
To-Do

### Text File Command
To-Do

## Modifications
To add your own module any module of the *.launch file can be commented 

## ToDo

* Implement a handler for periodicity in yaw error 
<p align="center">
![equation] (http://latex.codecogs.com/gif.latex?%24%20%5Cleft%28%20e_%7B%5Cpsi%7D%20%3D%20%5Cmin_K%20%7C%5Cpsi%20-%20%5Cpsi_%7Bgoal%7D%20&plus;%202%20K%20%5Cpi%20%7C%20%5Cright%20%29%20%24)
</p>
* Modify controller so as to actuate only when Drone is Flying 
* Implement a PS3 Joystick Module
* Add GPS-Sensor Measurements and Kalman Filter for Estimation
* Add a new controller
* Make a GUI that shows Drone Status
