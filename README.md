# QUACS: A ROS controller for AR.Drone

## Introduction
This is a repository of a ROS package that controls the parrot AR.DRONE.

## Dependencies
It depends primarily on "ardrone_autonomy" package and "tum_simulator". It runs in ROS FUERTE. 

## Modules
### State Estimation
This module estimates quadrotor state using Digital Low-Pass (Butterworth) Filtering of Sensors, Kalman Filtering to fusion sensors and Odometry to predict positions. 
Publishes results in message topic 'ardrone/sensorfusion/navdata' with type Odometry

### Controller
There is a PD controller implemented to control the position but further control strategies are planned. 
Commands Drone using message topic 'cmd_vel' with type Twist

PID parameters can be modified in PID_parameters.yaml inside /parameters directory. The I parameter is set to zero as it makes the system unstable. 


### State Handling
Both the controller and the drone are finite-state machines and this node handles such properties. (read ardrone_autonomy)
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
To run 
	$ roslaunch ardrone_control ardrone_PIDcontrol_keyboard.launch


## To-Do's
	* Implement a handler for Yaw Controller ( $$ e_{\psi} = \min_K |\psi - \psi_{goal} + 2 K \pi |$$ )
	* Modify controller so as to actuate only when Drone is Flying 
	* Add GPS-Sensor Measurements and Kalman Filter for Estimation
	* Add a new controller
