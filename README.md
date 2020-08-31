# Self_Driving_Vehicle_Control

This is the final project assignment of Introduction to Self-Driving Cars Course offered by University of Toronto on Coursera. The "controller2d.py" file contains a controller object. The final result can be consulted on YouTube through the link: https://www.youtube.com/watch?v=OMtp3IUQzX0.

* The waypoints include positions as well as the speed the vehicle should attain.
* As a result, the waypoints become the reference signal for our controller and navigating to all the waypoints effectively completes the full track.
* Since the controller reference contains both position and speed, we need to implement both Longitudinal and Lateral Control.

### 1. Longitudinal Control
In this project, I implemented a controller in Python and used it to drive a car autonomously around a track in Carla Simulator. The output of the controller will be the vehicle throttle, brake and steering angle commands. The throttle and brake come from the Longitudinal speed control and the steering comes from the Lateral Control.


