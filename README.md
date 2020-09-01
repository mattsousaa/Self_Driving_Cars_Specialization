# Self_Driving_Vehicle_Control

This is the final project assignment of Introduction to Self-Driving Cars Course offered by University of Toronto on Coursera. The "controller2d.py" file contains a controller object. The final result can be consulted on YouTube through the link: https://www.youtube.com/watch?v=OMtp3IUQzX0.

## Project notes:

* The waypoints include positions as well as the speed the vehicle should attain.
* As a result, the waypoints become the reference signal for our controller and navigating to all the waypoints effectively completes the full track.
* Since the controller reference contains both position and speed, we need to implement both Longitudinal and Lateral Control.
* The output of the controller will be the vehicle throttle, brake and steering angle commands.
* The throttle and brake come from the Longitudinal speed control and the steering comes from our Lateral Control.

## Self Driving Cars Longitudinal and Lateral Control Design
In this project, I implemented a controller in Python and used it to drive a car autonomously around a track in Carla Simulator. The output of the controller will be the vehicle throttle, brake and steering angle commands. The throttle and brake come from the Longitudinal speed control and the steering comes from the Lateral Control.

### 1. Longitudinal Control
![image](https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/pid_longitudinal.png)

For longitudinal control I implemented a PID Controller responsible for take the desired speed as the reference and outputs throttle and brake.

A PID controller consists of three components. First, a pure gain Kp that scales the vehicle acceleration based on the speed error. This ensures that the vehicle is accelerating in the correct direction with the magnitude proportional to the error.

Second, in integral term KI sets up the output based on accumulated past errors. This ensures the steady steed errors are eliminated for ramp referencing.

Finally, the derivative term KD dampens the overshoot caused by the integration term.

To complete the longitudinal control, we must convert the acceleration output from the PID controller into throttle and brake commands. For simplicity, positive outputs will be throttle and negative outputs will correspond to brake.

### 2. Lateral Control

For lateral control, I implemented the Pure Pursuit controller. First of all, we need to define some orientation axles for the vehicle (bicycle) model as seen below. All states variables and inputs are defined relative to the centre of front axle. Main vehicle reference frame for lateral control can be at: The center of front axle, the center of gravity (cg) or the center of rear axle.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/car_axles.png"/>
</p>

The Heading error **(<img src="https://render.githubusercontent.com/render/math?math=\psi">)**  is the component of velocity perpendicular to trajectory divided by ICR radius. For lateral control the desired heading must be zero. The Crosstalk error **(<img src="https://render.githubusercontent.com/render/math?math=e">)** is the distance from center of front axle to the closest point on path. Yaw angle **(<img src="https://render.githubusercontent.com/render/math?math=\theta_c">)** is the heading angle of the vehicle with respect to the global frame of reference (varies from **<img src="https://render.githubusercontent.com/render/math?math=-\pi">** to **<img src="https://render.githubusercontent.com/render/math?math=-\pi">**). Similarly, the heading of the path or path Yaw is the sum of **<img src="https://render.githubusercontent.com/render/math?math=\psi %2B\theta_c">**. Lastly, the **<img src="https://render.githubusercontent.com/render/math?math=v">** variable is the current forward speed of the bicycle.

Geometric path tracking is one of the most popular classes of path tracking in robotics and autonomous vehicle. It exploits geometric relationship between the vehicle and the path resulting in compact control law solutions to the path tracking problem. The pure pursuit method consists of geometrically calculating the trajectory curvature. This method connects the centre of rear axle location to a target point on the path ahead of the vehicle.






