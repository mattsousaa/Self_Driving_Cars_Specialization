# Self_Driving_Vehicle_Control

This is the final project assignment of [Introduction to Self-Driving Cars Course](https://www.coursera.org/learn/intro-self-driving-cars) offered by University of Toronto on Coursera. The "controller2d.py" file contains a controller object. The final result can be consulted on YouTube through the link: https://www.youtube.com/watch?v=OMtp3IUQzX0.

## Project notes:

* The waypoints include positions as well as the speed the vehicle should attain.
* As a result, the waypoints become the reference signal for our controller and navigating to all the waypoints effectively completes the full track.
* Since the controller reference contains both position and speed, we need to implement both Longitudinal and Lateral Control.
* The output of the controller will be the vehicle throttle, brake and steering angle commands.
* The throttle and brake come from the Longitudinal speed control and the steering comes from our Lateral Control.

## Self-Driving Cars Longitudinal and Lateral Control Design
In this project, I implemented a controller in Python and used it to drive a car autonomously around a track in [Carla Simulator](https://carla.org/). The output of the controller will be the vehicle throttle, brake and steering angle commands. The throttle and brake come from the Longitudinal speed control and the steering comes from the Lateral Control.

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

The Pure Pursuit method consists of geometrically calculating the trajectory curvature. This method connects the centre of rear axle location to a target point on the path ahead of the vehicle. Steering angle determined by target point location and angle between the vehicle's heading direction and lookahead direction. Using the bicycle model the steering angle (**<img src="https://render.githubusercontent.com/render/math?math=\delta">**) can be very easily calculated. The crosstrack error is defined here as the lateral distance between the heading vector and the target point.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/longitudinal.png"/>
</p>

As seen in the figure above I didn't use the look ahead distance directly. Here I associated the yaw to get **<img src="https://render.githubusercontent.com/render/math?math=\alpha">** instead of crosstrack error. First, I stablished a look ahead distance constant as a minimum. Then I search for all the points after the first one and if the distance of this look ahead point is over the look ahead distance minimum then I calculate **<img src="https://render.githubusercontent.com/render/math?math=\hat{\alpha}">**. With alpha is easy to calculate the steering angle as show in the figure. Finaly I tuned the look ahead gain. For more details read the commented code.

## Self-Driving Vehicle Control Simulation

I used Windows 10 to perform this practice and followed the [tutorials](https://d3c33hcgiwev3.cloudfront.net/IFfxQie8Eem9HA6xGGaRfg_20f6060027bc11e98ed3dfcfdba7c72b_CARLA-Setup-Guide-_Windows-x64_.pdf?Expires=1599004800&Signature=Xk8ZORr3BqsBpNycEl5byliVQk60qgNhLEbt1So~aS8UgFVaEoYSC41F3GeeioJpcI-S08CBCD1OMvs0Fok6I-NOzJhDPtkx95vwx2F8Ypb-Q6MJzy0P2pxgCfnuJb1bvIIP~yntnYzMxogTKLdMx2Gvq923zPW~Oi0fCZHRK20_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A) provided for installation. To do this assignment, the [CARLA](https://carla.org/) simulator along with the assignment code needs to be installed. Please follow these [instructions](https://www.coursera.org/learn/intro-self-driving-cars/programming/ac8R5/final-project-self-driving-vehicle-control). The simulator will begin to run if the **module_7 client** connects to the server properly. It will open two new feedback windows, one of which shows the trajectory and the other which shows the controls feedback as you can see below.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/Capturarw.PNG"/>
</p>

The trajectory feedback will contain the car, start and end positions, entire path/path traveled and a small shaded region which denotes the subset of interpolated points to be sent into the controller for control updates. Linear interpolation is used between waypoints to provide a finer resolution path/speed requests for the controller. The X and Y axes are in meters.

The controls feedback shows the throttle, steering and brake outputs, as well as the speed response for the simulation (desired speed and current speed in the single plot). This is a general feedback for viewing what the client is sending to the CARLA server in terms of control commands. The desired speed is set to the closest interpolated speed point to the current position of the car. Below the simulator is the command prompt with throttle, steering and **<img src="https://render.githubusercontent.com/render/math?math=\alpha">** values.

## Results from [Carla Simulator](https://carla.org/)

### Longitudinal Control

To evaluate the performance of the longitudinal controller the next image is provided. In the plot, the speed profile proposed to drive the car around the track is in orange, and the real vehicle speed obtained by using the PID controller is in blue.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/Figure_2.png"/>
</p>

>During the whole trajectory, the controller provided the right outputs to the vehicle to mantain the desired speed within the proposed profile; however, at some point (between the 500 to 750 waypoints) the controller did not reach the desired speed, but a possible solution for this issue is to force a brake signal to reduce the speed in the corner.

### Lateral Control

Similar to the longitudinal controller, a image is provided to shown the performance of the lateral controller. In the plot, the trajectory proposed to drive the car around the track is shown in blue, and the trayectory followed by the vehicle by using the Pure Pursuit Controller is shown in orange.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Vehicle_Control/blob/master/images/Figure_1.png"/>
</p>

>In this case, the vehicle followed the desired trajectory. The Pure Pursuit Controller had a great performance overall and its precision was good enough for this case.

Other types of controllers such as the Stanley Control or even the Model Predictive Control could be implemented to try to enhance the system performance.

If you have any doubts or suggestions feel free to contact me.
