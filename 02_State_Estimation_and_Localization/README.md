# Multi-Sensor (IMU+LIDAR) Fusion for Pose Estimation

This is the final project assignment of [State Estimation and Localization for Self-Driving Cars Course](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars) offered by University of Toronto on Coursera. The "es_ekf.py" file contains the state estimator. In this assignment, I implemented the Error-State Extended Kalman Filter (ES-EKF) to localize a vehicle using data from the [Carla Simulator](https://carla.org/).

## Project notes:
* The data folder contains the data you will use for the project, and the rotations.py file contains a Quaternion class and other rotation-related functions that are already implemented for you.
* The filter relies on *IMU* data to propagate the state forward in time, and *GPS* and *LIDAR* position updates to correct the state estimate. 
* In **part 1** you will fill in the skeleton implementation of the ES-EKF that is provided, by writing code to perform the filter prediction step and the correction step.
* In **part 2**, you will examine the effects of sensor miscalibration on the vehicle pose estimates.
* In **part 3**, you will explore the effects of sensor dropout, that is, when all external positioning information (from *GPS* and *LIDAR*) is lost for a short period of time. 

## Vehicle State Estimation on a Roadway

Kalman filtering is an algorithm that provides estimates of some unknown variables given the measurements observed over time. Kalman filters have been demonstrating its usefulness in various applications. Kalman filters have relatively simple form and require small computational power and are used to estimate states based on linear dynamical systems in state space format.

While recursive least squares updates the estimater of a static parameter, the Kalman Filter is able to update and estimate of an evolving state. The goal of the Kalman filter is to take a probabilistic estimate of this state and update it in real time using two steps: prediction and corretion. Starting from an initial probabilistic estimate at time **<img src="https://render.githubusercontent.com/render/math?math=k-1">** as in the figure below, our goal is to use a motion model which could be derived from wheel odometry or inertial sensor measurements to predict our new state. 

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/kalman_filter.png"/>
</p>

Then, we'll use the observation model derived from *GPS* for example, to correct that prediction of vehicle position at time **<img src="https://render.githubusercontent.com/render/math?math=k">**. Each of these components, the initial estimate, the predicted state and the final corrected state are all random variables that we will specify by their means and covariances. In this way, we can think of the Kalman filter as a technique to fuse information from different sensors to produce a final estimate of some unknown state taking into account uncertainty in motion and our measurements. Below is shown the Kalman filter in its simplest form.

<p align="center">
<img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/kalman_equations.png"/>
</p>

The bad news is that the world does not behave in a linear fashion. For this, Kalman's extended algorithm (EKF) is used around an operation point by means of Jacobian transformation matrices. For more information on this topic, check out this [article](https://www.embedded.com/using-nonlinear-kalman-filtering-to-estimate-signals/) by Dan Simon (for free). A detailed explanation of linearization and how it relates to the EKF can be found in Chapter 13, Sections 1 and 2 of  [Dan Simon, Optimal State Estimation (2006)](https://onlinelibrary.wiley.com/doi/book/10.1002/0470045345).

## Run
* Simply `python3` to execute the code, preferably using `conda` environment

## Algorithm
### Error-State Kalman Filter for State Estimation and Localization
* Using *IMU* and *GNSS* data as measurement input data
* Using **error-state kinematics model** as the motion/measurement model
* Using **quaternion kinematics** as the rotation process update
* Implement the algorithm in `Python3`
* Sample pictures for the results

## Results
* **Part 1: Ideal case**
<p align="center">
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_1/Figure_2.png" width="450" />
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_1/Figure_3.png" width="450" />
</p>

* **Part 2: Sensor's poor calibration**
<p align="center">
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_2/Figure_2.png" width="450" />
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_2/Figure_3.png" width="450" />
</p>

* **Part 3: Sensor's infomation drops**
<p align="center">
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_3/Figure_2.png" width="450" />
  <img src="https://github.com/mattsousaa/Self_Driving_Cars_Specialization/blob/master/02_State_Estimation_and_Localization/Saved%20Pictures/part_3/Figure_3.png" width="450" />
</p>

On these plots, the dashed red lines represent three standard deviations from the ground truth according to our estimator. This gives an indication of how well our model fits the actual dynamics of the vehicle and how well the estimator is performing overall. The estimation error should remain within the 3-sigma bounds at all times, as it does here. For more information, read the code carefully for more details.

## Key Algorithms
* Conversion from **Euler angle** to **Quaternion**:
    ```python
    def to_euler(self):
        """Return as xyz (roll pitch yaw) Euler angles."""
        roll = np.arctan2(2 * (self.w * self.x + self.y * self.z), 1 - 2 * (self.x**2 + self.y**2))
        pitch = np.arcsin(2 * (self.w * self.y - self.z * self.x))
        yaw = np.arctan2(2 * (self.w * self.z + self.x * self.y), 1 - 2 * (self.y**2 + self.z**2))
        return np.array([roll, pitch, yaw])
    ```
    * check the provided util code [`rotations.py`](rotations.py)
    
* Quaternion multiplication:
    * As an example, the `quaternion left multiplication` looks like this:
    ```python
    def quat_mult_right(self, q, out='np'):
        """
        Quaternion multiplication operation - in this case, perform multiplication
        on the right, that is, q*self.

        :param q: Either a Quaternion or 4x1 ndarray.
        :param out: Output type, either np or Quaternion.
        :return: Returns quaternion of desired type.
        """
        v = np.array([self.x, self.y, self.z]).reshape(3, 1)
        sum_term = np.zeros([4,4])
        sum_term[0,1:] = -v[:,0]
        sum_term[1:, 0] = v[:,0]
        sum_term[1:, 1:] = -skew_symmetric(v)
        sigma = self.w * np.eye(4) + sum_term

        if type(q).__name__ == "Quaternion":
            quat_np = np.dot(sigma, q.to_numpy())
        else:
            quat_np = np.dot(sigma, q)

        if out == 'np':
            return quat_np
        elif out == 'Quaternion':
            quat_obj = Quaternion(quat_np[0], quat_np[1], quat_np[2], quat_np[3])
            return quat_obj
    ```
    * check the provided util code [`rotations.py`](rotations.py)
    * Note: it is critical to choose left or right multiplication for quaternions, because it impacts the **coordinate system** in consideration is *local* or *global* 


## Reference
### Websites
* [Coursera website for this course: State Estimation and Locaization](https://www.coursera.org/learn/state-estimation-localization-self-driving-cars/home/welcome)
* [Coursera website for the other courses from University of Toronto about Self-driving Car topics](https://www.coursera.org/specializations/self-driving-cars)

### Papers & Textbooks
* [Quaternion kinematics for the error-state Kalman filter, Sola, 2017](https://arxiv.org/pdf/1711.02508.pdf)
* [Quaternions and Rotations, Jia, 2013](http://graphics.stanford.edu/courses/cs348a-17-winter/Papers/quaternion.pdf)
