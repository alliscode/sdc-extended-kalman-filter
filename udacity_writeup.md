# Extended Kalman Filter Project 
Self-Driving Car Engineer Nanodegree Program 
 
--- 
 
#### [Rubric](https://review.udacity.com/#!/rubrics/748/view) Points 
##### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.   
 
### Compiling 
 
- <b>Your code should compile</b> 
 
 I have verified that the source code in this project will build with cmake and make as described in the starter project readme. CMakeLists.txt has not been modified from it's original form so I do not anticipate any issues here. 
 
### Accuracy 
 
- <b>RMSE <= [0.08, 0.08, 0.60, 0.60] when using the file: "sample-laser-radar-measurement-data-1.txt".</b> 
- <b>RMSE <= [0.20, 0.20, 0.50, 0.85] when using the file: "sample-laser-radar-measurement-data-1.txt".</b> 
 
My program produces the following RMSE accuracies for these inputs: 
 
| Input data | Dimension | RMSE | 
| --- | --- | --- | 
| data-1 | px | 0.0651 | 
| data-1 | py | 0.0606 | 
| data-1 | vx | 0.5302 | 
| data-1 | vy | 0.5442 | 
| data-2 | px | 0.1855 | 
| data-2 | py | 0.1902 | 
| data-2 | vx | 0.4765 | 
| data-2 | vy | 0.8108 | 
 
### Follows the Correct Algorithm 
 
- <b>Your Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.</b> 
 
The general flow of the algorithm is controlled by the `FusionEKF` class. Once the baseline has been established with the first measurement, this class works with an instance of the `kalman_filter` class to perform the following procedure: 
 
1. Receive a new sensor measurement. 
 
2. Predict the new state mean and covariance using the updated state transition matrix and the process noise. 
3. Update the previously predicted state `x' & p'` using the new sensor measurement. The step uses either the basic Kalman filter or the Extended Kalman filter depending on the sensor type. 
 
- <b>Your Kalman Filter algorithm handles the first measurements appropriately.</b> 
 
When the first sensor measurement is presented to the `ProcessMeasurement` method of the `FusionEKF` class, the state of the `FusionEKF` and `kalman_filter` instances are initialized. The specifics of this initialization depend on the type of sensor that is providing the first measurement. 
 
- Laser: 
    - The state mean vector `x` is initialized with the `px` and `py` values from the laser measurement and `vx = vy = 0`. The velocities are initialized to `0` because the laser measurement does not provide velocity readings directly and we do not have enough information at this point to make a reasonable prediction. 
    - The state covariance matrix `P` is initialized with `0` covariance (off diagonal entries), `Var(px) = Var(py) = 1` and `Var(vx) = Var(vy) = 1000`. The high variance in the velocity reflects the fact that our initial guess of `vx = vy = 0` has a high level of uncertainty. 
- Radar: 
    - The state mean vector `x` is initialized by converting the radar measurement from polar to cartesian coordinates. This provides values for the position and velocity dimensions. 
    - The state covariance matrix `P` is initialized to the identity matrix. This reflects the fact that the radar measurement provides direct measurements of position and velocity. 
 
Once the state mean and covariance have been initialized, the initial timestamp is saved before returning. 
 
- <b>Your Kalman Filter algorithm first predicts then updates.</b> 
 
The `ProcessMeasurement` method of the `FusionEKF` class handles the prediction and update routine starting at line number `108`.  
 
The time delta between the current and previous measurements is calculated and used to update the state transition and process noise matrices. These matrices are then passed to the `predict` method of the `kalman_filter` instance. 
 
After the prediction step, the `kalman_filter` instance is used to perform the update, the details of which depend on the sensor type. 
 
- <b>Your Kalman Filter can handle radar and lidar measurements.</b> 
 
To handle the laser and radar measurements, the `kalman_filter` class provides an `Update` method (line number `29`) and an `UpdateEKF` method (line number `43`). 
 
### Code Efficiency 
 
- <b>Your algorithm should avoid unnecessary calculations.</b> 
 
Every attempt has been made to remove inefficiencies while still keeping the code clean and architectural consistent. The following steps have been taken towards this end: 
- Calculations are performed only as many times as needed. A good example of this is the `H` and `H.transform()` matrices that are precalculated and passed to the `Update` method. 
- The `kalman_filter` class has been modified to keep it as general as possible by ensuring that it's interface and implementation are not tied to the specific problem at hand (laser and radar sensor fusion). If a third sensor type was added, the `kalman_filter` class would could still be used without modification. 
 
 
 