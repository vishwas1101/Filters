# IMU Filters

The code needs to be cleaned and optimised. No additional libraries or pre-made functions are used in the uploaded files (other than Wire.h for the Arduino codes).

# Complementary Filter
A main idea behind using a complementary filter is to integrate the accelerometer and gyroscope data to obtain the attitude of the system. This is achieved by using a low pass filter on the accelerometer and a high pass filter on the gyroscope data, i.e. the data of each of the sensors is combined in a definite ratio whose sum is equal (complement). The ratio I have chosen is 0.98 * Accelerometer and 0.02 * Gyroscope data.  

I have eliminated the initial error in the accelerometer and gyroscope reading by averaging the first 500 values. Followed by which, the angles are calculated.
Basic Understanding on euler angles will be essential. 

The code for implementing the complementary filter on an Arduino + MPU6050 can be found in the file Complementary_Filter.ino 
The  hexadecimel register addresses are taken from the MPU6050 register map. You may change them or write to the induvidual bits of the register to change the scale and range of the measurement. 

A quaternion based complementary filter is implemented. This avoids the gimbal lock in euler angles when 1DOF is lost because quaternion goes into 4 Dimensions, it will not have any sigularities while depicting something in 3 Dimensions.

# Kalman Filter
The kalman Filter is also be used to estimate the orientation of the system by combining the accelerometer and gyroscope data. This is achieved by using an algorithm that uses a series of measurements observed over time, containing noise and other inaccuracies in its measurements, and  produces estimates of the state of the system which is more accurate than those based on a single measurements like the complementary filter. It is basically a two step process. 
1)Prediction: Here, the kalman filter estimates the current time step state vaiables along with their inconsistancies with the help of the estimates from the previous time step
2)Update: This step calculates the correction of the state and its covariance with estimates in that time step.

Note that kalman filter can only be applied for linear systems that have gaussian probability distribution.

The code to implement a Kalman Filter on an Arduino + MPU6050 can be found in KalmanFilterMatlab.m and the KalmanFilterSimulink.slx. Although MATLAB provides pre-made and tested functions for the implementation of kalman filters, I have written the code from scratch to give me a better understanding of the concepts. The code has been tuned to get the most accurate orientation. Note that Hardware support package for Arduino needs to be installed. to run the code. This can be found in the Add-On Library in MATLAB. To test the connections between MATLAB and Arduino, run the IMU_interfacing.m file.

A comparison between Complementary Filter vs Kalman Filter can be found in the file ComplementaryVsKalman.m. I have provided a numerical analysis for the comparison, will upgrade to visual based representation soon.

As mentioned kalman filters cabe be used onle for linear systems and which have a gassiun joint probability distribution. This limitation is over come by using Extended Kalman Filters, Unscented Kalman Filters, and Particle Filters.


## Turning the Kalman Filter:

There are certain constants that can be tuned and changed to imporve the performance of the kalman filter 

#### 1)The Process Noise Covariance Matrix Qk: 
This marix consists of the accelerometer variance Q_angle and the the variance in bias Q_bias. Depending on high your Q_angle vlue is the filter becomes slower and less responsive to change, this means that you are trusting your estimate too much. Therefore, a unresponsive and slow filter can be fixed by lowering the Q_angle value. In case of lot of drift in the angles, then you are under valuing your gyroscope bias. So, an increase in the Q_bias will tend to decrease the drift in the angles.

#### 2)The Variance in Measurement R: 
This indicates how much you trust your measurements of the accelerometer. A high value would indicate the variance is high and you trust your new measurements less, this will result in a slow response. A very low value will result in overshooting and noisy behaviour, since you trust the accelerometer measurements too much.

### To Update: 
- Quaternion based kalman filters
- Graphical/Visual representation
