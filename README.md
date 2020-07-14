# IMU Filters
The code needs to be cleaned and optimised. 

# Complementary Filter
A main idea behind using a complementary filter is to integrate the accelerometer and gyroscope data to obtain the attitude of the system. This is achieved by using a low pass filter on the accelerometer and a high pass filter on the gyroscope data, i.e. the data of each of the sensors is combined in a definite ratio whose sum is equal (complement). The ratio I have chosen is 0.98 * Accelerometer and 0.02 * Gyroscope data.  

I have eliminated the initial error in the accelerometer and gyroscope reading by averaging the first 500 values. Followed by which, the angles are calculated.
Basic Understanding on euler angles will be essential. 

The code for implementing the complementary filter on an Arduino + MPU6050 can be found in the file Complementary_Filter.ino 
The  hexadecimel register addresses are taken from the MPU6050 register map. You may change them or write to the induvidual bits of the register to change the scale and range of the measurement. 

# Kalman Filter
The kalman Filter is also be used to estimate the orientation of the system by combining the accelerometer and gyroscope data. This is achieved by using an algorithm that uses a series of measurements observed over time, containing noise and other inaccuracies in its measurements, and  produces estimates of the state of the system which is more accurate than those based on a single measurements like the complementary filter. It is basically a two step process. 
1)Prediction: Here, the kalman filter estimates the current time step state vaiables along with their inconsistancies with the help of the estimates from the previous time step
2)Update: This step calculates the correction of the state and its covariance with estimates in that time step.

The code to implement a Kalman Filter on an Arduino + MPU6050 can be found in KalmanFilterMatlab.m and the KalmanFilterSimulink.slx. Although MATLAB provides pre-made and tested functions for the implementation of kalman filters, I have written the code from scratch to give me a better understanding of the concepts. The code has been tuned to get the most accurate orientation.

A comparison between Complementary Filter vs Kalman Filter can be found in the file ComplementaryVsKalman.m. I have provided a numerical analysis for the comparison, will upgrade to visual based representation soon.
