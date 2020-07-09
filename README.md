# Complementary Filter
Get yaw pitch and roll without the use of any additional library other than Wire.h
Wire.h is used for I2C communication 

A complementary filter is used, it integrates the accelerometer and the gyroscope. It calculates angles by applying a high pass filter to the gyroscope data and a low pass filter to the accelerometer and combines them.

The hexadecimel register addresses maybe seen in the pdf of the register map.

Here, the initial error in the accelerometer and gyroscope in calculated by approximating the initial 500 values. Followed by which new angles are calculated with respect to the initial orientation of the MPU6050.

Basic understanding of euler angles and complementary filter will help.

