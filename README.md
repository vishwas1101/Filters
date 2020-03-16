# MPU6050
Get yaw pitch and roll without the use of any additional library other than Wire.h

The hexadecimel register addresses maybe seen in the register map provided.

Here, the initial error in the accelerometer and gyroscope in calculated by approximating the initial 500 values. Followed by which new angles are calculated with respect to the initial orientation of the MPU6050.

Basic understanding of euler angles and complementary filter will help.
