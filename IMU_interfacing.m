a = arduino('/dev/tty.usbmodem143301', 'Uno', 'Libraries', 'I2C');
imu = mpu6050(a);

while true
    
    accelReading = readAcceleration(imu);
    gyrReading = readAngularVelocity(imu);
    disp(accelReading);
    disp(gyrReading);

end 

