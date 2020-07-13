%clear arduino cache to run without error and make matlab plot instead of
%showing raw values

uno = arduino('/dev/tty.usbmodem143301', 'Uno', 'Libraries', 'I2C');
imu = mpu6050(uno);

rollCF = 0.0;
pitchCF = 0.0;

RAD2DEG = 180/pi;
dt = 0.001;
Q_angle = 0.0009;
Q_bias = 0.003;
R = 0.03;

while true 
    accRead = readAcceleration(imu);
    gyrRead = readAngularVelocity(imu);
    
    accRawX = accRead(1);
    accRawY = accRead(2);
    accRawZ = accRead(3);
    
    gyrRawX = gyrRead(1);
    gyrRawY = gyrRead(2);
    gyrRawZ = gyrRead(3);
    
    accAngleX = atan(accRawY/sqrt(power(accRawX,2) + power(accRawZ,2)))*RAD2DEG;
    accAngleY = atan(-1*(accRawX/sqrt(power(accRawY,2) + power(accRawZ,2))))*RAD2DEG;

    gyrRateX = gyrRawX/131.0;
    gyrRateY = gyrRawY/131.0;
    
    gyrAngleX = gyrRateX * dt;
    gyrAngleY = gyrRateY * dt;
    
    rollCF = 0.98 * (rollCF + gyrAngleX) + 0.02 * accAngleX;
    pitchCF = 0.98 * (pitchCF + gyrAngleY) + 0.02 * accAngleY;  
    
    rollKF = kalFilterRoll(dt, Q_angle, Q_bias, R, accAngleX, gyrRateX);
    pitchKF = kalFilterPitch(dt, Q_angle, Q_bias, R, accAngleY, gyrRateY);
    
    fprintf(" %f : %f : %f : %f \n", rollCF, pitchCF, rollKF, pitchKF);
    
    plot
end 



function rollKF = kalFilterRoll(dt, Q_angle, Q_bias, R, newAngle, newRate)

persistent angle0;
persistent P0;


%Process Noise Convariance Matrix
Q = [Q_angle 0; 0 Q_bias] * dt;
F = [1  -dt; 0 1];
H0 = [1 0];
bias0 = 0.0;

rate0 = newRate - bias0;

if isempty(angle0); angle0 = 0;
    
else angle0 = angle0 + (dt * rate0);
end 

rollKF = angle0;

if isempty(P0); P0 = [0, 0; 0, 0];
else P0 = F*P0*F' + Q;
end 
P0_temp = P0;

S0 = H0*P0*H0' + R;
K0 = (P0_temp * H0')/S0;
y0 = newAngle - rollKF; 
rollKF = rollKF + K0(1) * y0;
bias0 = rollKF + K0(2) * y0;
angle0 = rollKF;

I = [1 0; 0 1];
P0_temp = (I - K0*H0)*P0_temp;
P0 = P0_temp;

end 

function pitchKF = kalFilterPitch(dt, Q_angle, Q_bias, R, newAngle, newRate)

persistent angle1;
persistent P1;


%Process Noise Convariance Matrix
Q = [Q_angle 0; 0 Q_bias] * dt;
F1 = [1  -dt; 0 1];
H1 = [1 0];
bias1 = 0.0;

rate1 = newRate - bias1;

if isempty(angle1); angle1 = 0; 
else angle1 = angle1 + (dt * rate1);
end 

pitchKF = angle1;

if isempty(P1); P1 = [0, 0; 0, 0];
else P1 = F1*P1*F1' + Q;
end 
P1_temp = P1;

S0 = H1*P1*H1' + R;
K1 = (P1_temp * H1')/S0;
y1 = newAngle - pitchKF; 
pitchKF = pitchKF + K1(1) * y1;
bias1 = pitchKF + K1(2) * y1;
angle1 = pitchKF;

I = [1 0; 0 1];
P1_temp = (I - K1*H1)*P1_temp;
P1 = P1_temp;

end 