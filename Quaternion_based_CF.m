uno = arduino('/dev/tty.usbmodem-13310', 'Uno', 'Libraries', 'I2C');
imu = mpu6050(uno);

Q0 = quaternion(1, 0, 0, 0);
Qk = quaternion();
Qk_next = quaternion();
Qk_next2 = quaternion():
Q_acc = quaternion();
Q_acc_dash = quaternion();
Q_up = qauternion();
e_up = [0 0 1];

u_a = 0.003;
dt = 0.001;

i = 1; 

while True
    i = i + 1;
    
    accRead = readAcceleration(imu);
    gyrRead = readAngularVelocity(imu);
    
    accRawX = accRead(1);
    accRawY = accRead(2);
    accRawZ = accRead(3);
    
    gyrRawX = gyrRead(1);
    gyrRawY = gyrRead(2);
    gyrRawZ = gyrRead(3);
   
    gyrRateX = gyrRawX/131.0;
    gyrRateY = gyrRawY/131.0;
    gyrRateZ = gyrRawZ/131.0;
    
    Q_omega_X = 0.5 * gyrRateX * dt;
    Q_omega_Y = 0.5 * gyrRateY * dt;
    Q_omega_Z = 0.5 * gyrRateZ * dt;
     
    Q_omega = quaternion(1, Q_omega_X, Q_omega_Y, Q_omega_Z);
    
    if i == 1
        Qk = Q0;
    end
    
    Qk_next = Qk * Q_omega;
    
    Q_acc = quaternion(0, accRawX, accRawY, accRawZ);
    
    Q_acc_dash = Qk_next *  Q_acc * conj(Qk_next);
    
    n_up = Q_acc_dash * e_up;
    
    gamma = acos(dot(Q_acc_dash, e_up)/ norm(Q_acc_dash));
    
    Q_up = [1 n_up*sin(ua*gamma/2)];
    
    Qk_next2 = Q_up * Qk_next;
    
    disp(Qk_next2);
    
    Qk = Qk_next2;
    
end 