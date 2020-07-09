#include <Wire.h>
#define RAD2DEG 180/PI

int accRaw[3], gyrRaw[3];
int accError[3], gyrError[3];
int accCal[2], gyrCal[2];
int roll, pitch;

//constants and noise 
float Q = 2.0, R = 4.0;   //covariance noise, Gaussian noise 
float kalGain[2]; //kalman gains 
float P[3] = {0.1, 0.1, 0.1};  //error is covariance matrices 

//time
float dt = 0.01;

unsigned long t, timer;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x00);
  Wire.endTransmission(true);

   //Accelerometer 
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);

  //Gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x10);
  Wire.endTransmission(true);
  
  //Calculating error of gyroscope and accelerometer
  //take average of 500 values as the error 


  t = millis(); 
  
  //getting accelerometer error 
  for(int i = 0; i<500; i++){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                       
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    accRaw[0] = (Wire.read()<<8|Wire.read())/4096.0; 
    accRaw[1] = (Wire.read()<<8|Wire.read())/4096.0;
    accRaw[2] = (Wire.read()<<8|Wire.read())/4096.0;

    accError[0] = accError[0] + ((atan((accRaw[1])/sqrt(pow((accRaw[0]),2) + pow((accRaw[2]),2)))*RAD2DEG));
    accError[1] = accError[1] + ((atan(-1*(accRaw[0])/sqrt(pow((accRaw[1]),2) + pow((accRaw[2]),2)))*RAD2DEG));
    accError[2] = accError[2] + ((atan((accRaw[2])/sqrt(pow((accRaw[0]),2) + pow((accRaw[2]),2)))*RAD2DEG));
   }
  for(int i = 0; i<3; i++){
    accError[i] = accError[i]/500.0;
   }

  //getting gyroscope error
  for(int i = 0; i<500; i++){
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    gyrRaw[0] = (Wire.read()<<8|Wire.read());
    gyrRaw[1] = (Wire.read()<<8|Wire.read());
    gyrRaw[2] = (Wire.read()<<8|Wire.read());

    gyrError[0] = gyrError[0] + (gyrRaw[0]/32.8);
    gyrError[1] = gyrError[1] + (gyrRaw[1]/32.8);
    gyrError[2] = gyrError[2] + (gyrRaw[2]/32.8); 
  }
  for(int i = 0; i<3; i++){
    gyrError[i] = gyrError[i]/500.0;
  }
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  accRaw[0] = (Wire.read()<<8|Wire.read());
  accRaw[1] = (Wire.read()<<8|Wire.read());
  accRaw[2] = (Wire.read()<<8|Wire.read());

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  gyrRaw[0] = Wire.read()<<8|Wire.read();
  gyrRaw[1] = Wire.read()<<8|Wire.read();
  gyrRaw[2] = Wire.read()<<8|Wire.read();

  accCal[0] = atan2((accRaw[1] - accError[1])/256, (accRaw[2] - accError[2])/256) * RAD2DEG;
  gyrCal[0] = gyrCal[0] + ((gyrRaw[0] - gyrError[0])/14.375)*dt;

  if(gyrCal[0] >= 180)
  {
    gyrCal[0] = gyrCal[0] - 360;
  }
  else if(gyrCal[0] < 180)
  {
    gyrCal[0] = gyrCal[0] + 360;
  }

  roll = roll + ((gyrRaw[0] - gyrError[0])/14.375)*dt;

  accCal[1] = atan2((accRaw[0] - accError[0])/256, (accRaw[2] - accError[2])/256) * RAD2DEG;
  gyrCal[1] = gyrCal[1] + ((gyrRaw[1] - gyrError[1])/14.375)*dt;


  if(gyrCal[1] >= 180)
  {
    gyrCal[1] = gyrCal[1] - 360;
  }
  else if(gyrCal[1] < 180)
  {
    gyrCal[1] = gyrCal[1] + 360;
  }

  pitch = pitch - ((gyrRaw[1] - gyrError[1])/14.375) * dt;

  P[0] = P[0] + (2*P[1] + dt*P[2])*dt;
  P[1] = P[1] + P[2] * dt;
  P[0] = P[0] + Q * dt;
  P[2] = P[2] + Q * dt;

  kalGain[0] = P[0] / (P[0] + R);
  kalGain[1] = P[1] / (P[1] + R);

  roll = roll +  (accCal[0] - roll) * kalGain[0];
  pitch = pitch + (accCal[1] - pitch) * kalGain[0];

  P[0] = P[0] * (1 - kalGain[0]);
  P[1] = P[1] * (1 - kalGain[1]);
  P[2] = P[2] - kalGain[1] * P[1];

  t = millis();
  timer = millis() - timer;
  timer = (dt * 1000) - timer;
 
}
