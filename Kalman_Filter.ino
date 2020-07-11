#include <Wire.h>
#define RAD2DEG 180/PI
 
float accRaw[3], gyrRaw[3];
float accError[3], gyrError[3];
float accAngle[3], gyrRate[2];
float roll = 0, pitch = 0;

float Qangle =  0.001;
float Qbias = 0.003;
float R = 0.03;

float angle = 0;
float bias = 0;

float P[2][2] = {0,0,0,0};

float rate, dt;
float time, prevtime;

float S, K[2], y;  

float kalAngle(float newAngle, float rateGyr, float dt){
  rate = rateGyr - bias;
  angle = angle + rate * dt; 

  P[0][0] = P[0][0] + dt * (dt * P[1][1] - P[0][1] - P[1][0] + Qangle);
  P[0][1] = P[0][1] - dt * P[1][1];
  P[1][0] = P[1][0] - dt * P[1][1];
  P[1][1] = P[1][1] + dt * Qbias;

  S = P[0][0] + R;

  K[0] = P[0][0]/S;
  K[1] = P[1][0]/S;

  y = newAngle - angle;

  angle = angle + K[0]*y;
  bias = bias + K[1]*y;

  P[0][0] = P[0][0] - K[0] * P[0][0];
  P[0][1] = P[0][1] - K[0] * P[0][1];
  P[1][0] = P[0][0] - K[1] * P[0][0];
  P[1][1] = P[1][1] - K[1] * P[0][1];

  return angle;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
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
  time  = millis(); 
  
  //getting accelerometer error 
  for(int i = 0; i<500; i++){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                       
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    accRaw[0] = (Wire.read()<<8|Wire.read()); 
    accRaw[1] = (Wire.read()<<8|Wire.read());
    accRaw[2] = (Wire.read()<<8|Wire.read());

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

    gyrError[0] = gyrError[0] + (gyrRaw[0]/131.0);
    gyrError[1] = gyrError[1] + (gyrRaw[1]/131.0);
    gyrError[2] = gyrError[2] + (gyrRaw[2]/131.0); 
  }
  for(int i = 0; i<3; i++){
    gyrError[i] = gyrError[i]/500.0;
  }
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
  dt = (millis() - time)/1000;
  time = millis();
  
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

  accAngle[0] = atan((accRaw[1])/sqrt(pow((accRaw[0]),2) + pow((accRaw[2]),2)))*RAD2DEG - accError[0];
  accAngle[1] = atan(-1*(accRaw[0])/sqrt(pow((accRaw[1]),2) + pow((accRaw[2]),2)))*RAD2DEG - accError[1];

  gyrRate[0] = gyrRaw[0]/131.0 - gyrError[0];
  gyrRate[1] = gyrRaw[1]/131.0 - gyrError[1];

  //calculating pitch and roll
  roll = kalAngle(accAngle[0], gyrRate[0], dt);
  pitch = kalAngle(accAngle[1], gyrRate[1], dt);
  
  Serial.print("pitch: ");
  Serial.print(pitch);
  Serial.print("   roll: ");
  Serial.println(roll);

}
