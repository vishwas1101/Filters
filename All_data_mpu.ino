#include<Wire.h>
#define RAD2DEG (180/3.14)

//Array index number 0,1,2 corresponds to X,Y,Z axes
float accRaw[3], accAngle[3], accError[3];
float gyrRaw[3], gyrAngle[3], gyrError[3];
float yaw, pitch, roll;
float time, newtime, prevtime; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Begin wire communication with mpu6050
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
  
  time = millis();

  //Calculating error of gyroscope and accelerometer
  //take average of 200 values as the error 

  //getting accelerometer error 
  for(int i = 0; i<200; i++){
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);                       
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);

    accRaw[0] = (Wire.read()<<8|Wire.read())/4096.0; 
    accRaw[1] = (Wire.read()<<8|Wire.read())/4096.0;
    accRaw[2] = (Wire.read()<<8|Wire.read())/4096.0;

    accError[0] = accError[0] + ((atan((accRaw[1])/sqrt(pow((accRaw[0]),2) + pow((accRaw[2]),2)))*RAD2DEG));
    accError[1] = accError[1] + ((atan(-1*(accRaw[0])/sqrt(pow((accRaw[1]),2) + pow((accRaw[2]),2)))*RAD2DEG));
    accError[2] = accError[2] + ((atan((accRaw[2])/sqrt(pow((accRaw[1]),2) + pow((accRaw[0]),2)))*RAD2DEG));
   }
  for(int i = 0; i<3; i++){
    accError[i] = accError[i]/200.0;
   }

  //getting gyroscope error
  for(int i = 0; i<200; i++){
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
    gyrError[i] = gyrError[i]/200.0;
  }  
}

void loop() {
  // put your main code here, to run repeatedly:
  prevtime = time; 
  time = millis();
  newtime = (time - prevtime)/1000;

  //Getting acceleromter values and angles from them 
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  accRaw[0] = (Wire.read()<<8|Wire.read())/4096.0;
  accRaw[1] = (Wire.read()<<8|Wire.read())/4096.0;
  accRaw[2] = (Wire.read()<<8|Wire.read())/4096.0;

  //inorder to get angles from the raw data, we use:
  //Eulers formula 
  accAngle[0] = (atan((accRaw[1])/sqrt(pow((accRaw[0]),2) + pow((accRaw[2]),2)))*RAD2DEG) - accError[0];
  accAngle[1] = (atan(-1*(accRaw[0])/sqrt(pow((accRaw[1]),2) + pow((accRaw[2]),2)))*RAD2DEG) - accError[1];    
  accAngle[2] = (atan((accRaw[2])/sqrt(pow((accRaw[0]),2) + pow((accRaw[1]),2)))*RAD2DEG) - accError[2];
  
  
  //Getting gyroscope values and angles from them
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  gyrRaw[0] = Wire.read()<<8|Wire.read();
  gyrRaw[1] = Wire.read()<<8|Wire.read();
  gyrRaw[2] = Wire.read()<<8|Wire.read();

  for(int i = 0; i<3; i++){
  gyrRaw[i] = (gyrRaw[i]/32.8) - gyrError[i];
  gyrAngle[i] = gyrRaw[i]*newtime; 
  }

  roll = 0.98 * (roll + gyrAngle[0]) + 0.02*accAngle[0];
  pitch = 0.98 * (pitch + gyrAngle[1]) + 0.02*accAngle[1];
  yaw = 0.98 * (yaw + gyrAngle[2]) + 0.02*accAngle[2];

 Serial.print("Xº: ");
 Serial.print(roll);
 Serial.print("   |   ");
 Serial.print("Yº: ");
 Serial.print(pitch);
 Serial.print("   |   ");
 Serial.print("Zº: ");
 Serial.print(yaw);
 Serial.println(" ");

}
