#include<Wire.h>

//Array index number 0,1,2 corresponds to X,Y,Z axes
float accRaw[3]; 
float gyrRaw[3];

void setup(){
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
 
}

void loop() {
  // put your main code here, to run repeatedly:

  //Getting accelerometer values
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);

  accRaw[0] = (Wire.read()<<8|Wire.read())/4096.0;
  accRaw[1] = (Wire.read()<<8|Wire.read())/4096.0;
  accRaw[2] = (Wire.read()<<8|Wire.read())/4096.0;
  
  
  //Getting gyroscope values 
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  gyrRaw[0] = Wire.read()<<8|Wire.read();
  gyrRaw[1] = Wire.read()<<8|Wire.read();
  gyrRaw[2] = Wire.read()<<8|Wire.read();

 Serial.print("Acc X: ");
 Serial.print(accRaw[0]);
 Serial.print("   |   ");
 Serial.print("Acc Y: ");
 Serial.print(accRaw[1]);
 Serial.print("Acc Z ");
 Serial.print(accRaw[2]);
 Serial.print("   |   ");
 Serial.print("Gyr X: ");
 Serial.print(gyrRaw[0]);
 Serial.print("   |   ");
 Serial.print("gyr Y: ");
 Serial.print(gyrRaw[1]);
 Serial.print("   |   ");
 Serial.print("gyr Z ");
 Serial.print(gyrRaw[2]);
 Serial.println(";");
}
