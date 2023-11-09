//#include <i2c_t3.h>
#include "Arduino.h"
#include "Wire.h"
#include "LSM6DSL.h"
#include "LIS3MDL.h"
#include "BLE.h"

#define BAUDRATE 115200
#define DT  0.02          // Loop time E.g 0.02 = 20 milliseconds
#define AA  0.97         // complementary filter constant
#define G_GAIN 0.070    // [deg/s/LSB]

/* Gyro calibration */
#define magXmax 1893
#define magYmax 1023
#define magZmax 1050
#define magXmin -1407
#define magYmin -3101
#define magZmin -2174

byte buff[6];
int accRaw[3];
int magRaw[3];
int gyrRaw[3];
float rate_gyr_y = 0.0;     // [deg/s]
float rate_gyr_x = 0.0;     // [deg/s]
float rate_gyr_z = 0.0;     // [deg/s]
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroZangle = 0.0;
float AccYangle = 0.0;
float AccXangle = 0.0;
float AccZangle = 0.0;
float CFangleX = 0.0;
float CFangleY = 0.0;
float CFangleZ = 0.0;
float heading = 0.0;
float dummy = 0.0;
float dummy_vel = 0.0;

struct Packet pkt_payload;

//Used by Kalman Filters
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0;
float y_bias = 0;
float XP_00 = 0, XP_01 = 0, XP_10 = 0, XP_11 = 0;
float YP_00 = 0, YP_01 = 0, YP_10 = 0, YP_11 = 0;
float KFangleX = 0.0;
float KFangleY = 0.0;



//For Time stamp
long startTime = 0;
long elapsed_time = 0;
double time_stamp = 0.0;

float kalmanFilterX(float accAngle, float gyroRate)
{
	float  y, S;
	float K_0, K_1;


	KFangleX += DT * (gyroRate - x_bias);

	XP_00 +=  - DT * (XP_10 + XP_01) + Q_angle * DT;
	XP_01 +=  - DT * XP_11;
	XP_10 +=  - DT * XP_11;
	XP_11 +=  + Q_gyro * DT;

	y = accAngle - KFangleX;
	S = XP_00 + R_angle;
	K_0 = XP_00 / S;
	K_1 = XP_10 / S;

	KFangleX +=  K_0 * y;
	x_bias  +=  K_1 * y;
	XP_00 -= K_0 * XP_00;
	XP_01 -= K_0 * XP_01;
	XP_10 -= K_1 * XP_00;
	XP_11 -= K_1 * XP_01;

	return KFangleX;
}

float kalmanFilterY(float accAngle, float gyroRate)
{
	float  y, S;
	float K_0, K_1;


	KFangleY += DT * (gyroRate - y_bias);

	YP_00 +=  - DT * (YP_10 + YP_01) + Q_angle * DT;
	YP_01 +=  - DT * YP_11;
	YP_10 +=  - DT * YP_11;
	YP_11 +=  + Q_gyro * DT;

	y = accAngle - KFangleY;
	S = YP_00 + R_angle;
	K_0 = YP_00 / S;
	K_1 = YP_10 / S;

	KFangleY +=  K_0 * y;
	y_bias  +=  K_1 * y;
	YP_00 -= K_0 * YP_00;
	YP_01 -= K_0 * YP_01;
	YP_10 -= K_1 * YP_00;
	YP_11 -= K_1 * YP_01;

	return KFangleY;
}

void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device (initiate again)
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}

void setup_berryIMU() 
{
  Wire.begin(); //Init i2c
  Wire.setClock(400000); //Change i2c bus speed to 400k Hz
  Serial.begin(BAUDRATE); //set baudrate to 115200
}

void enable_acclerometer()
{
        //initialise the accelerometer
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL1_XL,0b10011111);        // ODR 3.33 kHz, +/- 8g , BW = 400hz
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL8_XL,0b11001000);        // Low pass filter enabled, BW9, composite filter
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL3_C,0b01000100);         // Enable Block Data update, increment during multi byte read
}

void enable_magnetometer()
{
  //initialise the magnetometer
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG1, 0b11011100);      // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled.
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG2, 0b00100000);      // +/- 8 gauss
  writeTo(LIS3MDL_ADDRESS,LIS3MDL_CTRL_REG3, 0b00000000);      // Continuous-conversion mode
}

void enable_gyroscope()
{
        //initialise the gyroscope
        writeTo(LSM6DSL_ADDRESS,LSM6DSL_CTRL2_G,0b10011100);         // ODR 3.3 kHz, 2000 dps
}

void berryIMU_measure()
{
  startTime = millis();
  //Read the measurements from the sensors, combine and convert to correct values
  //The values are expressed in 2’s complement (MSB for the sign and then 15 bits for the value) 
  //Start at OUT_X_L_A and read 6 bytes.
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_XL, 6, buff);
  accRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  accRaw[1] = (int)(buff[2] | (buff[3] << 8));
  accRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (accRaw[0] >= 32768) accRaw[0] = accRaw[0] - 65536;
  if (accRaw[1] >= 32768) accRaw[1] = accRaw[1] - 65536;
  if (accRaw[2] >= 32768) accRaw[2] = accRaw[2] - 65536;

  //readMAG(buff);
  readFrom(LIS3MDL_ADDRESS, 0x80 | LIS3MDL_OUT_X_L, 6, buff);
  magRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  magRaw[1] = (int)(buff[2] | (buff[3] << 8));
  magRaw[2] = (int)(buff[4] | (buff[5] << 8));
  
  /*
  magRaw[0] -= (magXmin + magXmax) /2 ;
  magRaw[1] -= (magYmin + magYmax) /2 ;
  magRaw[2] -= (magZmin + magZmax) /2 ; */
  
  readFrom(LSM6DSL_ADDRESS, LSM6DSL_OUT_X_L_G, 6, buff);
  gyrRaw[0] = (int)(buff[0] | (buff[1] << 8));   
  gyrRaw[1] = (int)(buff[2] | (buff[3] << 8));
  gyrRaw[2] = (int)(buff[4] | (buff[5] << 8));
  if (gyrRaw[0] >= 32768) gyrRaw[0] = gyrRaw[0]- 65536;
  if (gyrRaw[1] >= 32768) gyrRaw[1] = gyrRaw[1]- 65536;
  if (gyrRaw[2] >= 32768) gyrRaw[2] = gyrRaw[2]- 65536;

  //Convert Gyro raw to degrees per second
  rate_gyr_x = (float) gyrRaw[0] * G_GAIN;
  rate_gyr_y = (float) gyrRaw[1]  * G_GAIN;
  rate_gyr_z = (float) gyrRaw[2]  * G_GAIN;

  //Calculate the angles from the gyro
  gyroXangle+=rate_gyr_x*DT;
  gyroYangle+=rate_gyr_y*DT;
  gyroZangle+=rate_gyr_z*DT;

    //Convert Accelerometer values to degrees
  AccXangle = (float) (atan2(accRaw[1],accRaw[2])+M_PI)*RAD_TO_DEG;
  AccYangle = (float) (atan2(accRaw[2],accRaw[0])+M_PI)*RAD_TO_DEG;
  //AccZangle = (float) (asin(accRaw[2] / (sqrt(pow(accRaw[0], 2) + pow(accRaw[1], 2) + pow(accRaw[2], 2)) ))) * RAD_TO_DEG;

  //If IMU is up the correct way, use these lines
  AccZangle -= (float) 90.0;
  AccXangle -= (float)180.0;
  if (AccYangle > 90)
    AccYangle -= (float)270;
  else
    AccYangle += (float)90;

  //Complementary filter used to combine the accelerometer and gyro values.
  CFangleX=AA*(CFangleX+rate_gyr_x*DT) +(1 - AA) * AccXangle;
  CFangleY=AA*(CFangleY+rate_gyr_y*DT) +(1 - AA) * AccYangle;
  //CFangleZ = (AA*(CFangleZ+rate_gyr_z*DT) +(1 - AA) * AccZangle);
  
  // Kalman
  CFangleX= kalmanFilterX(AccXangle, rate_gyr_x);
  CFangleY= kalmanFilterY(AccYangle, rate_gyr_y);
  dummy = sqrt(CFangleX*CFangleX + CFangleY*CFangleY);
  dummy_vel = sqrt(rate_gyr_x*rate_gyr_x + rate_gyr_y*rate_gyr_y);
  //Tilt Compensation

  //Compute heading  
  heading = 180 * atan2(magRaw[1],magRaw[0])/M_PI;
  //heading = (int) 180*atan2(magYcomp,magXcomp)/M_PI;

  //Convert heading to 0 - 360
  if(heading < 0)
  {
    heading += 360;
  }
            

  //Each loop should be at least DT
  while(millis() - startTime < (DT*1000))
  {
    delay(1);
  }
  
  elapsed_time = millis() - startTime;
  //Serial.print(elapsed_time);
  //time_stamp = (double) elapsed_time / 1000.0;

  pkt_payload.CFangle_data = dummy;
  pkt_payload.gyrovel_data = dummy_vel;
  
  /*
  if ((heading <= 45) || (heading >= 315) || ((heading >= 135) && (heading <= 225)))
  {
    pkt_payload.CFangle_data = CFangleX;
    pkt_payload.gyrovel_data = rate_gyr_x;
  }
  else
  {
    pkt_payload.CFangle_data = CFangleY;
    pkt_payload.gyrovel_data = rate_gyr_y;
  }
  */

}

float getAccXangle()
{
  return AccXangle;
}

float getAccYangle()
{
  return AccYangle;
}

float getGyroXangle()
{
  return gyroXangle;
}

float getGyroYangle()
{
  return gyroYangle;
}

float getGyroZangle()
{
  return gyroZangle;
}

void dbg_print()
{
  //Serial.print("Time Stamp:\t");
  //Serial.print(pkt_payload.time_stamp);
  Serial.print("AccX\t");
  Serial.print(AccXangle);
  Serial.print("\t#  AccY\t");
  Serial.print(AccYangle);
  Serial.print("\t#  AccZ\t");
  Serial.print(AccZangle);
  Serial.print("\t#  GyrX\t");
  Serial.print(gyroXangle);
  Serial.print("\t#  GyrY\t");
  Serial.print(gyroYangle);
  Serial.print("\t#  GyrZ\t");
  Serial.print(gyroZangle);
  Serial.print("\t# CFangleX\t");
  Serial.print(CFangleX);
  Serial.print("\t# CFangleY\t");
  Serial.print(CFangleY);
  //Serial.print("\t# CFangle\t");
  //Serial.print(pkt_payload.CFangle_data); 
  /*
  Serial.print("\t# AccXNorm\t");
  Serial.print(accXnorm);
  Serial.print("\t# AccYNorm\t");
  Serial.print(accYnorm); 
  Serial.print("\t# Pitch\t");
  Serial.print(pitch);  
  Serial.print("\t# Roll\t");
  Serial.print(roll);  
  Serial.print("\t# MagXComp\t");
  Serial.print(magXcomp);  
  Serial.print("\t# MagYComp\t");
  Serial.print(magYcomp);  
  */
  Serial.print("\t# Heading\t ");
  Serial.println(heading);
}

void setup() 
{
  // put your setup code here, to run once:
  setup_berryIMU();
  enable_acclerometer();
  enable_magnetometer();
  enable_gyroscope();
  blePayload_setup();
}

void loop() 
{
  // put your main code here, to run repeatedly:
  Serial.print("Starting \n");
  berryIMU_measure();
  //delay(3000);
  dbg_print();
  ble_transmit(pkt_payload);
}
