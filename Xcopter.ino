// 8:50 am 1 apr'18
#include <SFE_BMP180.h>
#include <Wire.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

unsigned long long int botinitialized = 0;

SFE_BMP180 bmp;
RF24 radio(4, 10);
MPU6050 mpu;

#define mpu_int 2
#define echo    3
#define trig    7
#define ultratrim 0.11

// received data from nrf
int datasize = 13;
int8_t data[13];
const uint64_t pipe = 0xE8E8F0F0E1LL;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
// below variables are in local frame
int16_t gyro[3];      // raw gyroscope data
int16_t acce[3];      // raw acceleration data
float accescaled[3];  // acceleration scaled for -9.807 to +9.807  specifically for all directions in a linear relation
volatile bool mpuInterrupt = false;


// Interrupt functions
void dmpDataReady()
{
  mpuInterrupt = true;
}
float ultraheight;
unsigned long int ultralast;
unsigned long int ultraheightlast;
double zcos; // zcos is cosine of angle between local z axis and global z axis
void signalgot()
{
#define lpultra 0.9
  float templocal = ( (micros() - ultralast) * 0.00017025 ) - ultratrim ;
  if ( botinitialized <= 2000 )
  {
    ultraheight = 0;
  }
  else
  {
    ultraheight = ultraheight * lpultra + templocal * ( 1 - lpultra );
  }
  ultraheightlast = micros();
}

// four BLDC motors
Servo MotorLeftFront;
Servo MotorRightFront;
Servo MotorLeftBack;
Servo MotorRightBack;

// the A0,A1,A2 pins on board show the status of the FC
void writestatus(int a)
{
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  digitalWrite(A0, a & 4);
  digitalWrite(A1, a & 2);
  digitalWrite(A2, a & 1);
}

// Calling this function will send signal to update ultrasonic distance height
void updateultraheight()
{
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  ultralast = micros();
}


void setup()
{
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(2000000);
  while (!Serial);

  // initialize device
  mpu.initialize();
  pinMode(mpu_int, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-2101);
  mpu.setYAccelOffset(-1079);
  mpu.setZAccelOffset(1501);
  mpu.setXGyroOffset(70);
  mpu.setYGyroOffset(-6);
  mpu.setZGyroOffset(2);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(mpu_int) , dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output

  radio.begin();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  while (!dmpReady) {}

  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;
  data[8] = 0;
  data[9] = 0;
  data[10] = 0;
  data[11] = 0;
  data[12] = 0;

  MotorLeftBack.attach(5);
  MotorRightBack.attach(6);
  MotorRightFront.attach(8);
  MotorLeftFront.attach(9);

  if (!bmp.begin())
  {
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    writestatus(6);
  }

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  attachInterrupt(1, signalgot, FALLING);


  MotorLeftBack  .writeMicroseconds( 900 );
  MotorLeftFront .writeMicroseconds( 900 );
  MotorRightBack .writeMicroseconds( 900 );
  MotorRightFront.writeMicroseconds( 900 );

  delay(3000);

  MotorLeftBack  .writeMicroseconds( 900 );
  MotorLeftFront .writeMicroseconds( 900 );
  MotorRightBack .writeMicroseconds( 900 );
  MotorRightFront.writeMicroseconds( 900 );

  delay(5000);

  initKPKIKD();
  botinitialized = micros();
}

/* PID Variables

*/

// bmp 180 barometer variables
double GroundPressure = 0;
float baroheight = 0;
unsigned long int baroheightlast = 0;
unsigned char bmpstate = 0;
unsigned long int bmpneed = 500;
double Temperature;
double Pressure;
double PressureAveraged;

// yaw   represents the angle in degrees between global xz plane and local xaxis , when rotated about local -z axis
// pitch represents the angle of rotation required about local -y axis to make local x axis parallel to global xy plane
// roll  represents the angle of rotation required about local +x axis to make local y axis parallel to global xy plane
float ypr[3];

// yprRate is same is gyro except the 0 error removed >> in my case the error was always  1 or -1
float yprRate[3];

// acceleration in global z axis scaled for m/s^2
float accez;

float YawRatePID;
float PitchRatePID;
float RollRatePID;

float YawRateP, YawRateI, YawRateD;
float PitchRateP, PitchRateI, PitchRateD;
float RollRateP, RollRateI, RollRateD;

float YawError, PitchError, RollError;
float YawRateError, PitchRateError, RollRateError;
float YawRateErrorForDerivative, PitchRateErrorForDerivative, RollRateErrorForDerivative;
float YawRateErrorForDerivativeOld, PitchRateErrorForDerivativeOld, RollRateErrorForDerivativeOld;
float YawRateErrorTotal, PitchRateErrorTotal, RollRateErrorTotal;

// #define TuningOff
// #define TuningRadioAttitude
// #define TuningRadioAltitude
// #define printMPUDMPdata

// manipulate
float KYawRateP = 3.0, KYawRateI = 2.6, KYawRateD = 0.04, KYawP = 2.0;
float KPitchRateP = 1.7, KPitchRateI = 2.0, KPitchRateD = 0.03, KPitchP = 2.5;
float KRollRateP = 1.7, KRollRateI = 2.0, KRollRateD = 0.03, KRollP = 2.5;

/*
  float KYawRateP=3.0,KYawRateI=3.0,KYawRateD=0.04,KYawP=2.0;
  float KPitchRateP=1.5,KPitchRateI=2.3,KPitchRateD=0.025,KPitchP=2.5;
  float KRollRateP=1.5,KRollRateI=2.3,KRollRateD=0.025,KRollP=2.5;

  float KYawRateP=2.6,KYawRateI=2.6,KYawRateD=0.04,KYawP=1.8;
  float KPitchRateP=1.3,KPitchRateI=2.0,KPitchRateD=0.025,KPitchP=2.2;
  float KRollRateP=1.3,KRollRateI=2.0,KRollRateD=0.025,KRollP=2.2;
*/

float Throttle = 0;
float YawReq = 0, YawRateReq   = 0;
float PitchReq = 0, PitchRateReq = 0;
float RollReq = 0, RollRateReq  = 0;

float LeftFrontMotor = 900, RightFrontMotor = 900, LeftBackMotor = 900, RightBackMotor = 900;

#define PitchTrim -0.1
#define RollTrim  -0.1

float dt;
int pr = 0;

// Altitude PID variables and prameters below

// acceleration , velocity and Altitude estimated values as given by the Kalman filter, the function written at last
float azreal;
float vzreal;
float Altit;

float vzrealReq;

float KvzrealP = 200.0, KvzrealI = 100.0, KvzrealD = 15.0;

float vzrealP = 0;
float vzrealI = 0;
float vzrealD = 0;

float vzrealPID = 0;

float vzrealError = 0;
float vzrealErrorTotal = 0;

float DifferentialThrottle = 0;

// Altitude PID variables and prameters above


unsigned long int pidlooplast = 0;
unsigned long int radiolast = 0;
unsigned long int bmplast = 0;

void initKPKIKD()
{
#ifdef TuningRadioAttitude
  KYawRateP = 0; KYawRateI = 0; KYawRateD = 0; KYawP
  KPitchRateP = 0; KPitchRateI = 0; KPitchRateD = 0; KPitchP = 0;
  KRollRateP = 0; KRollRateI = 0; KRollRateD = 0; KRollP = 0;
#endif
#ifdef TuningRadioAltitude
  KvzrealP = 0; KvzrealI = 0; KvzrealD = 0;
  KYawP = 0; KYawRateP = 0; KYawRateI = 0; KYawRateD = 0;
#endif
}

// convert quaternion to horizon based absolute yaw, pitch and roll
void gethorizonypr(Quaternion q)
{
  // direction cosines for local x axis after rotation by q quaternion given by dmp of mpu6050
  float newix, newiy, newiz;
  newix = (q.w * q.w) + (q.x * q.x) - (q.y * q.y) - (q.z * q.z);
  newiy =  2 * ( ( q.x * q.y ) + ( q.w * q.z ) ) ;
  newiz =  2 * ( ( q.x * q.z ) - ( q.w * q.y ) );

  // direction cosines for local y axis after rotation by q quaternion given by dmp of mpu6050
  float newjx, newjy, newjz;
  newjx =  2 * ( ( q.x * q.y ) - ( q.w * q.z ) ) ;
  newjy =  (q.w * q.w) - (q.x * q.x) + (q.y * q.y) - (q.z * q.z);
  newjz =  2 * ( ( q.y * q.z ) + ( q.w * q.x ) );

  // direction cosines for local z axis after rotation by q quaternion given by dmp of mpu6050
  float newkx, newky, newkz;
  newkx =  2 * ( ( q.x * q.z ) + ( q.w * q.y ) );
  newky =  2 * ( ( q.y * q.z ) - ( q.w * q.x ) ) ;
  newkz =  (q.w * q.w) - (q.x * q.x) - (q.y * q.y) + (q.z * q.z);

  zcos = newkz > 0 ? newkz : -newkz;
  if ( zcos > 1 ) {
    zcos = 1;
  } else if ( zcos < 0 ) {
    zcos = 0;
  }


  float rx, ry, rz;

  //for correctpitch
  rz = 0;
  if ( newjx != 0 )
  {
    ry = 1 / sqrt( 1 + ( ( newjy * newjy ) / ( newjx * newjx ) ) );
    rx = ( -1 * ry * newjy ) / newjx;
  }
  else
  {
    ry = 0;
    rx = 1;
  }
  ypr[1] = (180 / PI) * atan( ( sqrt( ( newiz * newiz ) + ( rx * newiy - ry * newix ) * ( rx * newiy - ry * newix ) ) ) / ( newix * rx + newiy * ry ) );
  if ( ypr[1] * newiz > 0 )
  {
    ypr[1] *= -1;
  }

  //for correctroll
  rz = 0;
  if ( newix != 0 )
  {
    ry = 1 / sqrt( 1 + ( ( newiy * newiy ) / ( newix * newix ) ) );
    rx = ( -1 * ry * newiy ) / newix;
  }
  else
  {
    ry = 0;
    rx = 1;
  }
  ypr[2] = (180 / PI) * atan( ( sqrt( ( newjz * newjz ) + ( rx * newjy - ry * newjx ) * ( rx * newjy - ry * newjx ) ) ) / ( newjx * rx + newjy * ry ) );
  if ( ypr[2] * newjz < 0 )
  {
    ypr[2] *= -1;
  }

  //for correctyaw
  ry = 0;
  if ( newkx != 0 )
  {
    rz = 1 / sqrt( 1 + ( ( newkz * newkz ) / ( newkx * newkx ) ) );
    rx = ( -1 * rz * newkz ) / newkx;
    if ( rx < 0 )
    {
      rx = -1 * rx;
      rz = -1 * rz;
    }
  }
  else
  {
    rz = 0;
    rx = 1;
  }
  float temp =  newix * rx + newiz * rz ;
  if ( temp > 1)
  {
    temp = 0.9999;
  }
  else if ( temp < -1 )
  {
    temp = -0.9999;
  }
  ypr[0] = (180 / PI) * acos( temp );
  if ( newiy > 0 )
  {
    ypr[0] *= -1;
  }

  // below linearized model is specific to different accelerometers
  /*
     tilt each axis to make them point ground, all axis ( +x,-x,+y,+z,-y,-z ) will give different 9.8 m/s2
     so linearize them or use the below linearized values at your own risk
  */

  accescaled[0] = acce[0] * 0.00120165 + 0.46263381;
  accescaled[1] = acce[1] * 0.00119142 + 0.23828575;
  accescaled[2] = acce[2] * 0.00119864 + 0.00898978;

  accez =   ( newiz * accescaled[0] )   +  ( newjz * accescaled[1] )   +   ( newkz * accescaled[2] )   ;
  accez = accez - 9.80665;
}

unsigned char turnoff = 0;
float temp = 0;

// Kalman fikter function for Altit, vzreal and azreal estimation
void updateAltitude();   // called after getting barometer data
void predictAltitude();  // called after getting acceleration data

float maximum4( float f1 , float f2 , float f3 , float f4 )
{
  f1 = ( ( f1 > f3 ) ? f1 : f3 );
  f2 = ( ( f2 > f4 ) ? f2 : f4 );
  return ( ( f1 > f2 ) ? f1 : f2 ) ;
}


// Yes I copied this comment, sue me
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop() {

  if ( mpuInterrupt )
  {
    writestatus(1);

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    if ( mpuIntStatus & 0x10 )
    {
      mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02)
    {
      fifoCount = mpu.getFIFOCount();

      while (fifoCount >= packetSize)
      {
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
      }

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGyro (gyro, fifoBuffer);
      mpu.dmpGetAccel(acce, fifoBuffer);
      gethorizonypr(q);

      yprRate[0] = -gyro[2];
      yprRate[1] = gyro[1];
      yprRate[2] = gyro[0];

      if ( yprRate[0] <= 1 && yprRate[0] >= -1 ) {
        yprRate[0] = 0;
      }
      if ( yprRate[1] <= 1 && yprRate[1] >= -1 ) {
        yprRate[1] = 0;
      }
      if ( yprRate[2] <= 1 && yprRate[2] >= -1 ) {
        yprRate[2] = 0;
      }

      predictAltitude();

#ifdef printMPUDMPdata
      Serial.print(ypr[0]);
      Serial.print(" \t ");
      Serial.print(ypr[1]);
      Serial.print(" \t ");
      Serial.print(ypr[2]);
      Serial.print(" \t ");
      Serial.print(yprRate[0]);
      Serial.print(" \t ");
      Serial.print(yprRate[1]);
      Serial.print(" \t ");
      Serial.print(yprRate[2]);
      Serial.print(" \t ");
      Serial.print(acce[0]);
      Serial.print(" \t ");
      Serial.print(acce[1]);
      Serial.print(" \t ");
      Serial.print(acce[2]);
      Serial.print(" \t ");
      Serial.print(accescaled[0]);
      Serial.print(" \t ");
      Serial.print(accescaled[1]);
      Serial.print(" \t ");
      Serial.print(accescaled[2]);
      Serial.print(" \t ");
      Serial.print(Altit);
      Serial.print(" \t ");
      Serial.print(vzreal);
      Serial.print(" \t ");
      Serial.println(azreal);
#endif
    }

    writestatus(4);

    if ( turnoff == 0 )
    {
      Throttle   = ( ( ((float)(data[1] & 0x00ff)) * 4.3 ) * 0.7 ) + 900 ; // at hover Throttle requirement is 550

      if ( data[6] > 5.0 )
      {
        vzrealReq  = ((data[6] - 5.0) / 25.0) * 0.7;
      }
      else if ( data[6] < -5.0 )
      {
        vzrealReq  = ((data[6] + 5.0) / 25.0) * 0.7;
      }
      else
      {
        vzrealReq = 0;
      }
      
      if ( data[5] > 5.0 )
      {
        YawReq    +=  ((data[5] - 5.0) / 45.0) ;
      }
      else if ( data[5] < -5.0 )
      {
        YawReq  += ((data[5] + 5.0) / 45.0) ;
      }
      PitchReq   =  ((data[3] * 2.5) / 3.0)  + PitchTrim ;
      RollReq    =  ((data[2] * 2.5) / 3.0)  + RollTrim  ;

      if ( YawReq >= 180 ) {
        YawReq = YawReq - 360;
      }
      else if ( YawReq <= -180 ) {
        YawReq = YawReq + 360;
      }
    }
    else
    {
      Throttle = Throttle - 1;
      YawReq = YawReq;
      PitchReq = 0;
      RollReq = 0;
      vzrealReq = 0;
    }

    while ( micros() - pidlooplast < 10000 ) {}

    dt = ((float)(micros() - pidlooplast)) / 1000000;
    pidlooplast = micros();

    //Serial.print("dt = ");Serial.println(dt*1000,5);
    dt = 0.01;

    if ( Throttle > 970 )
    {
      YawError   = YawReq   - ypr[0];
      if ( YawError > 180 ) {
        YawError = YawError - 360;
      }
      else if ( YawError < -180 ) {
        YawError = YawError + 360;
      }
      PitchError = PitchReq - ypr[1];
      RollError  = RollReq  - ypr[2];
      
      if ( vzrealReq > 0.8 ) 
      {     
        vzrealReq = 0.8;
      } 
      else if ( vzrealReq < -0.8 )
      {
        vzrealReq = -0.8;
      }
      YawRateReq   = KYawP   * YawError   ;
      PitchRateReq = KPitchP * PitchError ;
      RollRateReq  = KRollP  * RollError  ;

      vzrealError    = + vzreal     - vzrealReq    ;
      YawRateError   = + yprRate[0] - YawRateReq   ;
      PitchRateError = + yprRate[1] - PitchRateReq ;
      RollRateError  = + yprRate[2] - RollRateReq  ;

      vzrealP    =   vzrealError    *  KvzrealP    ;
      YawRateP   =   YawRateError   *  KYawRateP   ;
      PitchRateP =   PitchRateError *  KPitchRateP ;
      RollRateP  =   RollRateError  *  KRollRateP  ;

      vzrealErrorTotal += (vzrealError);
      vzrealI = vzrealErrorTotal * KvzrealI * dt;

      YawRateErrorTotal += (YawRateError);
      YawRateI = YawRateErrorTotal * KYawRateI * dt;

      PitchRateErrorTotal += (PitchRateError);
      PitchRateI = PitchRateErrorTotal * KPitchRateI * dt;

      RollRateErrorTotal += (RollRateError);
      RollRateI = RollRateErrorTotal * KRollRateI * dt;

      // let us not involve the set point in derivative to avoid spikes
      YawRateErrorForDerivative   = ( + yprRate[0] /* - YawRateReq   */ );
      PitchRateErrorForDerivative = ( + yprRate[1] /* - PitchRateReq */ );
      RollRateErrorForDerivative  = ( + yprRate[2] /* - RollRateReq  */ );

      vzrealD     = azreal * KvzrealD;
      YawRateD    = ( ( YawRateErrorForDerivative   - YawRateErrorForDerivativeOld   ) / dt ) * KYawRateD   ;
      PitchRateD  = ( ( PitchRateErrorForDerivative - PitchRateErrorForDerivativeOld ) / dt ) * KPitchRateD ;
      RollRateD   = ( ( RollRateErrorForDerivative  - RollRateErrorForDerivativeOld  ) / dt ) * KRollRateD  ;

      YawRateErrorForDerivativeOld    =  YawRateErrorForDerivative   ;
      PitchRateErrorForDerivativeOld  =  PitchRateErrorForDerivative ;
      RollRateErrorForDerivativeOld   =  RollRateErrorForDerivative  ;

      DifferentialThrottle =  vzrealP + vzrealI + vzrealD ;

      YawRatePID    =  YawRateP    +  YawRateI    +  YawRateD    ;
      PitchRatePID  =  PitchRateP  +  PitchRateI  +  PitchRateD  ;
      RollRatePID   =  RollRateP   +  RollRateI   +  RollRateD   ;

      if ( DifferentialThrottle <= -400 || DifferentialThrottle >= 400 )
      {
        if ( DifferentialThrottle > 0 ) {
          DifferentialThrottle = 400;
        }
        else {
          DifferentialThrottle = -400;
        }
      }

      if ( YawRatePID <= -300 || YawRatePID >= 300 )
      {
        if ( YawRatePID > 0 ) {
          YawRatePID = 300;
        }
        else {
          YawRatePID = -300;
        }
      }

      if ( PitchRatePID <= -300 || PitchRatePID >= 300 )
      {
        if ( PitchRatePID > 0 ) {
          PitchRatePID = 300;
        }
        else {
          PitchRatePID = -300;
        }
      }

      if ( RollRatePID <= -300 || RollRatePID >= 300 )
      {
        if ( RollRatePID > 0 ) {
          RollRatePID = 300;
        }
        else {
          RollRatePID = -300;
        }
      }

      LeftFrontMotor  = Throttle + PitchRatePID - RollRatePID + YawRatePID - DifferentialThrottle;
      RightFrontMotor = Throttle + PitchRatePID + RollRatePID - YawRatePID - DifferentialThrottle;
      LeftBackMotor   = Throttle - PitchRatePID - RollRatePID - YawRatePID - DifferentialThrottle;
      RightBackMotor  = Throttle - PitchRatePID + RollRatePID + YawRatePID - DifferentialThrottle;

      float maxval = maximum4(LeftFrontMotor, RightFrontMotor, LeftBackMotor, RightBackMotor );

      if ( maxval > 1970 )
      {
        float difference = maxval - 1970;
        LeftFrontMotor  -= difference;
        RightFrontMotor -= difference;
        LeftBackMotor   -= difference;
        RightBackMotor  -= difference;
      }

    }
    else
    {
      YawError   = 0;
      PitchError = 0;
      RollError  = 0;

      vzrealError = 0; vzrealErrorTotal = 0;

      YawRateError   = 0; YawRateErrorForDerivative   = 0; YawRateErrorForDerivativeOld   = 0;
      PitchRateError = 0; PitchRateErrorForDerivative = 0; PitchRateErrorForDerivativeOld = 0;
      RollRateError  = 0; RollRateErrorForDerivative  = 0; RollRateErrorForDerivativeOld  = 0;

      YawRateErrorTotal  = 0;
      PitchRateErrorTotal = 0;
      RollRateErrorTotal = 0;

      YawRatePID   = 0;
      PitchRatePID = 0;
      RollRatePID  = 0;
      DifferentialThrottle = 0;

      LeftFrontMotor  = Throttle;
      RightFrontMotor = Throttle;
      LeftBackMotor   = Throttle;
      RightBackMotor  = Throttle;

      YawReq = ypr[0];
    }

    MotorLeftFront .writeMicroseconds ( LeftFrontMotor  );
    MotorRightFront.writeMicroseconds ( RightFrontMotor );
    MotorLeftBack  .writeMicroseconds ( LeftBackMotor   );
    MotorRightBack .writeMicroseconds ( RightBackMotor  );

    writestatus(0);
  }

  if ( micros() - ultralast >= 60000 )
  {
    updateultraheight();
  }



  if ( radio.available() )
  {
    writestatus(3);

    radiolast = micros();

    while (radio.available())
    {
      radio.read( data , datasize );
    }

    unsigned int a = (( (data[9] & 0x00ff)  << 8 ) | (data[10] & 0x00ff ) );
    unsigned int b = (( (data[11] & 0x00ff) << 8 ) | (data[12] & 0x00ff ) );
    float Temp = a + ( ((float)b) / 1000 ) ;

    switch ( data[8] )
    {
      case 0:
        {
          KYawRateP = 0; KYawRateI = 0; KYawRateD = 0;
          KPitchRateP = 0; KPitchRateI = 0; KPitchRateD = 0;
          KRollRateP = 0; KRollRateI = 0; KRollRateD = 0;
          break;
        }

      case 1 :
        {
          KYawRateP = Temp;
          break;
        }
      case 2 :
        {
          KYawRateI = Temp;
          break;
        }
      case 3 :
        {
          KYawRateD = Temp;
          break;
        }

      case 4 :
        {
          KPitchRateP = Temp;
          break;
        }
      case 5 :
        {
          KPitchRateI = Temp;
          break;
        }
      case 6 :
        {
          KPitchRateD = Temp;
          break;
        }

      case 7 :
        {
          KRollRateP = Temp;
          break;
        }
      case 8 :
        {
          KRollRateI = Temp;
          break;
        }
      case 9 :
        {
          KRollRateD = Temp;
          break;
        }

      case 10 :
        {
          KvzrealP = Temp;
          break;
        }
      case 11 :
        {
          KvzrealI = Temp;
          break;
        }
      case 12 :
        {
          KvzrealD = Temp;
          break;
        }

    }

    writestatus(0);
  }


  if ( millis() - bmplast >= bmpneed && bmpstate != -100 )
  {
    writestatus(2);
    switch ( bmpstate )
    {
      case 0 :
        {
          bmpneed = bmp.startTemperature();
          if ( bmpneed == 0 )
          {
            Serial.println("bmp Temperature start fail");
            writestatus(6);
            bmpstate = -100;
            bmpneed  = 500;
          }
          else {
            bmpstate = 1 ;
          }

          break;
        }

      case 1:
        {
          bmpneed = bmp.getTemperature(Temperature);
          if ( bmpneed != 0 )
          {
            bmpneed = bmp.startPressure(3);
            if ( bmpneed == 0 )
            {
              Serial.println("bmp Pressure start fail");
              bmpstate = -100;
              bmpneed  = 500;
              writestatus(6);
            }
            else {
              bmpstate = 2;
            }
          }
          else
          {
            Serial.println("bmp Temperature read fail");
            writestatus(6);
            bmpneed  = 500;
            bmpstate = -100;
          }
          break;
        }

      case 2:
        {
          bmpneed = bmp.getPressure(Pressure, Temperature);
          if ( bmpneed != 0 )
          {
            #define lpbaro 0.9
            #define lpbaroground 0.9
            if ( baroheightlast == 0 )
            {
              GroundPressure = Pressure;
              PressureAveraged = Pressure;
            }
            if ( Altit <= 0.02 && Altit >= -0.02 )
            {
              GroundPressure = GroundPressure * lpbaroground + Pressure * ( 1 - lpbaroground );
            }
            PressureAveraged = PressureAveraged * lpbaro + Pressure * (1 - lpbaro);
            baroheight = 44330 * ( 1 - pow(PressureAveraged / 1013.25 , 0.1903 ) )   -   44330 * ( 1 - pow(GroundPressure / 1013.25 , 0.1903 ) );

            updateAltitude();

            bmpneed  = 30;
            bmpstate = 0;
            baroheightlast = micros();
          }
          else
          {
            Serial.println("bmp Pressure read fail");
            bmpneed  = 200;
            bmpstate = -100;
            writestatus(6);
          }
          break;
        }
    }
    bmplast = millis();
    writestatus(0);

  }
  else if ( bmpstate == -100 )
  {
    Serial.println("bmp has failed");
    bmpstate = 0;
    bmpneed = 200;
    bmplast  = millis();
  }

  if ( micros() - radiolast > 2000000 )
  {
    turnoff = 1;
  }

  if ( ( Throttle <= 970 && turnoff == 1 ) || ( micros() - pidlooplast > 40000 ) )
  {
    if ( micros() - pidlooplast > 40000 )
    {
      Serial.println("mpu fail safe");
      writestatus(5);
    }
    else
    {
      Serial.print("radio fail safe");
      writestatus(7);
    }
    MotorLeftFront  .writeMicroseconds ( 900 );
    MotorRightFront .writeMicroseconds ( 900 );
    MotorLeftBack   .writeMicroseconds ( 900 );
    MotorRightBack  .writeMicroseconds ( 900 );
    while (1) {}
  }

}


/*
   Kalman filter below is used to figure out altitude and velocity in global z axis
   the model is estimated very poorly. I was concerned about systen ram,
   the theoretical/mathemetic model has assumption that the acceleration is 0
   which is obviously never possible, for your lowcost quadcopter
   so use it at your own risk :p
   It worked well for me, but you could do better, I believe you
*/

/*
   any matrix 2 x 2  D is
                     ___         ___
                     |               |
                     |   D00    D01  |
                     |   D10    D11  |
                     |___         ___|


   any matrix 2 x 1 D  is
                       ___     ___
                      |           |
                      |    D0     |
                      |    D1     |
                      |___     ___|
*/

unsigned long int lastcall = 0;
float P_00, P_01, P_10, P_11; // prior errors
float X_0, X_1;           // model estimates
float Q = 0.03;           // accelerometer noise
float lowpassedX0 = 0;
float lowpassedX1 = 0;

float lowpacce[3];

void predictAltitude()
{ 
  
  float dtlocal = ((float)( micros() - lastcall )) / 1000000 ;
  if ( lastcall == 0 )
  {
    dtlocal = 0;
  }

  if( accez > 0.045 || accez < -0.045 )
  {
    azreal = accez;
  }
  else
  {
    azreal = 0;
  }
  X_1    = X_1 + azreal * dtlocal;
  lowpassedX1 = lowpassedX1 * 0.8 + X_1 * 0.2;
  X_0    = X_0 + X_1 * dtlocal + 0.5 * azreal * dtlocal * dtlocal;
  lowpassedX0 = lowpassedX0 * 0.8 + X_0 * 0.2;

  vzreal = lowpassedX1;
  Altit  = lowpassedX0;

  float T00, T01, T10, T11; // Temporary matrix
  T00 = P_00 + ( P_01 + P_10 ) * dtlocal + P_11 * dtlocal * dtlocal + Q;
  T01 = P_01 + P_11 * dtlocal + Q * dtlocal;
  T10 = P_10 + P_11 * dtlocal;
  T11 = P_11 + Q;

  P_00 = T00;
  P_01 = T01;
  P_10 = T10;
  P_11 = T11;

  lastcall = micros();

    /*Serial.print  (-20);
    Serial.print  (" ");
    Serial.print  ( 0 );
    Serial.print  (" ");
    Serial.print  (+20);
    Serial.print  (" ");
    Serial.print  (Altit * 100);
    Serial.print  (" ");
    //Serial.print  (ultraheight  * 100);
    Serial.print  (" ");
    //Serial.print  (baroheight  * 100);
    Serial.print  (" ");
    Serial.print  (vzreal * 100);
    Serial.print  (" ");
    //Serial.print  (azreal * 1000);
    Serial.print  (" ");
    //Serial.print  (lowpassed * 1000);lowpassed = lowpassed * 0.999 + azreal * 0.001;
    Serial.print  (" ");
    //Serial.print  (dtlocal,10);
    Serial.println();*/
}

float X0, X1;             // sateextimations
float P00, P01, P10, P11; // state estimation errors
float R = 5.0;           // noise from barometer

void updateAltitude()
{
  float Z0;
  if ( Altit <= 1.5  && zcos > 0.2)
  {
    float fraction = pow((Altit / 1.5), 2);
    Z0 = baroheight * fraction + ultraheight * ( 1 - fraction );
    R = 0.03;
  }
  else
  {
    Z0 = baroheight;
    R = 5.0;
  }
  float Y0 = Z0 - X_0;

  float K00 = P_00 / (P_00 + R);
  float K10 = P_10 / (P_00 + R);

  X0 = X_0 + K00 * Y0;
  X1 = X_1 + K10 * Y0;

  P00 = P_00 * ( 1 - K00 );
  P01 = P_01 * ( 1 - K00 );
  P10 = P_00 * ( -K10 ) + P_10;
  P11 = P_10 * ( -K10 ) + P_11;

  lowpassedX0 = lowpassedX0 * 0.8 + X0 * 0.2;
  lowpassedX1 = lowpassedX1 * 0.8 + X1 * 0.2;

  vzreal = lowpassedX1;
  Altit  = lowpassedX0;

  X_0 = X0;
  X_1 = X1;
  P_00 = P00;
  P_01 = P01;
  P_10 = P10;
  P_11 = P11;
}
