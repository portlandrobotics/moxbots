/**************************************************************
 * This sketch is for an Arduino 101 BLE rover using Blynk and Adafruit Motor Shield V2
 * Code was copied from both Adafruit and Blynk librarie examples
 * Full documentation on building this rover yourself on Hackster.IO and electronhacks.com
 * Sketch is released under MIT license
 **************************************************************/


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "CurieIMU.h"

void setMotor(int id, int direction, int speed=-1);



// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor* motors[4];


#define IS_TRACKED_ROBOT

#define MOTOR_FL 3
#define MOTOR_FR 4
#define MOTOR_RL 2
#define MOTOR_RR 1

#define MOTOR_TR MOTOR_RR
#define MOTOR_TL MOTOR_RL

//######### SETUP ######################################
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("MoxBot booting");

  // Set up pin connected to wheel encoder switch
  pinMode(7,INPUT_PULLUP);


  // Set up motor driver boards
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // assign the motor channels into the array
  for(int i=0;i<4;i++) {
    motors[i] = AFMS.getMotor(i+1);
    motors[i]->setSpeed(255);
  }

  // start up the IMU, do some Gyro setup
  CurieIMU.begin();

  // 250 degrees per second
  CurieIMU.setGyroRange(250);
}



//######### Subroutines ################################

// Convenience function to manipulate the mototrs.  Mostly useful becuase two of the motors (2 and 4) are backwards

void setMotor(int id, int direction, int speed /* =-1*/)
{
  int swap = direction;
  if((id == 2) || (id==4))
  {
    if(direction == FORWARD)
      swap = BACKWARD;
    if(direction == BACKWARD)
      swap = FORWARD;
  }

  motors[id-1]->run(swap);

  if(speed>=0)
    motors[id-1]->setSpeed(speed);
  
}

// convenience function to set motor speed
void setMotorSpeed(int id, int speed)
{
  motors[id-1]->setSpeed(speed);
}

// convert gyro reading to degrees
float scaleGyro(int reading, int range) {
  return ( reading/32768.9)*range; 
}

// convert accelerometer reading to G's
float scaleAccel(int reading, int range) {
  return (float)(((float)reading/(float)32768.0)*(float)range);
}

// read all 3 gyro axis and store their results, in degrees, to the referenced arguments
void readGyroScaled(float &x, float &y, float &z) {
  int range = CurieIMU.getGyroRange();
  int rx,ry,rz;
  CurieIMU.readGyro(rx,ry,rz);
  x=scaleGyro(rx,range);
  y=scaleGyro(ry,range);
  z=scaleGyro(rz,range);
}

// read all 3 acceleromoter axis and store their results, in G's, to the referenced arguments
void readAccelScaled(float &x, float &y, float &z) {
  int range = CurieIMU.getAccelerometerRange();
  int rx,ry,rz;
  CurieIMU.readAccelerometer(rx,ry,rz);
  x=scaleAccel(rx,range);
  y=scaleAccel(ry,range);
  z=scaleAccel(rz,range);
}

//########## LOOP ######################################
void loop() {

  #if 0
  // this is a bunch of code we used to test the IMU sensor.  Left in here for future reference
  
  CurieIMU.setAccelerometerRange(4);
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  float accelRate = CurieIMU.getAccelerometerRate();
  float gyroRate = CurieIMU.getGyroRate();
  {
    char b2[512];
    sprintf(b2,"%f %f",accelRate,gyroRate);
    Serial.println(b2);
  }
    
  while(1) {
    float x,y,z;
    readAccelScaled(x,y,z);
    char buffer[512];
    sprintf(buffer,"%f %f %f",x,y,z);
    Serial.println(buffer);
    delay(1000);
}


   int gyroRange = CurieIMU.getGyroRange();
  while(1) {
    float x,y,z;
    readGyroScaled(x,y,z);
    //CurieIMU.autoCalibrateGyroOffset();
    //CurieIMU.readGyro(x,y,z);
    Serial.print(x);
    Serial.print(" ");
    Serial.print(y);
    Serial.print(" ");
    Serial.println(z);
    delay(1000);
    
    
  }
  // end disabled test code
#endif


  

  // kill the motors and wait 1 second to get clean "zero" reading to calibrate the gyro
  setMotor(MOTOR_FL,RELEASE);
  setMotor(MOTOR_RL,RELEASE);
  setMotor(MOTOR_FR,RELEASE);
  setMotor(MOTOR_RR,RELEASE);

  delay(3000);

  // calibrate the Gryo
   CurieIMU.autoCalibrateGyroOffset();


  // calibrate the accelerate
   CurieIMU.setAccelerometerRate(800);

  // 2G range
  CurieIMU.setAccelerometerRange(2);
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);

  int range = CurieIMU.getAccelerometerRange();
  int rx,ry,rz;

  // not sure if this helps, but take a last accelerometer measurement
  //   and then subtract it from future measurements
  CurieIMU.readAccelerometer(rx,ry,rz);
  int xtracal=rx;
  
  Serial.println(xtracal);

  unsigned long startTime = micros();

  // desired travel distance, presently in wheel clicks
  float travel = 16*4;

  // *** loop state variables ***
  float lastm = startTime;
  float velocity =0;
  float location = 0;

  float lastSwitchEvent = 0;

  float turnaccum = 0.;

  // iteration counter for occasionally printing status messages
  int cup=0;

  int lastread=2;
  int holdoff = 0;

  int countdown=10;
  
  while(1) {
    CurieIMU.readAccelerometer(rx,ry,rz);
    float x=scaleAccel(rx-xtracal,range);

    // optional deadzone
    //if (fabs(x) < .01)
    //  x = 0;

    // measure time since last loop interation
    float m = micros();
    float dt = (m-lastm)/1e6;

    velocity += 9.8*x*dt;

    //location += velocity * dt;
    int switchread = digitalRead(7);
    /*if(m-lastSwitchEvent < 10) {
      // debounce
    }
    else*/
    if(lastread != switchread) {
      //if(!lastread && switchread)
        location++;
      lastread=switchread;
      lastSwitchEvent = m;
      Serial.print(switchread);
      Serial.print(" ");
      Serial.println(location);
    }

    if(cup++ % 10000 < 1) {
      float t = (m-startTime)/1e6;
      char buffer[512];
      
      sprintf(buffer, "%d %f %f : %f %f : %f",rx,x,dt,velocity,location,velocity / t);
      Serial.println(buffer);
    }

    if(fabs(location) > travel+.5)
      break;

#if 0
    if (countdown > 0) {
      countdown--;
    }
    else if(countdown ==0) {
      if(fabs(location) > .01) {
        char buffer[512];
         sprintf(buffer, "FAIL %f %f : %f %f",x,dt,velocity,location);
          Serial.println(buffer);
          return;
      }
#endif

    float gx,gy,gz;
    readGyroScaled(gx,gy,gz);

    turnaccum += gz*dt;

      
      int motorspeedLeft = 0x80;
      int motorspeedRight = 0x80 + turnaccum;

              char buffer[512];
         sprintf(buffer, "FAIL %f %f : %d %f",gz,turnaccum,motorspeedRight,location);
          Serial.println(buffer);

      setMotor(MOTOR_FL,BACKWARD, motorspeedLeft);
      setMotor(MOTOR_RL,BACKWARD, motorspeedLeft);
      setMotor(MOTOR_FR,BACKWARD, motorspeedRight);
      setMotor(MOTOR_RR,BACKWARD, motorspeedRight);
      countdown = -1;

      #if 0
    }
    else {
      //countdown --;
      //if(countdown < -10000)
      //  break;
    }
#endif
    lastm=m;
  }

  // turn off the motors
  setMotor(MOTOR_FL,RELEASE);
  setMotor(MOTOR_RL,RELEASE);
  setMotor(MOTOR_FR,RELEASE);
  setMotor(MOTOR_RR,RELEASE);


  return;

#if 0
  // this the gyro based turn code
  //   we'll put this stuff back in soon
  
  setMotor(MOTOR_FL,FORWARD, 128);
  setMotor(MOTOR_RL,FORWARD, 128);
  setMotor(MOTOR_FR,FORWARD, 64);
  setMotor(MOTOR_RR,FORWARD, 64);


  float accum = 360;
    
  int speed=64;
  lastm = millis();
  while(1) {
    delay(50);

    float m = millis();
    float dm = (m-lastm)/1000.;

    float gx,gy,gz;
    readGyroScaled(gx,gy,gz);

    accum += gz*dm;

    if(gz < 18 && speed > 0)
      speed--;
    else if (speed < 255)
      speed++;

    setMotorSpeed(MOTOR_FR, speed);
    setMotorSpeed(MOTOR_RR, speed);

#ifdef DEBUGSTUFF
    Serial.println(dm);
    Serial.println(gz);
    Serial.println(accum);
    Serial.println(speed);
    Serial.println("");
#endif

    if(accum < 0)
      break;

    lastm=m;
  }

  setMotor(MOTOR_FL,RELEASE);
  setMotor(MOTOR_RL,RELEASE);
  setMotor(MOTOR_FR,RELEASE);
  setMotor(MOTOR_RR,RELEASE);

  delay(1000);


  return;
#endif
}



