#include <Wire.h>
#include <Servo.h>

#include "QTRSensors.h"
#include "Ultrasonic.h"
#include "Battery.h"

#include "DRV8835MotorShield.h"


// Ranger distances
#define safety 20
#define threshold 50

// Gains for the line follower segment
#define Kp .25
#define Kd .25 

// Motor speed and pins 
#define DEFAULT_SPEED 150 // Default speed, may need a different one for the two pairs of motors
#define MAX_SPEED 200 // Max Speed

// Threshold for battery
#define battery_threshold 200

// Line follower setup
#define MIDDLE_SENSOR 4 // the middle sensor of the array sensor
#define NUM_SENSORS 4      // number of sensors used
#define TIMEOUT 2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2
#define DEBUG 1 // set to 1 if serial debug output needed

DRV8835MotorShield motors; // The H bridge for the motors
Servo steering; // Handles the steering servo motor

Ultrasonic left_range(35,34); // Trigger Pin, Echo Pin of left ranger
Ultrasonic right_range(25,24); // Trigger Pin, Echo Pin of right ranger
Ultrasonic front_range(23,22); // Trigger Pin, Echo Pin of left ranger 

Battery battery(3400, 4600, A0, 3); // Pulled from Battery.h git's example

QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 13}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); // IR sensor, {} contain pins of the sensors
unsigned int sensors[NUM_SENSORS]; 

void sensor_calibration();
void follow_segment();
void set_motor(char side, int motorspeed);
void move_motors(double left_speed, double right_speed);

int last_error = 0;

//========================================================================================
// ARDUINO MAIN FUNCTIONS
//========================================================================================
void setup()
{
  Serial.begin(9600);
  steering.attach(2); // Gotta attach that servo motor
  stopRobot();
}
void loop(){
    while (int(front_range.Ranging(CM)) < safety)
    {
      // if any point the ranger detects something very close in the front of it, it needs to stop.
    stopRobot();
    Serial.println(front_range.Ranging(CM));
    delay(1000);
    reverse();
    delay(1250);
    }
  if (int(front_range.Ranging(CM))  > threshold)
  {
    // if everything's all clear, just keep moving!
    forward();
    Serial.println(front_range.Ranging(CM));
  }
  if (front_range.Ranging(CM) < threshold)
  {
    if(left_range.Ranging(CM) > threshold && (left_range.Ranging(CM) < right_range.Ranging(CM)))
    {
      // if it's blocked in the front and the left side has less obstacles, it'll turn left. 
      turnLeft();
      delay(500);
    }
    else if (right_range.Ranging(CM) > threshold && (right_range.Ranging(CM) < left_range.Ranging(CM)))
    {
      // if it's blocked in the front and the right side has less obstacles, it'll turn right.
      turnRight();
      delay(500);
    }
    else // if they are equally obstructed
    { 
      while (right_range.Ranging(CM) < threshold && left_range.Ranging(CM) < threshold)
      {
        reverse(); // reverse so you don't bump anything if you are blocked on all sides
        delay(500); // set the delay to whatever it takes for it
      }
      // Get out of there, do a 180!
      if (left_range.Ranging(CM) < right_range.Ranging(CM))
      {
        turnAroundLeft(); // if the left side is clearer, turn around using the left side
        delay(1000); // set the delay time that is equal to whatever time it takes to turn around. If the delay time is really long, you might do a 360!
      }
      if (right_range.Ranging(CM) < left_range.Ranging(CM))
      {
        turnAroundRight(); // if the right side is clearer, turn around using the right side
        delay(1000); // set the delay time that is equal to whatever time it takes to turn around. If the delay time is really long, you might do a 360!
      }
    }
  }
}

//===================================================================================
// LINE FOLLOWER SEGMENT
//===================================================================================
//===================================================================================
// FUNCTIONS TO DRIVE CAR AND CALBIRATE SENSOR
//===================================================================================
void forward() {
  steering.write(90); // the steering wheels are straight
  set_motor(2, DEFAULT_SPEED); // DRV motors, drive motors forward
  
}
void reverse() {
  steering.write(90); // the steering wheels are straight
  set_motor(2, DEFAULT_SPEED*(-1)); // DRV motors, negative speed to reverse}
}
void turnRight() {
  steering.write(180); // the steering wheels so the car turns right
  set_motor(2, DEFAULT_SPEED); // May need to change the speeds so it doesn't turn very aggressively and crash
}
void turnLeft() {
  steering.write(0); // the steering wheels so the car turns left
  set_motor(2, DEFAULT_SPEED); // May need to change the speeds so it doesn't turn very aggressively and crash
}

void stopRobot() { //Stop the robot!
  steering.write(90); // the steering wheels are straight
  set_motor(2, 0); // all drive motors stopped
}
void turnAroundRight() // Turn that robot around!
{
  steering.write(180); // turn the steering right 
  set_motor(2, DEFAULT_SPEED); // and just drive motors
}
void turnAroundLeft()
{
  steering.write(0); // turn the steering left 
  set_motor(2, DEFAULT_SPEED); // and just drive motors
}

//===================================================================================
// SETTING THOSE MOTOR SPEEDS
//===================================================================================
void set_motor(int side, int motorspeed)
{
  if (motorspeed > MAX_SPEED ) motorspeed = MAX_SPEED; // limit top speed
  
  if (side == 0) // May have to switch 0 and 1 for left and right motors
  {
    if (motorspeed < 0)
    {
      motors.setM1Speed(motorspeed);
    }
    if (motorspeed == 0) 
    {
      motors.setM1Speed(motorspeed);      
    }
    if ( motorspeed > 0)
    {
      motors.setM1Speed(motorspeed);
    }
  }
  else if (side == 1)
  {
    if (motorspeed < 0)
    {
      motors.setM2Speed(motorspeed);
    }
    if (motorspeed == 0)
    {
      motors.setM2Speed(motorspeed);
    }
    if ( motorspeed > 0)
    {
      motors.setM2Speed(motorspeed);
    }
  }
  else
  {
    motors.setM1Speed(motorspeed);
    motors.setM2Speed(motorspeed*(-1));
  }
}

