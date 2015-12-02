#include <Servo.h>
#include "QTRSensors.h"
#include "Ultrasonic.h"
#include "Battery.h"
#include <Wire.h>
#include "DRV8835MotorShield.h"


// Ranger distances
#define safety 15.24
#define threshold 200
#define oddMax 500

// Gains for the line follower segment
#define Kp .25
#define Kd .25

// Motor speed and pins
#define DEFAULT_SPEED -200 // Default speed, may need a different one for the two pairs of motors
#define MAX_SPEED -200 // Max Speed

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

Ultrasonic left_range(32,33); // Trigger Pin, Echo Pin of left ranger
Ultrasonic right_range(28,29); // Trigger Pin, Echo Pin of right ranger
Ultrasonic front_range(22,23); // Trigger Pin, Echo Pin of left ranger

Battery battery(3400, 4600, A0, 3); // Pulled from Battery.h git's example

QTRSensorsRC qtrrc((unsigned char[]) {2, 3, 4, 13}, NUM_SENSORS, TIMEOUT, EMITTER_PIN); // IR sensor, {} contain pins of the sensors
unsigned int sensors[NUM_SENSORS];

//void sensor_calibration();
//void follow_segment();

void stopRobot();
void reverse();
void forward();
void turnLeft();
void turnRight();
void turnAroundLeft();
void turnAroundRight();

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

//	sensor_calibration();
}
void loop(){
  long frontRange = front_range.Ranging(CM); 
  long leftRange = left_range.Ranging(CM);
  long rightRange = right_range.Ranging(CM);
  for(int i = 0; i < 4; i++) {
    frontRange += front_range.Ranging(CM);
    leftRange += left_range.Ranging(CM);
    rightRange += right_range.Ranging(CM);
  }
  frontRange = frontRange/5.0;
  leftRange = leftRange/5.0;
  rightRange = rightRange/5.0;

  if (frontRange < safety || frontRange > oddMax)
  {
    Serial.println("Stop");
  		// if any point the ranger detects something very close in the front of it, it needs to stop.
  	stopRobot();
		delay(500);
    while (front_range.Ranging(CM) < safety && front_range.Ranging(CM) < threshold)
		{
      Serial.println("Reverse\n");
  		reverse(); // reverse so you don't bump anything if you are blocked on all sides
			//	delay(500); // set the delay to whatever it takes for it
		}
           
  }
//	else if (frontRange > threshold)
//	{
//    Serial.println("Forward");
//		// if everything's all clear, just keep moving!
//    forward();
//     // delay(1);
//    
//	}
	else /*if (frontRange < threshold)*/
	{
		if(leftRange <= threshold &&  (leftRange > rightRange))
		{
      Serial.println("Turn Left\n");
			// if it's blocked in the front and the left side has less obstacles, it'll turn left.
			turnLeft();
		//	delay(500);
		}
		else if (rightRange <= threshold && (rightRange > leftRange))
		{
      Serial.println("Turn Right\n");
			// if it's blocked in the front and the right side has less obstacles, it'll turn right.
			turnRight();
			//delay(500);
		}
		else // if they are equally obstructed
		{
      if (rightRange > threshold && leftRange > threshold) 
      {
        Serial.println("Forward");
      }
			else if (rightRange < threshold && leftRange < threshold)
			{
        Serial.println("Reverse");
				reverse(); // reverse so you don't bump anything if you are blocked on all sides
				//delay(500); // set the delay to whatever it takes for it
			}
			// Get out of there, do a 180!
			else if (leftRange > rightRange)
			{
        Serial.println("Turn Around Left\n");
				turnLeft(); // if the left side is clearer, turn around using the left side
				//delay(1000); // set the delay time that is equal to whatever time it takes to turn around. If the delay time is really long, you might do a 360!
			}
			else if (rightRange > leftRange)
			{
        Serial.println("Turn Around Right\n");
				turnRight(); // if the right side is clearer, turn around using the right side
				//delay(1000); // set the delay time that is equal to whatever time it takes to turn around. If the delay time is really long, you might do a 360!
			}
		}
	}
  /*while (battery.level() < battery_threshold) // if the battery level is low!
  {
  	line_position = qtrrc.readLine(sensors, 1); // start reading those sensors
  	follow_segment(); // go to line follower function
  }*/
}
/*
//===================================================================================
// LINE FOLLOWER SEGMENT
//===================================================================================
void follow_segment(){
	// Since we're using white tape on black surface, we need to figure out what the
	// LIGHT reflectance values are and set a threshold for that. This was just an example.
	// Basically if we don't detect the white tape, you need to just keep moving forward.
	if (sensors[0] < DARK && sensors[1] < DARK && sensors[2] < DARK &&
      sensors[3] < DARK)
	{
	    // go straight
	    forward();
  	}
  	else
  	{
  	 // Oh god, how do you do this with a separate steering motor??
	  long int error = line_position - 2500;
	  int motorSpeed = Kp * error + Kd * (error - last_error);
	  last_error = error;

	  int rightMotorSpeed = DEFAULT_SPEED + motorSpeed;
	  int leftMotorSpeed = DEFAULT_SPEED - motorSpeed;

	  set_motor(1, leftMotorSpeed);
	  set_motor(0, rightMotorSpeed);
	  
		long int error = line_position - 2500;
		int range_adj = Kp * error + Kd * (error-last_error);
		last_error = error;

		steering =
	  
	}
}
*/
//===================================================================================
// FUNCTIONS TO DRIVE CAR AND CALBIRATE SENSOR
//===================================================================================
/*void sensor_calibration(){
    qtrrc.calibrate();
    // Since our counter runs to 90, the total delay will be
    // 90*20 = 1800 ms.
    delay(20);
    stopRobot();
	delay(2000);
}*/
void forward() {
	steering.write(90); // the steering wheels are straight
	motors.setSpeeds(DEFAULT_SPEED, DEFAULT_SPEED); // DRV motors, drive motors forward // Toshiba hbridge motors, motors forward
}
void reverse() {
  steering.write(0); // the steering wheels are straight
  motors.setSpeeds(-DEFAULT_SPEED, -DEFAULT_SPEED); // DRV motors, negative speed to reverse
}

void turnRight() {
  steering.write(180); // the steering wheels so the car turns right
  motors.setSpeeds(DEFAULT_SPEED, DEFAULT_SPEED);; // May need to change the speeds so it doesn't turn very aggressively and crash

}
void turnLeft() {
  steering.write(0); // the steering wheels so the car turns left
  motors.setSpeeds(DEFAULT_SPEED, DEFAULT_SPEED); // May need to change the speeds so it doesn't turn very aggressively and crash

}

void stopRobot() { //Stop the robot!
	steering.write(90); // the steering wheels are straight
	motors.setSpeeds(0, 0); // all drive motors stopped
}
// The functions below are kind of redundant

//void turnAroundRight() // Turn that robot around!
//{
//	steering.write(180); // turn the steering right
//	motors.setSpeeds(0, 0); // and just drive motors
//}
//void turnAroundLeft()
//{
//	steering.write(0); // turn the steering left
//	motors.setSpeeds(-DEFAULT_SPEED, DEFAULT_SPEED); // and just drive motors
//}

//===================================================================================
// SETTING THOSE MOTOR SPEEDS
//===================================================================================
//void set_motor(int side, int motorspeed)
//{
//  if (motorspeed > MAX_SPEED ) motorspeed = MAX_SPEED; // limit top speed
//
//	if (side == 0) // May have to switch 0 and 1 for left and right motors
//	{
//		if (motorspeed < 0)
//		{
//			motors.setM1Speed(motorspeed);
//		}
//		if (motorspeed == 0)
//		{
//			motors.setM1Speed(motorspeed);
//		}
//		if ( motorspeed > 0)
//		{
//			motors.setM1Speed(motorspeed);
//		}
//	}
//	else if (side == 1)
// 	{
//		if (motorspeed < 0)
//		{
//			motors.setM2Speed(motorspeed);
//		}
//		if (motorspeed == 0)
//		{
//			motors.setM2Speed(motorspeed);
//		}
//		if ( motorspeed > 0)
//		{
//			motors.setM2Speed(motorspeed);
//		}
// 	}
// 	else
// 	{
// 		motors.setM1Speed(-motorspeed);
// 		motors.setM2Speed(-motorspeed);
// 	}
//}
