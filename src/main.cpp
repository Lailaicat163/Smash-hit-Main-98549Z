#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>



#include "vex.h"

using namespace vex;

#define PI 3.1415926535897

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor leftMotorC = motor(PORT3, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT4, ratio18_1, true);
motor rightMotorB = motor(PORT5, ratio18_1, true);
motor rightMotorC = motor(PORT6, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 266.7, 190.5, mm, 1);

controller Controller1 = controller(primary);

motor IntakeMotor = motor(PORT12, ratio18_1, true);
motor UpperMotor = motor(PORT13, ratio36_1, true);
motor Second_IM = motor(PORT14, ratio18_1, true);

//Pneumatic A is for the scraper mechanism
digital_out scraper = digital_out(Brain.ThreeWirePort.A);

//Initializes the rotational sensors. Set true to inverse the rotation and velocity to negative values.
rotation rotational = rotation(PORT17, false);

//Initializes the inertial sensor. starts from 0 degrees and increases by turning clockwise.
inertial inertialSensor = inertial(PORT19);


//sets max rpm of motors
const float maxMotorRPM = 600.0;

// controller Controller2 = controller(partner);

// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);

//////////////////////////////////////////////
//Start PID FUNCTIONS AND AUTO CODE
//////////////////////////////////////////////
//Sets the initial robot vector (x, y, heading in degrees). Set whichever one to comment to start change position.
static double robotPosition[3] = {87.5, 17.5, 90};  //right side of the field
//static double robotPosiition[3] = {67.5, 17.5, 90};  //left side of the field

//PID Settings; Tweak these values to tune PID.
//For going straight
const double kP = 0.07;
const double kI = 0;
const double kD = 0;

//For turning
const double turning_kP = 0.5;
const double turning_kI = 0;
const double turning_kD = 0;

//Initializing other variables for PID. These will be changed by the function, not the user.
//distance errors
static double distanceError; //Current value - desired value: Positional Value
static double prevDistanceError = 0; //Positional Value 20 milliseconds ago
static double derivativeDistanceError; //error - prevError: Speed Value
static double integralDistanceError = 0; //Total error = total error + error

//Turning errors
static double headingError; //Current value - desired value: Positional Value
static double prevHeadingError = 0; //Positional Value 20 milliseconds ago
static double derivativeHeadingError; //error - prevError: Speed Value
static double integralHeadingError = 0; //Total error = total error + error
//PID settings end

//Variables modified for use
bool resetPID_Sensors = false;
bool enableDrivePID = false;

//NOT DONE THE PID FUNCTION PLZ DON'T TOUCH
//PID FUNCTION STARTS HERE
int drivePID(double x_value, double y_value, double heading_value) {
  while(enableDrivePID) {
    if (resetPID_Sensors == true) {
      //Resets the sensors and variables
      rotational.resetPosition();
      distanceError = 0;
      prevDistanceError = 0;
      derivativeDistanceError = 0;
      integralDistanceError = 0;
      headingError = 0;
      prevHeadingError = 0;
      derivativeHeadingError = 0;
      integralHeadingError = 0;
      resetPID_Sensors = false;
    }
    //Arc length formula (theta in deg)t: theta/180 * pi (radian conversion) * radius
    double distanceTravelled = rotational.position(deg) * PI / 180 * 1.375;  //1.375 is radius of odom wheel in inches
    double robotHeading = inertialSensor.heading(); //in degrees
    //Calculates target values for distance
    double targetDistance = sqrt( pow(x_value - robotPosition[0], 2) + pow(y_value - robotPosition[1], 2) );
    //Forward movement error calculations
    distanceError = targetDistance - distanceTravelled;
    //Derivative and integral error calculations
    derivativeDistanceError = distanceError - prevDistanceError;
    if (fabs(distanceError) < 5) { //integral windup prevention. makes it so that integral only adds up when close to target
      integralDistanceError += distanceError;
    } else {
      integralDistanceError = 0;
    }
    //Calculation of motor power
    double motorPower = (distanceError * kP) + (integralDistanceError * kI) + (derivativeDistanceError * kD);
    //end forward movement error calculations
    //START Angular movement calculations
    double headingError = heading_value - robotHeading + 540.0;

    // Use fmod for floating-point modulo. modulo is the remainder after division
    headingError = fmod(headingError, 360.0);
    // Ensures positive result.
    if (headingError < 0) {
      headingError += 360.0;
    }
    //This sets the calculations in range of -180 to 180 degrees
    if (headingError > 180.0) {
    headingError -= 360.0;
    }
    //Derivative and integral error calculations MAY BE CHANGED LATER INTO ABS VALUE
    derivativeHeadingError = headingError - prevHeadingError;
    if (fabs(headingError) < 5) { //integral windup prevention. makes it so that integral only adds up when close to target
      integralHeadingError += headingError;
    } else {
      integralHeadingError = 0;
    }
    //Calculation of turning motor power
    double turnMotorPower = (headingError * turning_kP) + (integralHeadingError * turning_kI) + (derivativeHeadingError * turning_kD);
    //Making motors move
    double motorsLeftPower = motorPower + turnMotorPower;
    double motorsRightPower = motorPower - turnMotorPower;
    //Limiting motor power to max RPM. Gets minimum rpm of +600, then maximum rpm of -600, setting the -600 <= rpm <= 600 limit.
    motorsLeftPower = fmax(fmin(motorsLeftPower, maxMotorRPM), -maxMotorRPM);
    motorsRightPower = fmax(fmin(motorsRightPower, maxMotorRPM), -maxMotorRPM);
    LeftDriveSmart.spin(forward, motorsLeftPower, rpm);
    RightDriveSmart.spin(forward, motorsRightPower, rpm);
    //Sets previous robot position vector to current robot position vector. Cos and sin in radians because that is what they take as arguments.
    //Getting the sin and cos is like polar coordinates where distance travelled is the radius and robot heading is the angle. 
    //(x,y) == (Rcos(theta),Rsin(theta))
    robotPosition[0] += distanceTravelled * cos(robotHeading * PI / 180);
    robotPosition[1] += distanceTravelled * sin(robotHeading * PI / 180);
    robotPosition[2] = robotHeading;
    if (fabs(robotPosition[0] - x_value) < 0.1 && fabs(robotPosition[1] - y_value) < 0.1 && fabs(robotPosition[2] - heading_value) < 1.0) {
      LeftDriveSmart.stop();
      RightDriveSmart.stop();
      enableDrivePID = false; //disables PID when target is reached
    }
    //Sets prevError to current error
    prevDistanceError = distanceError;
    prevHeadingError = headingError;
    return 1;  // doesn't matter what it returns
    vex::task::sleep(20); //waits 20 milliseconds before next loop
  }
  return 2; // doesn't matter what it returns
}


#pragma endregion VEXcode Generated Robot Configuration

// ----------------------------------------------------------------------------
//                                                                            
//    Project:                                               
//    Author:
//    Created:
//    Configuration:        
//                                                                            
// ----------------------------------------------------------------------------

// Include the V5 Library
#include "vex.h"
#include <iostream>
// Allows for easier use of the VEX Library
using namespace vex;

// Begin project code

void preAutonomous(void) {
  // actions to do when the program starts
  Brain.Screen.clearScreen();
  Brain.Screen.print("pre auton code");
  wait(1, seconds);
  Drivetrain.setDriveVelocity(100, percent);
  rotational.resetPosition(); //resetting the rotational sensor position to 0
  //calibrating the inertial sensor MUST DO THIS
  inertialSensor.calibrate(); //in this version inertial var isn't here but assuming it is.
  while(inertialSensor.isCalibrating() == true) {
    wait(20, msec);
  }
  if (inertialSensor.isCalibrating() == false) {
    inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
  }
}

void autonomous(void) {
  Drivetrain.setDriveVelocity(100, percent);
  Brain.Screen.clearScreen();
  Brain.Screen.print("autonomous code");
  IntakeMotor.setVelocity(100, percent);
  UpperMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  // place automonous code here
  enableDrivePID = true;
  while (robotPosition[0] != 87.5 and robotPosition[1] != 22.5 and robotPosition[2] != 0) {
    drivePID(87.5, 22.5, 0);
  }
  resetPID_Sensors = true;
  // while (robotPosition[0] != 150 and robotPosition[1] != 90 and robotPosition[2] != 50) {
  //   drivePID(150, 90, 50);
  // }
  // resetPID_Sensors = true;
}

bool scraperState = true;

//following codes are for input purpose, if there are some bugs exist, 
//then put the codes back to userc=Control
void input() {
  if(Controller1.ButtonR2.pressing() == true){
    IntakeMotor.spin(forward);
    Second_IM.spin(forward);
  }
  else if(Controller1.ButtonR1.pressing() == true){
    IntakeMotor.spin(reverse);
    Second_IM.spin(reverse);
  }
  else{
    IntakeMotor.stop();
    Second_IM.stop();
  }
  if(Controller1.ButtonL1.pressing() == true){
    UpperMotor.spin(reverse);
  }
  else if(Controller1.ButtonL2.pressing() == true){
    UpperMotor.spin(forward);
  }
  else{
    UpperMotor.stop();
  }
  if (Controller1.ButtonA.pressing() == true){
    wait(10, msec);
    if (scraperState == true){
      scraper.set(false);
    }
    else{
      scraper.set(true);
    }
  }
}
//trrestrest

event Input;

void userControl(void) {
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  IntakeMotor.setStopping(brake);
  IntakeMotor.setVelocity(100, percent);
  // place driver control in this while loop
  
  while(true){
    enableDrivePID = false; //disables PID control during user control
    wait(10, msec);
    input();
  }  
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // create competition instance
  competition Competition;
  IntakeMotor.setVelocity(75, percent);
  Second_IM.setVelocity(75, percent);
  

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  
  
  // Run the pre-autonomous function.
  preAutonomous();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}