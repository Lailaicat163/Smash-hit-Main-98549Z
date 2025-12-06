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
motor leftMotorA = motor(PORT1, ratio6_1, false);
motor leftMotorB = motor(PORT2, ratio6_1, false);
motor leftMotorC = motor(PORT3, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT4, ratio6_1, true);
motor rightMotorB = motor(PORT5, ratio6_1, true);
motor rightMotorC = motor(PORT6, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 266.7, 190.5, mm, 1);
   
controller Controller1 = controller(primary);

motor IntakeMotor = motor(PORT12, ratio36_1, true);
motor UpperMotor = motor(PORT13, ratio36_1, true);
motor Second_IM = motor(PORT14, ratio36_1, true);

//Pneumatic A is for the scraper mechanism
digital_out scraper = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.H);

//Initializes the rotational sensors. Set true to inverse the rotation and velocity to negative values.
rotation rotational = rotation(PORT17, true);

//Initializes the inertial sensor. starts from 0 degrees and increases by turning clockwise.
inertial inertialSensor = inertial(PORT19);


//sets max rpm of motors
float maxMotorPercentage = 100.0;

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

bool isAutonomous = false;
bool isDriverControl = false; 


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
//right side tracks from left corner
//left side tracks from left corner
//static double robotPosiition[3] = {61.25, 14.5, 90};  //left side of the field

/*
coordinates of left side ball cluster
48,53
*/
//PID Settings; Tweak these values to tune PID.
//For going straight
const double kP = 7;  //7
const double kI = 0.1;
const double kD = 5; //2

//For turning
//0.09
const double turning_kP = 0.6; //less than .1 
const double turning_kI = 0; //should be a really small number 0.024 example
//0.05
const double turning_kD = 0; //less than .05 usually

//Initializing other variables for PID. These will be changed by the function, not the user.
//distance errors
double distanceError; //Current value - desired value: Positional Value
double prevDistanceError = 0; //Positional Value 20 milliseconds ago
double derivativeDistanceError; //error - prevError: Speed Value
double integralDistanceError = 0; //Total error = total error + error

//Turning errors
double headingError; //Current value - desired value: Positional Value
double prevHeadingError = 0; //Positional Value 20 milliseconds ago
double derivativeHeadingError; //error - prevError: Speed Value
double integralHeadingError = 0; //Total error = total error + error
//PID settings end

//Variables modified for use
bool resetPID_Sensors = false;
bool enableDrivePID = false;

//For delta distance travelled calculation
double previousDistanceTravelled = 0;

//Variables for desired location
double desiredTurnValue = 0;
double targetDistance = 0;

double totalDistanceTravelled = 0;

int timerCount = 0;

//NOT DONE THE PID FUNCTION PLZ DON'T TOUCH
//PID FUNCTION STARTS HERE
int drivePID() {
  while(enableDrivePID == true) {
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
      previousDistanceTravelled = 0;
      totalDistanceTravelled = 0;
    }
    double robotHeading = inertialSensor.heading(); //in degrees
    //Calculates target values for distance
    //Forward movement error calculations
    distanceError = targetDistance;
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
    // START Angular movement calculations
    headingError = desiredTurnValue - robotHeading;
    headingError = -1 * (atan2(sin(headingError * PI /180), cos(headingError * PI / 180)) * 180.0 / PI);
    // Derivative
    double derivativeHeadingError = headingError - prevHeadingError;

    if (fabs(headingError) < 15) { //integral windup prevention. makes it so that integral only adds up when close to target
      integralHeadingError += headingError;
    } else {
      integralHeadingError = 0;
    }
    //Calculation of turning motor power
    double turnMotorPower = (headingError * turning_kP) + (integralHeadingError * turning_kI) + (derivativeHeadingError * turning_kD);
    //Making motors move
    double motorsLeftPower = motorPower + turnMotorPower;
    double motorsRightPower = motorPower - turnMotorPower;
    //Limiting motor power to max Percentage. Gets minimum rpm of +100, then maximum rpm of -100, setting the -100 <= percentage <= 100 limit.
    motorsLeftPower = fmax(fmin(motorsLeftPower, maxMotorPercentage), -maxMotorPercentage);
    motorsRightPower = fmax(fmin(motorsRightPower, maxMotorPercentage), -maxMotorPercentage);
    LeftDriveSmart.spin(forward, motorsLeftPower, percent);
    RightDriveSmart.spin(forward, motorsRightPower, percent);
    //Sets previous robot position vector to current robot position vector. Cos and sin in radians because that is what they take as arguments.
    //Getting the sin and cos is like polar coordinates where distance travelled is the radius and robot heading is the angle. 
    //(x,y) == (Rcos(theta),Rsin(theta))
    //Arc length formula (theta in deg)t: theta/180 * pi (radian conversion) * radius
    //Delta distance travelled since last reset
    totalDistanceTravelled = rotational.position(deg) * PI / 180 * 1.375;  //1.375 is radius of odom wheel in inches
    targetDistance = targetDistance - (totalDistanceTravelled - previousDistanceTravelled);
    if (fabs(targetDistance) < 0.1 && fabs(headingError) < 3 && timerCount < 20) { //If within 1 inches and 3 degree of target, stop motors and exit task
      timerCount += 1;
    } else if (fabs(targetDistance) > 0.1 && fabs(headingError) < 3) {
      timerCount = 0;
    } else if (timerCount >= 20) {
      timerCount = 0;
      LeftDriveSmart.setStopping(brake);
      RightDriveSmart.setStopping(brake);
      LeftDriveSmart.stop();
      RightDriveSmart.stop();
      resetPID_Sensors = true; //resets sensors when target reached
      enableDrivePID = false;  //disables PID when target reached
      return 0; //exits the function and TASK
    }
      
  
    //Sets prevError to current error
    previousDistanceTravelled = totalDistanceTravelled;
    prevDistanceError = distanceError;
    prevHeadingError = headingError;
    vex::task::sleep(20); //waits 20 milliseconds before next loop
  }
  return 1; //Doesn't matter what this returns
}

bool scraperState = false;

//following codes are for input purpose, if there are some bugs exist, 
//then put the codes back to userc=Control
void input() {
  if(Controller1.ButtonR2.pressing() == true){
    IntakeMotor.setVelocity(100, percent);
    Second_IM.setVelocity(100, percent);
    IntakeMotor.spin(forward);
    Second_IM.spin(forward);
  }
  else if(Controller1.ButtonR1.pressing() == true){
    IntakeMotor.setVelocity(50, percent);
    Second_IM.setVelocity(50, percent);
    IntakeMotor.spin(reverse);
    Second_IM.spin(reverse);
  }
  else{
    IntakeMotor.setVelocity(100, percent);
    Second_IM.setVelocity(100, percent);
    IntakeMotor.stop();
    Second_IM.stop();
  }
  if(Controller1.ButtonL1.pressing() == true){
    UpperMotor.setVelocity(50, percent);
    UpperMotor.spin(reverse);
  }
  else if(Controller1.ButtonL2.pressing() == true){
    UpperMotor.setVelocity(100, percent);
    UpperMotor.spin(forward);
  }
  else{
    UpperMotor.setVelocity(100, percent);
    UpperMotor.stop();
  }
  
}
void Scraper(){
    // wait(10, msec);
    if (scraperState == true){
      scraper.set(false);
      scraperState = false;
    }
    else{
      scraper.set(true);
      scraperState = true;
    }
  }

bool DescoreState = false;

void Descore(){
  if (DescoreState == true){
    descore.set(false);
    DescoreState = false;
  }
  else{
    descore.set(true);
    DescoreState = true;
  }
  
}

bool NegusConfirmed = false;

void Negus(){
  if (NegusConfirmed == true){
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Stopping Robot...");
    Brain.programStop();
  }
  else{
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Stop robot? Y or X");
    NegusConfirmed = true;
  }
}

void NoNegus(){
  if (NegusConfirmed == true){
    Controller1.Screen.clearLine(1);
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("Stopping Cancelled!");
    NegusConfirmed = false;
    wait(1500, msec);
    Controller1.Screen.clearLine(1);
  }
}
//trrestrest

event Input;



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
  Drivetrain.setDriveVelocity(100, percent);
  rotational.resetPosition(); //resetting the rotational sensor position to 0
  //calibrating the inertial sensor MUST DO THIS
  inertialSensor.calibrate(); 
}



void autonomous(void) {
  //UpperMotor forward goes right, reverse goes left
  //IntakeMotor and Second_IM forward up, reverse down
  isAutonomous = true;
  isDriverControl = false;
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  Brain.Screen.print("autonomous code");
  IntakeMotor.setVelocity(100, percent);
  UpperMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  IntakeMotor.setStopping(brake);
  if (inertialSensor.isCalibrating() == false) {
    inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
  }
  Controller1.ButtonX.pressed(Negus);
  scraperState = true;
  scraper.set(true);
  inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
  // Drivetrain.driveFor(forward, 55, inches);
  // Autonomous drivetrain has a bug where it will reverse the direction you input.
  // you will need to reverse your action.
  // Drivetrain.turnFor(left, 45, degrees);
  /* Desired locations for autonomous: Autonomous explaination
  Right side start autonomous: 
  1. Move to group of 3 ball location
  2. Intake
  3. turn left to middle goal
  4. outtake
  5. move in front of match load
  6. initiate scraper
  7. move into match load and intake only red balls
  8. move back
  9. turn to face long goal
  10. move forward
  11. outtake
  Left side start autonomous: 
  1. Move to group of 3 ball location
  2. Intake
  3. turn right to middle goal
  4. outtake
  5. move in front of match load
  6. initiate scraper
  7. move into match load and intake only red balls
  8. move back
  9. turn to face long goal
  10. move forward
  11. outtake
  */
  // place automonous code here
  // Start PID control
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 90;
  targetDistance = 11; //inches
  vex::task drivePID_Thread(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 110;
  targetDistance = 0; //inches
  vex::task drivePID_Thread2(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  maxMotorPercentage = 30;
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 110;
  targetDistance = 19; //inches
  vex::task drivePID_Thread3(drivePID);
  IntakeMotor.setVelocity(100, percent);
  Second_IM.setVelocity(15, percent); 
  UpperMotor.setVelocity(15, percent);
  Second_IM.spin(forward);
  UpperMotor.spin(forward);
  IntakeMotor.spin(forward);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  maxMotorPercentage = 100;
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 40;
  targetDistance = 0; //inches
  vex::task drivePID_Thread4(drivePID);
  IntakeMotor.stop();
  Second_IM.stop();
  UpperMotor.stop();
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 40;
  targetDistance = 14; //inches
  vex::task drivePID_Thread5(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 40;
  targetDistance = 0; //inches
  vex::task drivePID_Thread6(drivePID);
  Second_IM.setVelocity(100, percent); 
  UpperMotor.setVelocity(40, percent);
  IntakeMotor.spin(reverse);
  Second_IM.spin(reverse);
  UpperMotor.spin(reverse);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  vex::task::sleep(4000);
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 45;
  targetDistance = -(40); //inches
  vex::task drivePID_Thread7(drivePID);
  IntakeMotor.stop();
  Second_IM.stop();
  UpperMotor.stop();
  scraperState = false;
  scraper.set(false);  
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 270;
  targetDistance = 0; //inches
  vex::task drivePID_Thread8(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 270;
  targetDistance = 15; //inches
  IntakeMotor.setVelocity(100, percent);
  Second_IM.setVelocity(15, percent); 
  UpperMotor.setVelocity(15, percent);
  Second_IM.spin(forward);
  UpperMotor.spin(forward);
  IntakeMotor.spin(forward);
  vex::task drivePID_Thread9(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 270;
  targetDistance = -(10); //inches
  vex::task drivePID_Thread10(drivePID);
  IntakeMotor.stop();
  Second_IM.stop();
  UpperMotor.stop();
  scraperState = true;
  scraper.set(true); 
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 90;
  targetDistance = 0; //inches
  vex::task drivePID_Thread11(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 90;
  targetDistance = 13; //inches
  vex::task drivePID_Thread12(drivePID);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }
  //Next movement
  resetPID_Sensors = true;
  enableDrivePID = true;
  desiredTurnValue = 90;
  targetDistance = 0; //inches
  vex::task drivePID_Thread13(drivePID);
  IntakeMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent); 
  UpperMotor.setVelocity(100, percent);
  Second_IM.spin(forward);
  UpperMotor.spin(forward);
  IntakeMotor.spin(forward);
  while (enableDrivePID == true) {
    vex::task::sleep(10);
  }



  //Set desired location moves forward 5 inch and left 90 deg.
  // When it finishes, continue to next movement
  // resetPID_Sensors = true;
  // enableDrivePID = true;
  // drivePID(nextX, nextY, nextHeading);
  // while (robotPosition[0] != 150 and robotPosition[1] != 90 and robotPosition[2] != 50) {
  //   drivePID(150, 90, 50);
  // }
  // resetPID_Sensors = true;
}

void userControl(void) {
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  Brain.Screen.print("autonomous code");
  IntakeMotor.setVelocity(100, percent);
  UpperMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  IntakeMotor.setStopping(brake);
  Controller1.ButtonA.pressed(Scraper);
  Controller1.ButtonB.pressed(Descore);
  Controller1.ButtonX.pressed(Negus);
  Controller1.ButtonY.pressed(NoNegus);
  isAutonomous = false;
  isDriverControl = true;

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
  IntakeMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);
  
  
  // Run the pre-autonomous function.
  preAutonomous();

  while (isAutonomous == false && isDriverControl == false) {
    scraper.set(false);
    wait(100, msec);
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {

    wait(100, msec);
  }
}

