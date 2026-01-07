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
#define WHEEL_CIRCUMFERENCE 3.25 * PI  //inches
#define DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL 7 / 16  //inches
#define DISTANCE_FROM_CENTER_TO_HORIZONTAL_WHEEL 3  //inches
#define TRACK_WIDTH 10.75  //inches

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
motor leftMotorA = motor(PORT3, ratio6_1, true);
motor leftMotorB = motor(PORT4, ratio6_1, true);
motor leftMotorC = motor(PORT20, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT1, ratio6_1, false);
motor rightMotorB = motor(PORT2, ratio6_1, false);
motor rightMotorC = motor(PORT6, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 266.7, 190.5, mm, 1);
   
controller Controller1 = controller(primary);

motor IntakeMotor = motor(PORT12, ratio36_1, true);
motor UpperMotor = motor(PORT10, ratio36_1, true);
motor Second_IM = motor(PORT14, ratio36_1, true);

//Pneumatic A is for the scraper mechanism
digital_out scraper = digital_out(Brain.ThreeWirePort.A);
digital_out descore = digital_out(Brain.ThreeWirePort.H);

//Initializes the rotational sensors. Set true to inverse the rotation and velocity to negative values.
rotation rotationalLateral = rotation(PORT13, false);
rotation rotationalHorizontal = rotation(PORT18, true);

//Initializes the inertial sensor. starts from 0 degrees and increases by turning clockwise.
inertial inertialSensor = inertial(PORT14);

//Initializes the distance sensors (4)
distance distanceFront = distance(PORT15);
distance distanceLeft = distance(PORT16);
distance distanceRight = distance(PORT17);

distance doubleParkMacro = distance(PORT20);


//sets max rpm of motors
float maxMotorPercentage = 100.0;
//radian conversion variable to reduce number of calculations
double radianConversion = PI / 180.0; //conversion factor from degrees to radians
//Sets gear ratio
double gearRatio = 3 / 5; // input gear teeth / output gear teeth

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
//Quintic bezier curve path making start
//////////////////////////////////////////////
//First define start and end points
//x, y
//calculate interpolation points
//t goes from 0 to 1
//start and end value are for individual coordinates. For example, start x and end x.
double getLinearInterpolationValue(double startValue, double endValue, double ratio) {
  return startValue + (endValue - startValue) * ratio;
}
struct Point {
  double x;
  double y;
};












//////////////////////////////////////////////
//Math Functions Start
//////////////////////////////////////////////
//Matrix Class contains multiplication function and creating matrix of given size
class matrix {
private:
  double** data;
  int rows;
  int cols;

public:
  // Constructor
  matrix(int r, int c) : rows(r), cols(c) {
  //Assigns memory for each row
    data = new double*[rows];
    for(int i = 0; i < rows; i++) {
  // Makes each row get its own column, which is an array
      data[i] = new double[cols];
  // Memset makes all values in the matrix 0 to initialize it and get no errors
  // User will add values for the matrix later
      memset(data[i], 0, cols * sizeof(double));
      }
  }

  // Destructor removes allocated memory to prevent memory leaks
  ~matrix() {
    for(int i = 0; i < rows; i++) {
      delete[] data[i];
    }
    delete[] data;
  }

  // Access elements
  double& at(int r, int c) {
    return data[r][c];
  }

  // Getters
  int getRows() { return rows; }
  int getCols() { return cols; }

  // Matrix multiplication: this * other
  matrix multiply(const matrix& other) {
  // Result will be (this->rows) x (other.cols)
  matrix result(rows, other.cols);

  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < other.cols; j++) {
      double sum = 0.0;
      for(int k = 0; k < cols; k++) {
        sum += data[i][k] * other.data[k][j];
      }
    result.data[i][j] = sum;
    }
  }
  return result;
  }
  matrix subtract(const matrix& other) {
  //You can only subtract matrices of the same size
  matrix result(rows, cols);
    
  for(int i = 0; i < rows; i++) {
    for(int j = 0; j < cols; j++) {
      result.data[i][j] = data[i][j] - other.data[i][j];
      }
    } 
    return result;
    }
};
/* How to use matrix class:
1. Define matrix size
matrix A(2, 3); // 2 rows, 3 columns
matrix B(3, 2); // 3 rows, 2 columns
2. Set values
A.at(0, 0) = 1.0; A.at(0, 1) = 2.0; A.at(0, 2) = 3.0; // first row. Has 3 columns
A.at(1, 0) = 4.0; A.at(1, 1) = 5.0; A.at(1, 2) = 6.0; // second row. Has 3 columns
 Do the same for matrix B
B.at(0, 0) = 7;   B.at(0, 1) = 8;
B.at(1, 0) = 9;   B.at(1, 1) = 10;
B.at(2, 0) = 11;  B.at(2, 1) = 12;
3. Multiply matrices
matrix C = A.multiply(B); // Resulting matrix C will be 2x2
4. Get values from new matrix C
double val = C.at(0, 0); // Access element at row 0, column 0
*/
//////////////////////////////////////////////
//Autonomous Functions
//////////////////////////////////////////////
//Error graph tracking for PID
// Graph settings (add to top of your file with other constants)
const int GRAPH_WIDTH = 480;   // Brain screen width
const int GRAPH_HEIGHT = 240;  // Brain screen height
const int GRAPH_MARGIN = 30;   // Space for labels
const int MAX_DATA_POINTS = 100; // Number of points to display

// Data storage arrays
double errorHistory[MAX_DATA_POINTS];
int currentDataIndex = 0;

// Scale settings
double maxErrorScale = 10.0; // Adjust based on your typical error range

void initializeGraph() {
  // Clear array
  for (int i = 0; i < MAX_DATA_POINTS; i++) {
    errorHistory[i] = 0;
  }
  currentDataIndex = 0;
}

void drawGraph() {
  Brain.Screen.clearScreen();
  
  // Draw axes
  Brain.Screen.setPenColor(white);
  Brain.Screen.drawLine(GRAPH_MARGIN, GRAPH_MARGIN, 
                        GRAPH_MARGIN, GRAPH_HEIGHT - GRAPH_MARGIN); // Y-axis
  Brain.Screen.drawLine(GRAPH_MARGIN, GRAPH_HEIGHT - GRAPH_MARGIN, 
                        GRAPH_WIDTH - GRAPH_MARGIN, GRAPH_HEIGHT - GRAPH_MARGIN); // X-axis
  
  // Draw zero line
  Brain.Screen.setPenColor(red);
  int zeroY = GRAPH_HEIGHT / 2;
  Brain.Screen.drawLine(GRAPH_MARGIN, zeroY, 
                        GRAPH_WIDTH - GRAPH_MARGIN, zeroY);
  
  // Draw data points
  Brain.Screen.setPenColor(green);
  int graphWidth = GRAPH_WIDTH - 2 * GRAPH_MARGIN;
  int graphHeight = GRAPH_HEIGHT - 2 * GRAPH_MARGIN;
  
  for (int i = 1; i < MAX_DATA_POINTS; i++) {
    // Calculate screen positions
    int x1 = GRAPH_MARGIN + (i - 1) * graphWidth / MAX_DATA_POINTS;
    int x2 = GRAPH_MARGIN + i * graphWidth / MAX_DATA_POINTS;
    
    // Scale error values to screen coordinates
    int y1 = zeroY - (errorHistory[i - 1] / maxErrorScale) * (graphHeight / 2);
    int y2 = zeroY - (errorHistory[i] / maxErrorScale) * (graphHeight / 2);
    
    // Clamp to screen bounds
    y1 = fmax(GRAPH_MARGIN, fmin(y1, GRAPH_HEIGHT - GRAPH_MARGIN));
    y2 = fmax(GRAPH_MARGIN, fmin(y2, GRAPH_HEIGHT - GRAPH_MARGIN));
    
    Brain.Screen.drawLine(x1, y1, x2, y2);
  }
  
  // Draw labels
  Brain.Screen.setPenColor(white);
  Brain.Screen.printAt(5, 15, "Error: %.2f", errorHistory[MAX_DATA_POINTS - 1]);
  Brain.Screen.printAt(5, GRAPH_HEIGHT - 10, "Max: %.1f", maxErrorScale);
  Brain.Screen.printAt(5, zeroY, "0");
}

void addDataPoint(double errorValue) {
  // Shift all data left
  for (int i = 0; i < MAX_DATA_POINTS - 1; i++) {
    errorHistory[i] = errorHistory[i + 1];
  }
  // Add new data at the end
  errorHistory[MAX_DATA_POINTS - 1] = errorValue;
}

// Task to continuously update graph
int graphTask() {
  initializeGraph();
  while (true) {
    drawGraph();
    vex::task::sleep(50); // Update every 50ms (20 Hz)
  }
  return 0;
}

//Initial Positional Vector of the Robot
matrix robotPosition{3, 1};  //three rows, 1 column. x, y, heading
double previousAngleRotation = 0;
double previousHeading = 0;
double previousLateralTravelled = 0;
double previousHorizontalTravelled = 0;

//Odometry
int odometry() {
  while(true) {
    //code for odometry here
    //Calculating delta values
    double deltaLateral = (rotationalLateral.position(deg) - previousLateralTravelled) * radianConversion; //inches
    double deltaHorizontal = (rotationalHorizontal.position(deg) - previousHorizontalTravelled) * radianConversion; //inches
    //gets change in orientation as an absolute value. This is for the arc angle
    double deltaOrientation = inertialSensor.rotation(deg) - previousAngleRotation; //in degrees
    double arcAngle = fabs(deltaOrientation); //in degrees
    double arcAngleRadians = arcAngle * radianConversion; //in radians
    //Make a check for small changes in order to avoid sensor noise
    const double movementThreshold = 0.01; //inches
    if (fabs(deltaLateral) < movementThreshold 
    && fabs(deltaHorizontal) < movementThreshold 
    && fabs(deltaOrientation) < movementThreshold) {
      //No significant movement detected; skip this iteration
      vex::task::sleep(10);
      continue;
    }
    //gets the radius of the tracking centers arc
    //radius = distance travelled of lateral tracking wheel / arc angle * (PI/180) (to convert to radians)
    double lateralArcRadius;

    //Gets the local chord length
    double localDeltaX, localDeltaY;
    if (fabs(deltaOrientation) <= 0.001) {
      localDeltaX = deltaHorizontal;
      localDeltaY = deltaLateral;
    } else {
      if (deltaOrientation < 0) {
        lateralArcRadius = deltaLateral / arcAngleRadians - DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL;
      } else if (deltaOrientation > 0) {
        lateralArcRadius = deltaLateral / arcAngleRadians + DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL;
      }
      double horizontalArcRadius = deltaHorizontal / arcAngleRadians + DISTANCE_FROM_CENTER_TO_HORIZONTAL_WHEEL;
      localDeltaX = 2 * horizontalArcRadius * sin(arcAngleRadians / 2);
      localDeltaY = 2 * lateralArcRadius * sin(arcAngleRadians / 2);
    }
    //Converts local changes to global changes
    double gridOffset = previousAngleRotation + (deltaOrientation / 2); //in degrees
    double gridOffsetRadians = gridOffset * radianConversion; //in radians
    robotPosition.at(0,0) += localDeltaX * cos(gridOffsetRadians) - localDeltaY * sin(gridOffsetRadians);
    robotPosition.at(1,0) += localDeltaX * sin(gridOffsetRadians) + localDeltaY * cos(gridOffsetRadians);
    robotPosition.at(2,0) = inertialSensor.heading(deg);
    //Updating previous values
    previousAngleRotation = inertialSensor.rotation(deg);
    previousHeading = inertialSensor.heading(deg);
    previousLateralTravelled = rotationalLateral.position(deg);
    previousHorizontalTravelled = rotationalHorizontal.position(deg);
    vex::task::sleep(10); //waits 100 milliseconds before next loop
  }
  return 0; //Doesn't matter what this returns
}

//Brain odometry output test
int odometryTest() {
  while (true) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("x: ");
  Brain.Screen.print(robotPosition.at(0,0));
  Brain.Screen.newLine();
  Brain.Screen.print("y: ");
  Brain.Screen.print(robotPosition.at(1,0));
  Brain.Screen.newLine();
  Brain.Screen.print("Heading: ");
  Brain.Screen.print(robotPosition.at(2,0));
  Brain.Screen.newLine();
  Brain.Screen.print(rotationalLateral.position(deg));
  Brain.Screen.newLine();
  Brain.Screen.print(rotationalHorizontal.position(deg));
  Brain.Screen.setCursor(1,1);
  vex::task::sleep(100);
  }
  return 0;
}

bool forwardWall;
bool rightWall;
bool backWall;
bool leftWall;

int distanceReset() {
  while(true) {
    // Returns true if there isn't a wall. Work based on this.
    forwardWall = distanceFront.isObjectDetected();
    rightWall = distanceFront.isObjectDetected();
    backWall = doubleParkMacro.isObjectDetected();
    leftWall = distanceLeft.isObjectDetected();

    
  }
  return 0;
}

int doublePark() {
  while(doubleParkMacro.objectDistance(inches) < 5) {

  }
  return 0;
}

//Path Following Functions
//Ramsete Controller

//Ramsete tweaking values
const double zeta = 0.7; //damping coefficient
const double b = 2.0;  //acts like a proportional gain

//values for this matrix is defined in the autonomous function
matrix localTransformationMatrix(3,3);

//Variables that controller will modify
double leftMotorVelocity;
double rightMotorVelocity;

//Ramsete goTo function
void goTo(double desiredX, double desiredY, double desiredTheta, double desiredLinearVelocity, double desiredTurningVelocity, bool direction) {
  //Start the while loop for the controller
  while(true) {
  //Update the local transformation matrix based on current robot heading
  localTransformationMatrix.at(0,0) = cos(robotPosition.at(2,0) * radianConversion); //row 1, col 1
  localTransformationMatrix.at(0,1) = sin(robotPosition.at(2,0) * radianConversion); //row 1, col 2
  localTransformationMatrix.at(0,2) = 0; //row 1, col 3
  localTransformationMatrix.at(1,0) = -sin(robotPosition.at(2,0) * radianConversion); //row 2, col 1
  localTransformationMatrix.at(1,1) = cos(robotPosition.at(2,0) * radianConversion); //row 2, col 2
  localTransformationMatrix.at(1,2) = 0; //row 2, col 3
  localTransformationMatrix.at(2,0) = 0; //row 3, col 1
  localTransformationMatrix.at(2,1) = 0; //row 3, col 2
  localTransformationMatrix.at(2,2) = 1; //row 3, col 3
  //Coordinate errors
  matrix targetPosition{3,1};
  targetPosition.at(0,0) = desiredX;
  targetPosition.at(1,0) = desiredY;
  targetPosition.at(2,0) = desiredTheta;
  matrix globalError = targetPosition.subtract(robotPosition);
  matrix localError = localTransformationMatrix.multiply(globalError);
  //k is a gain value for the controller that adjusts based on desired velocities
  double k = 2 * zeta * sqrt(desiredTurningVelocity * desiredTurningVelocity + b * desiredLinearVelocity * desiredLinearVelocity);
  double velocity = desiredLinearVelocity * cos(localError.at(2,0) * radianConversion) + k * localError.at(0,0);
  double angleErrorTerm = localError.at(2,0) * radianConversion;
  //Prevents division by zero when the angle error is very small
  double sinc_term;
  if (fabs(angleErrorTerm) < 0.001) {
    sinc_term = 1.0;  // Limit as θ -> 0
  } else {
    sinc_term = sin(angleErrorTerm) / angleErrorTerm;
  }
  double angularVelocity = desiredTurningVelocity 
  + b * desiredLinearVelocity * sinc_term * localError.at(1,0)
  + k * localError.at(2,0) * radianConversion;
  //Calculate motor power
  double linearMotorVelocity = velocity / WHEEL_CIRCUMFERENCE;
  double wheelAngularContribution = (angularVelocity * TRACK_WIDTH) / (2 * WHEEL_CIRCUMFERENCE);
  leftMotorVelocity = linearMotorVelocity + wheelAngularContribution;
  rightMotorVelocity = linearMotorVelocity - wheelAngularContribution;
  // false is backward, true is forward
  if (direction == false) {
    leftMotorVelocity = -leftMotorVelocity;
    rightMotorVelocity = -rightMotorVelocity;
  }
  vex::task::sleep(20); //waits 20 milliseconds before next loop
  }
}

//Path Making

//Path 
int path() {
  goTo(1,1,1,1,1, true);
  return 0;
}


//PID Controller for tuning velocities given by the Ramsete controller

//Desired right and left velocities given by controller

//PID settings start
const double velocity_kP = 0.0;
const double velocity_kI = 0.0;
const double velocity_kD = 0.0;

//Error variables for velocity PID
//left side
double leftVelocity;
double leftVelocityError;
double leftPrevVelocityError = 0.0;
double leftDerivativeVelocityError;
double leftIntegralVelocityError = 0.0;
//right side
double rightVelocity;
double rightVelocityError;
double rightPrevVelocityError = 0.0;
double rightDerivativeVelocityError;
double rightIntegralVelocityError = 0.0;

//Velocity PID function
int velocityPID() {
  while(true) {
    //Velocity calculations
    leftVelocity = (LeftDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    rightVelocity = (RightDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    //Left side PID error calculations
    leftVelocityError = leftMotorVelocity - leftVelocity;
    leftDerivativeVelocityError = (leftVelocityError - leftPrevVelocityError) / 0.02;
    if (fabs(leftVelocityError) < 1.0) { //integral windup prevention. makes it so that integral only adds up when close to target
      leftIntegralVelocityError += leftVelocityError * 0.02;
    } else {
      leftIntegralVelocityError = 0.0;
    }
    //Right side PID error calculations
    rightVelocityError = rightMotorVelocity - rightVelocity;
    rightDerivativeVelocityError = (rightVelocityError - rightPrevVelocityError) / 0.02;
    if (fabs(rightVelocityError) < 1.0) { //integral windup prevention. makes it so that integral only adds up when close to target
      rightIntegralVelocityError += rightVelocityError * 0.02;
    } else {
      rightIntegralVelocityError = 0.0;
    }
    //Calculating motor powers
    double leftMotorPower = (leftVelocityError * velocity_kP) + (leftIntegralVelocityError * velocity_kI) + (leftDerivativeVelocityError * velocity_kD);
    double rightMotorPower = (rightVelocityError * velocity_kP) + (rightIntegralVelocityError * velocity_kI) + (rightDerivativeVelocityError * velocity_kD);
    //Setting motor powers
    LeftDriveSmart.spin(forward, leftMotorPower, percent);
    RightDriveSmart.spin(forward, rightMotorPower, percent);
    vex ::task::sleep(20); //waits 20 milliseconds before next loop
  }
}

//Start Driving Forward PID Controller
//For going straight
const double kP = 6;  //7
const double kI = 0;
const double kD = 0.01; //2 0.03

//For turning
const double turning_kP = 0; //less than .1 
const double turning_kI = 0; //should be a really small number 0.024 example
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

//PID function for driving
int drivePID() {
  while(enableDrivePID == true) {
    if (resetPID_Sensors == true) {
      //Resets the sensors and variables
      rotationalLateral.resetPosition();
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
    distanceError = targetDistance - totalDistanceTravelled;
    //Derivative and integral error calculations
    derivativeDistanceError = (distanceError - prevDistanceError) / 0.02;
    if (fabs(distanceError) < 5) { //integral windup prevention. makes it so that integral only adds up when close to target
      integralDistanceError += distanceError * 0.02;
    } else {
      integralDistanceError = 0;
    }
    //Calculation of motor power
    double motorPower = (distanceError * kP) + (integralDistanceError * kI) + (derivativeDistanceError * kD);
    //end forward movement error calculations
    // START Angular movement calculations
    headingError = desiredTurnValue - robotHeading;
    headingError = -1 * (atan2(sin(headingError * radianConversion), cos(headingError * radianConversion)) * 180.0 / PI);
    // Derivative
    double derivativeHeadingError = (headingError - prevHeadingError) / 0.02;

    if (fabs(headingError) < 15) { //integral windup prevention. makes it so that integral only adds up when close to target
      integralHeadingError += headingError * 0.02;
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
    totalDistanceTravelled = rotationalLateral.position(deg) * radianConversion;
    targetDistance = targetDistance - (totalDistanceTravelled - previousDistanceTravelled);
    addDataPoint(headingError);  // Or headingError, or motorPower, etc.
    if (fabs(targetDistance) < 0.1 && fabs(headingError) < 3 && timerCount < 20) { //If within 1 inches and 3 degree of target, stop motors and exit task
      timerCount += 1;
    } else if (fabs(targetDistance) > 0.1 || fabs(headingError) < 3) {
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






///////////////////////////////////////////////////////////////////////////////////////////////
//Controller input functions start here

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
// void Scraper(){
//     // wait(10, msec);
//     if (scraperState == true){
//       scraper.set(false);
//       scraperState = false;
//     }
//     else{
//       scraper.set(true);
//       scraperState = true;
//     }
//   }

// bool DescoreState = false;

// void Descore(){
//   if (DescoreState == true){
//     descore.set(false);
//     DescoreState = false;
//   }
//   else{
//     descore.set(true);
//     DescoreState = true;
//   }
// }

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
  rotationalLateral.resetPosition(); //resetting the rotational sensor position to 0
  rotationalHorizontal.resetPosition(); //resetting the rotational sensor position to 0
  //calibrating the inertial sensor MUST DO THIS
  inertialSensor.calibrate(); 
  while (inertialSensor.isCalibrating()) {
    wait(100, msec);
  }
  inertialSensor.resetRotation();
}



void autonomous(void) {
  //Robot Position Vector Initialization
  robotPosition.at(0,0) = 0; //x position in inches
  robotPosition.at(1,0) = 0; //y position in inches
  robotPosition.at(2,0) = 0; //heading in degrees
  rotationalHorizontal.resetPosition(); //resetting the rotational sensor position to 0
  rotationalLateral.resetPosition(); //resetting the rotational sensor position to 0
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  Brain.Screen.print("autonomous code");
  IntakeMotor.setVelocity(100, percent);
  UpperMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
  IntakeMotor.setStopping(brake);
  if (inertialSensor.isCalibrating() == false) {
    inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
    robotPosition.at(2,0) = 90;
    inertialSensor.resetRotation();
  }
  Controller1.ButtonX.pressed(Negus);
  scraperState = false;
  scraper.set(false);
  inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
  // place automonous code here
  // vex::task odometryTest_Thread(odometryTest);
  vex::task odometry_Thread(odometry);
  vex::task graph_Thread(graphTask);
}

void userControl(void) {
  vex::task odometry_Thread(odometry);
  while (true) {
  Brain.Screen.clearScreen();
  Brain.Screen.print("x: ");
  Brain.Screen.print(robotPosition.at(0,0));
  Brain.Screen.print("y: ");
  Brain.Screen.print(robotPosition.at(1,0));
  Brain.Screen.print("Heading: ");
  Brain.Screen.print(robotPosition.at(2,0));
  vex::task::sleep(4000);
  }
  if (inertialSensor.isCalibrating() == false) {
    inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
    inertialSensor.resetRotation();
  }
  enableDrivePID = false; //disables PID control during user control
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  Brain.Screen.print("driver control");
  IntakeMotor.setVelocity(100, percent);
  UpperMotor.setVelocity(100, percent);
  Second_IM.setVelocity(100, percent);
  LeftDriveSmart.setStopping(coast);
  RightDriveSmart.setStopping(coast);
  IntakeMotor.setStopping(brake);
  // Controller1.ButtonA.pressed(Scraper);
  // Controller1.ButtonB.pressed(Descore);
  // Controller1.ButtonX.pressed(Negus);
  // Controller1.ButtonY.pressed(NoNegus);

  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  IntakeMotor.setStopping(brake);
  IntakeMotor.setVelocity(100, percent);
  
  // place driver control in this while loop
  
  bool DescoreState = false;
  bool ScrapperCooling = false;
  bool DescoreCooling = false;
  double CoolingTime = 0;
  while(true){
    input();
    if (Controller1.ButtonA.pressing() == true){
      if (scraperState == true){
      scraper.set(false);
      scraperState = false;
      }
    else{
      scraper.set(true);
      scraperState = true;
      }
      ScrapperCooling = true;
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Cooling Down!");
      wait(1, seconds);
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      ScrapperCooling = false;
    }
    if (Controller1.ButtonB.pressing() == true){
      if (DescoreState == true){
        descore.set(false);
        DescoreState = false;
      }
      else{
        descore.set(true);
        DescoreState = true;
      }
      DescoreCooling = true;
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("Cooling Down!");
      wait(1, seconds);
      Controller1.Screen.clearLine(1);
      Controller1.Screen.setCursor(1,1);
      DescoreCooling = false;
    }
    vex::task::sleep(10);
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

  // Prevent main from exiting with an infinite loop.
  while (true) {

    wait(100, msec);
  }
}

