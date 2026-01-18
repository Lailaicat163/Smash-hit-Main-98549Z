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
#define BUFFER 50 //Last 50 values


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
motor leftMotorA = motor(PORT12, ratio6_1, true);
motor leftMotorB = motor(PORT14, ratio6_1, true);
motor leftMotorC = motor(PORT13, ratio6_1, true); //5.5 watt
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);

motor rightMotorA = motor(PORT15, ratio6_1, false);
motor rightMotorB = motor(PORT19, ratio6_1, false);
motor rightMotorC = motor(PORT18, ratio6_1, false); //5.5 watt
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 299.24, 266.7, 190.5, mm, 1);
   
controller Controller1 = controller(primary);

motor intakeMotorA = motor(PORT2, ratio6_1, false);
motor intakeMotorB = motor(PORT17, ratio6_1, true);
motor_group intakeMotor = motor_group(intakeMotorA, intakeMotorB);

motor hoodMotor = motor(PORT11, ratio6_1, true);

// Pneumatics
//Digital in is scraper
digital_out descore = digital_out(Brain.ThreeWirePort.H);
//Digital in is descore and digital out is hood
digital_out hood = digital_out(Brain.ThreeWirePort.A);
digital_out scraper = digital_out(Brain.ThreeWirePort.F);
//Initializes the rotational sensors. Set true to inverse the rotation and velocity to negative values.
rotation rotationalLateral = rotation(PORT20, false);
rotation rotationalHorizontal = rotation(PORT4, true);

//Initializes the inertial sensor. starts from 0 degrees and increases by turning clockwise.
inertial inertialSensor = inertial(PORT5);

// //Initializes the distance sensors (4)
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

//Set motion variables
double maxAcceleration = 15;
double maxJerk = 15;

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
      int left = Controller1.Axis3.position() + Controller1.Axis1.position();
      int right = Controller1.Axis3.position() - Controller1.Axis1.position();

      int drivetrainLeftSideSpeed = (left * abs(left)) / 100;
      int drivetrainRightSideSpeed = (right * abs(right)) / 100;
      // caculates left and right side speed based on quadratic curve so it is less sensitive
      // int leftAxis = Controller1.Axis3.position();
      // int rightAxis = Controller1.Axis2.position();
      // int drivetrainLeftSideSpeed = (leftAxis * abs(leftAxis)) / 100;
      // int drivetrainRightSideSpeed = (rightAxis * abs(rightAxis)) / 100;
      
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
double leftMotorVelocity = 0;
double rightMotorVelocity = 0;

double desiredX;
double desiredY;
double desiredTheta;
double desiredLinearVelocity = 0;
double desiredTurningVelocity = 0;
bool direction = false;

//Ramsete ramseteControl function
int ramseteControl() {
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
  return 0;
}

//Motion Profile (contains acceleration, jerk)


// Gets current velocity and acceleration for the robot
int motionCounter = 0;
int bufferIndex;
int prevBufferIndex;
//Velocity
double leftRobotVelocity[BUFFER] = {};
double rightRobotVelocity[BUFFER] = {};
//Acceleration
double leftRobotAcceleration[BUFFER] = {};
double rightRobotAcceleration[BUFFER] = {};

int getMotion() {
  while(true) {
    bufferIndex = motionCounter % BUFFER;
    int prevBufferIndex = (bufferIndex - 1 + BUFFER) % BUFFER;
    double dt = 0.02; //Change in time 20 milliseconds

    //velocity
    double leftVelocity = (LeftDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    double rightVelocity = (RightDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    
    leftRobotVelocity[bufferIndex] = leftVelocity;
    rightRobotVelocity[bufferIndex] = rightVelocity;

    //acceleration delta v over delta t
    if(motionCounter > 0) {
    double leftAcceleration = (leftRobotVelocity[bufferIndex] - leftRobotVelocity[prevBufferIndex]) / dt;
    double rightAcceleration = (rightRobotVelocity[bufferIndex] - rightRobotVelocity[prevBufferIndex]) / dt;

    leftRobotAcceleration[bufferIndex] = leftAcceleration;
    rightRobotAcceleration[bufferIndex] = rightAcceleration;
    } else { //First iteration
    leftRobotAcceleration[bufferIndex] = 0;
    rightRobotAcceleration[bufferIndex] = 0;      
    }
    motionCounter++;
    vex::task::sleep(20);
  }
  return 0;
}

// Get most recent velocity
double getCurrentLeftVelocity() {
  int index = (motionCounter - 1) % BUFFER;
  if (index < 0) index += BUFFER;
  return leftRobotVelocity[index];
}

// Get most recent acceleration
double getCurrentLeftAcceleration() {
  int index = (motionCounter - 1) % BUFFER;
  if (index < 0) index += BUFFER;
  return leftRobotAcceleration[index];
}

// Get most recent velocity
double getCurrentRightVelocity() {
  int index = (motionCounter - 1) % BUFFER;
  if (index < 0) index += BUFFER;
  return rightRobotVelocity[index];
}

// Get most recent acceleration
double getCurrentRightAcceleration() {
  int index = (motionCounter - 1) % BUFFER;
  if (index < 0) index += BUFFER;
  return rightRobotAcceleration[index];
}

//inches/second
int motionProfile() {
  // Track target accelerations that evolve over time
  double targetLeftAccel = 0;
  double targetRightAccel = 0;
  
  while(true) {
    double currentLeftVelocity = getCurrentLeftVelocity();
    double currentRightVelocity = getCurrentRightVelocity();
    double leftVelocityError = leftMotorVelocity - currentLeftVelocity;
    double rightVelocityError = rightMotorVelocity - currentRightVelocity;

    // Check if we're close enough to target - if so, just idle
    if (fabs(leftVelocityError) <= 0.1 && fabs(rightVelocityError) <= 0.1) {
      // Reset target accelerations when at target
      targetLeftAccel = 0;
      targetRightAccel = 0;
      vex::task::sleep(20);
      continue; // Skip the rest and just sleep
    }
    
    double dt = 0.02; // 20ms time step
    
    // Calculate what acceleration we need to reach target velocity
    double desiredLeftAccel = leftVelocityError / dt;
    double desiredRightAccel = rightVelocityError / dt;
    
    // Calculate how much we need to change our current target acceleration
    double leftAccelChange = desiredLeftAccel - targetLeftAccel;
    double rightAccelChange = desiredRightAccel - targetRightAccel;
    
    // Apply jerk limiting (max change in acceleration per time step)
    double maxJerkStep = maxJerk * dt;
    
    //Left jerk limiting
    if (leftAccelChange > maxJerkStep) {
      leftAccelChange = maxJerkStep;
    } else if (leftAccelChange < -maxJerkStep) {
      leftAccelChange = -maxJerkStep;
    }
    
    //Right jerk limiting
    if (rightAccelChange > maxJerkStep) {
      rightAccelChange = maxJerkStep;
    } else if (rightAccelChange < -maxJerkStep) {
      rightAccelChange = -maxJerkStep;
    }
    
    //Update target accelerations
    targetLeftAccel += leftAccelChange;
    targetRightAccel += rightAccelChange;
    
    //Set the hard cap on how much the robot can accelerate
    //Left
    if (targetLeftAccel > maxAcceleration) {
      targetLeftAccel = maxAcceleration;
    } else if (targetLeftAccel < -maxAcceleration) {
      targetLeftAccel = -maxAcceleration;
    }
    
    //Right
    if (targetRightAccel > maxAcceleration) {
      targetRightAccel = maxAcceleration;
    } else if (targetRightAccel < -maxAcceleration) {
      targetRightAccel = -maxAcceleration;
    }
    
    //Calculate new velocity values from acceleration
    double commandedLeftVelocity = currentLeftVelocity + targetLeftAccel * dt;
    double commandedRightVelocity = currentRightVelocity + targetRightAccel * dt;
    
    //Clamp to not overshoot target velocity
    //Left side clamping
    if (leftVelocityError > 0 && commandedLeftVelocity > leftMotorVelocity) {
      commandedLeftVelocity = leftMotorVelocity;
    } else if (leftVelocityError < 0 && commandedLeftVelocity < leftMotorVelocity) {
      commandedLeftVelocity = leftMotorVelocity;
    }
    
    //Right side clamping
    if (rightVelocityError > 0 && commandedRightVelocity > rightMotorVelocity) {
      commandedRightVelocity = rightMotorVelocity;
    } else if (rightVelocityError < 0 && commandedRightVelocity < rightMotorVelocity) {
      commandedRightVelocity = rightMotorVelocity;
    }
    
    //percent conversions (600 RPM = 100% for 11-watt motors)
    commandedLeftVelocity = (((commandedLeftVelocity / WHEEL_CIRCUMFERENCE) * 60.0 / gearRatio) / 600.0) * 100.0; // Convert to percent
    commandedRightVelocity = (((commandedRightVelocity / WHEEL_CIRCUMFERENCE) * 60.0 / gearRatio) / 600.0) * 100.0; // Convert to percent
    
    //Command motors
    LeftDriveSmart.spin(forward, commandedLeftVelocity, vex::percentUnits::pct);
    RightDriveSmart.spin(forward, commandedRightVelocity, vex::percentUnits::pct);
    
    vex::task::sleep(20);
  }
  return 0;
}

//testing max acceleration and jerk then printing it on brain screen
// Add these functions to your code

// Test function to find maximum acceleration
void testMaxAcceleration() {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Testing Max Acceleration...");
  
  // Reset position and stop motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  wait(500, msec);
  
  // Arrays to store velocity samples
  const int samples = 100;
  double velocities[samples];
  double timeStamps[samples];
  
  // Record start time
  double startTime = Brain.Timer.time(msec);
  
  // Apply full power and collect velocity data
  LeftDriveSmart.spin(forward, 100, percent);
  RightDriveSmart.spin(forward, 100, percent);
  
  for(int i = 0; i < samples; i++) {
    timeStamps[i] = (Brain.Timer.time(msec) - startTime) / 1000.0; // Convert to seconds
    double leftVel = (LeftDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    double rightVel = (RightDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    velocities[i] = (leftVel + rightVel) / 2.0; // Average velocity in inches/sec
    wait(20, msec);
  }
  
  // Stop motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  
  // Calculate maximum acceleration (change in velocity / change in time)
  double maxAccel = 0;
  for(int i = 1; i < samples; i++) {
    double dt = timeStamps[i] - timeStamps[i-1];
    if(dt > 0) {
      double accel = (velocities[i] - velocities[i-1]) / dt;
      if(accel > maxAccel) {
        maxAccel = accel;
      }
    }
  }
  
  // Display result
  Brain.Screen.clearScreen();
  Brain.Screen.print("Max Acceleration: ");
  Brain.Screen.print(maxAccel);
  Brain.Screen.print(" in/s²");
  wait(3, seconds);
}

// Test function to find maximum jerk
void testMaxJerk() {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Testing Max Jerk...");
  
  // Reset position and stop motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  wait(500, msec);
  
  // Arrays to store acceleration samples
  const int samples = 100;
  double accelerations[samples];
  double timeStamps[samples];
  double velocities[samples];
  
  // Record start time
  double startTime = Brain.Timer.time(msec);
  
  // Apply full power and collect velocity data
  LeftDriveSmart.spin(forward, 100, percent);
  RightDriveSmart.spin(forward, 100, percent);
  
  // First pass: collect velocities
  for(int i = 0; i < samples; i++) {
    timeStamps[i] = (Brain.Timer.time(msec) - startTime) / 1000.0; // Convert to seconds
    double leftVel = (LeftDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    double rightVel = (RightDriveSmart.velocity(rpm) / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
    velocities[i] = (leftVel + rightVel) / 2.0; // Average velocity in inches/sec
    wait(20, msec);
  }
  
  // Stop motors
  LeftDriveSmart.stop();
  RightDriveSmart.stop();
  
  // Second pass: calculate accelerations
  for(int i = 1; i < samples; i++) {
    double dt = timeStamps[i] - timeStamps[i-1];
    if(dt > 0) {
      accelerations[i] = (velocities[i] - velocities[i-1]) / dt;
    } else {
      accelerations[i] = 0;
    }
  }
  accelerations[0] = 0; // First sample has no previous data
  
  // Third pass: calculate maximum jerk (change in acceleration / change in time)
  double maxJerk = 0;
  for(int i = 2; i < samples; i++) {
    double dt = timeStamps[i] - timeStamps[i-1];
    if(dt > 0) {
      double jerk = (accelerations[i] - accelerations[i-1]) / dt;
      if(fabs(jerk) > maxJerk) {
        maxJerk = fabs(jerk);
      }
    }
  }
  
  // Display result
  Brain.Screen.clearScreen();
  Brain.Screen.print("Max Jerk: ");
  Brain.Screen.print(maxJerk);
  Brain.Screen.print(" in/s³");
  wait(3, seconds);
}

// Combined test function that runs both tests
void testMotionLimits() {
  Brain.Screen.clearScreen();
  Brain.Screen.print("Starting Motion Tests...");
  wait(1, seconds);
  
  // Test acceleration
  testMaxAcceleration();
  
  // Wait between tests
  wait(1, seconds);
  
  // Test jerk
  testMaxJerk();
  
  Brain.Screen.clearScreen();
  Brain.Screen.print("Motion tests complete!");
}


//go to function for path making
void goTo(double x, double y, double heading, double linearVelocity, double turningVelocity, bool forward) {
  desiredX = x;
  desiredY = y;
  desiredTheta = heading;
  desiredLinearVelocity = linearVelocity;
  desiredTurningVelocity = turningVelocity;
  direction = forward;

  // Wait until we reach the target
  while(true) {
    double xError = desiredX - robotPosition.at(0,0);
    double yError = desiredY - robotPosition.at(1,0);
    double positionError = sqrt(xError * xError + yError * yError);
    
    // Calculate heading error properly (wraps around 360 degrees)
    double headingError = desiredTheta - robotPosition.at(2,0);
    // Normalize to [-180, 180] range
    headingError = atan2(sin(headingError * radianConversion), cos(headingError * radianConversion)) / radianConversion;
    headingError = fabs(headingError);
    
    // Check if we've reached target
    if (positionError < 0.5 && headingError < 2.0) { // Within 0.5 inches and 2 degrees
      break;
    }

    vex::task::sleep(20);
  }
  
  // Stops at target
  desiredLinearVelocity = 0;
  desiredTurningVelocity = 0;
  vex::task::sleep(100); //Let robot come still
}
//Path Making

//Path

void setStartPosition(double x, double y, double heading) {
  robotPosition.at(0,0) = x;
  robotPosition.at(0,1) = y;
  robotPosition.at(0,2) = heading;
  goTo(robotPosition.at(0,0), robotPosition.at(0,1), robotPosition.at(0,2), 0, 0, true);
  vex::task ramseteTask();
}

//Red
int redDriveForwardPath() {
  Drivetrain.driveFor(4, vex::distanceUnits::in);
  return 0;
}

int redEast1GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
}

int redEast2GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);  
  return 0;
} 

int redWest1GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);  
  return 0;
} 

int redWest2GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
} 

//Blue
int blueDriveForwardPath() {
  Drivetrain.driveFor(4, vex::distanceUnits::in);
  return 0;
}

int blueEast1GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
}

int blueEast2GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
}

int blueWest1GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
}

int blueWest2GoalPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
  goTo(1,1,1,1,1, true);
  return 0;
}

int autoSkillsPath() {
  setStartPosition(20, 90, 90);
  //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
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
bool descoreState = false;
bool scraperState = false;
bool hoodState = false;

void Scraper(){
    if (scraperState == true){
      scraper.set(false);
      scraperState = false;
    }
    else{
      scraper.set(true);
      scraperState = true;
    }
}

void Descore(){
  if (descoreState == true){
    descore.set(false);
    descoreState = false;
  }
  else{
    descore.set(true);
    descoreState = true;
  }
}

void hoodStopper(){
  if (hoodState == true) {
    hood.set(false);
    hoodState = false;
  }
  else{
    hood.set(true);
    hoodState = true;
  }
}

bool DescoreState = false;
bool ScrapperCooling = false;
bool DescoreCooling = false;
bool hoodCooling = false;

bool slowMode = false;
    

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

//following codes are for input purpose, if there are some bugs exist, 
//then put the codes back to userc=Control
void input() {
  static bool L1WasPressed = false;
  static bool XWasPressed = false;
  static bool DownWasPressed = false;
  
  // Intake control - R1 forward, R2 reverse
  if(Controller1.ButtonR1.pressing()) {
    intakeMotor.setVelocity(100, percent);
    hoodMotor.setVelocity(100, percent);
    intakeMotor.spin(forward);
    hoodMotor.spin(forward);
  } 
  else if(Controller1.ButtonR2.pressing()) {
    intakeMotor.setVelocity(slowMode ? 25 : 100, percent);
    hoodMotor.setVelocity(slowMode ? 25 : 100, percent);
    intakeMotor.spin(reverse);
    hoodMotor.spin(reverse);
  }
  // ButtonUp for high goal scoring
  else if(Controller1.ButtonUp.pressing()) {
    intakeMotor.setVelocity(100, percent);
    hoodMotor.setVelocity(100, percent);
    intakeMotor.spin(forward);
    hoodMotor.spin(forward);
  }
  // ButtonLeft for middle goal scoring
  else if(Controller1.ButtonLeft.pressing()) {
    intakeMotor.setVelocity(slowMode ? 25 : 100, percent);
    hoodMotor.setVelocity(slowMode ? 25 : 100, percent);
    intakeMotor.spin(forward);
    hoodMotor.spin(reverse);  // Hood goes opposite direction
  }
  // Only stop if NO motor buttons are pressed
  else {
    intakeMotor.stop();
    hoodMotor.stop();
  }
  
  // L2 for slow mode
  slowMode = Controller1.ButtonL2.pressing();
  
  // Scraper toggle with L1 (debounced)
  if(Controller1.ButtonL1.pressing() && !L1WasPressed) {
    Scraper();
    L1WasPressed = true;
  } else if (!Controller1.ButtonL1.pressing()) {
    L1WasPressed = false;
  }
  
  // Descore toggle with X button (debounced)
  if(Controller1.ButtonX.pressing() && !XWasPressed) {
    Descore();
    XWasPressed = true;
  } else if (!Controller1.ButtonX.pressing()) {
    XWasPressed = false;
  }

  if(Controller1.ButtonDown.pressing() && !DownWasPressed) {
    hoodStopper();
    DownWasPressed = true;
  } else if (!Controller1.ButtonDown.pressing()) {
    DownWasPressed = false;
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
bool isAutonomous = true;
int xpos = -1;

int trackTouch() {
  while (isAutonomous) {
    if (Brain.Screen.pressing()) {
      xpos = Brain.Screen.xPosition();
    } else {
      xpos = -1;
    }
      wait(20, msec);
  }
  return 0;
}

char preAutonSelector;
bool isLeft = false;
bool isRight = false;

void blueSelectGoalNumber() {
  vex::task::sleep(400);
  if (xpos >= 0 && xpos < 240 && isLeft == true) { // Left 1 goal
    preAutonSelector = 'E';
  } else if (xpos >= 240 && xpos <= 480 && isLeft == true) { // Left 2 goal
    preAutonSelector = 'D';
  } else if (xpos >= 0 && xpos < 240 && isRight == true) { // Right 1 goal
    preAutonSelector = 'C';
  } else if (xpos >= 240 && xpos <= 480 && isRight == true) { // Right 1 goal
    preAutonSelector = 'B';
  }
}

void redSelectGoalNumber() {
  vex::task::sleep(400);
  if (xpos >= 0 && xpos < 240 && isLeft == true) { // Left 1 goal
    preAutonSelector = 'I';
  } else if (xpos >= 240 && xpos <= 480 && isLeft == true) { // Left 2 goal
    preAutonSelector = 'J';
  } else if (xpos >= 0 && xpos < 240 && isRight == true) { // Right 1 goal
    preAutonSelector = 'G';
  } else if (xpos >= 240 && xpos <= 480 && isRight == true) { // Right 1 goal
    preAutonSelector = 'H';
  }
}

void blueSelect () {
  vex::task::sleep(400);
  if (xpos >= 0 && xpos < 160) { // For Left Side
    Brain.Screen.clearScreen();
    isLeft = true;
    
    //1 Goal Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Left 1 Goal");
    
    //2 Goals Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.print("Left 2 Goals");
    
    Brain.Screen.pressed(blueSelectGoalNumber);
  } else if (xpos >= 160 && xpos < 320) { // For Right Side
    Brain.Screen.clearScreen();
    isRight = true;
    
    //1 Goal Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Right 1 Goal");
    
    //2 Goals Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.print("Right 2 Goals");
    
    Brain.Screen.pressed(blueSelectGoalNumber);
  } else if (xpos >= 320) { // For Going Forward
    preAutonSelector = 'A';
  }
}

void redSelect () {
  vex::task::sleep(400);
  if (xpos >= 0 && xpos < 160) { // For Left Side
    Brain.Screen.clearScreen();
    isLeft = true;
    
    //1 Goal Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Left 1 Goal");
    
    //2 Goals Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.print("Left 2 Goals");
    
    Brain.Screen.pressed(redSelectGoalNumber);
  } else if (xpos >= 160 && xpos < 320) { //For Right Side
    Brain.Screen.clearScreen();
    isRight = true;
    
    //1 Goal Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Right 1 Goal");
    
    //2 Goals Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.print("Right 2 Goals");
    
    Brain.Screen.pressed(redSelectGoalNumber);
  } else if (xpos >= 320) { // For Going Forward
    preAutonSelector = 'F';
  }
}

void skillsSelect () {
  vex::task::sleep(400);

  if (xpos >= 0 && xpos < 240) {
    preAutonSelector = 'L';
  } else if (xpos >= 240 && xpos <= 480) {
    preAutonSelector = 'K';
  }
}

void secondPage() {
  vex::task::sleep(400);
  if (xpos >= 0 && xpos < 160) { // Red Side
    Brain.Screen.clearScreen();
    
    //Left Side Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 160, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Left Side");
    
    //Right Side Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(160, 0, 160, 240);
    Brain.Screen.setCursor(5, 16);
    Brain.Screen.print("Right Side");
    
    //Drive Forward Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(320, 0, 160, 240);
    Brain.Screen.setCursor(5, 32);
    Brain.Screen.print("Drive Forward");
    
    Brain.Screen.pressed(redSelect);
  } else if (xpos >= 160 && xpos < 320) { // Blue Side
    Brain.Screen.clearScreen();
    
    //Left Side Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 160, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Left Side");
    
    //Right Side Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(160, 0, 160, 240);
    Brain.Screen.setCursor(5, 16);
    Brain.Screen.print("Right Side");
    
    //Drive Forward Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(320, 0, 160, 240);
    Brain.Screen.setCursor(5, 32);
    Brain.Screen.print("Drive Forward");
    
    Brain.Screen.pressed(blueSelect);
  } else if (xpos >= 320) { // For Skills
    Brain.Screen.clearScreen();
    
    //Driver Skills Button
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0, 0, 240, 240);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Driver Skills");
    
    //Auton Skills Button
    Brain.Screen.setFillColor(purple);
    Brain.Screen.drawRectangle(240, 0, 240, 240);
    Brain.Screen.setCursor(5, 24);
    Brain.Screen.print("Auton Skills");
    
    Brain.Screen.pressed(skillsSelect);
  }
}

void firstPage() {
  //Red Button
  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(0, 0, 160, 240);
  Brain.Screen.setCursor(5, 1);
  Brain.Screen.print("Red Side");
  
  //Blue Button
  Brain.Screen.setFillColor(blue);
  Brain.Screen.drawRectangle(160, 0, 160, 240);
  Brain.Screen.setCursor(5, 16);
  Brain.Screen.print("Blue Side");
  
  //Skills Button
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(320, 0, 160, 240);
  Brain.Screen.setCursor(5, 32);
  Brain.Screen.print("Skills Run");
  
  //Check if brain pressed
  Brain.Screen.pressed(secondPage);
}

void preAutonomous(void) {
  // actions to do when the program starts
  //Set initial brain state
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
  
  vex::task trackTouch_Thread(trackTouch);
  //Calls auton selector
  //firstPage();
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
  
  intakeMotor.setVelocity(100, percent);
  hoodMotor.setVelocity(100, percent);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  intakeMotor.setStopping(brake);
  hoodMotor.setStopping(brake);
  
  if (inertialSensor.isCalibrating() == false) {
    inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
    robotPosition.at(2,0) = 90;
    inertialSensor.resetRotation();
  }
  
  Controller1.ButtonX.pressed(Negus);
  scraperState = false;
  hood.set(false);
  inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation

  Drivetrain.driveFor(forward, 5, inches);

  // vex::task odometryTest_Thread(odometryTest);
  // vex::task odometry_Thread(odometry);
  // vex::task getMotion_Thread(getMotion); // Start motion tracking
  // vex::task motionProfile_Thread(motionProfile); // Start motion profile
  //vex::task graph_Thread(graphTask);
  
  // place automonous code here
  //Drivetrain.driveFor(forward, 10, inches);
  // Drivetrain.driveFor(reverse, 45, inches);
  // switch (preAutonSelector) {
  //   case 'A': // Blue Drive Forward
  //     blueDriveForwardPath();
  //     isAutonomous = false;
  //     break;
  //   case 'B': // Blue East 1 Goal
  //     blueEast1GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'C': // Blue East 2 Goal
  //     blueEast2GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'D': // Blue West 1 Goal
  //     blueWest1GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'E': // Blue West 2 Goal
  //     blueWest2GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'F': // Red Drive Forward
  //     redDriveForwardPath();
  //     isAutonomous = false;
  //     break;
  //   case 'G': // Red East 1 Goal
  //     redEast1GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'H': // Red East 2 Goal
  //     redEast2GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'I': // Red West 1 Goal
  //     redWest1GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'J': // Red West 2 Goal
  //     redWest2GoalPath();
  //     isAutonomous = false;
  //     break;
  //   case 'K': // Auton Skills
  //     autoSkillsPath();
  //     isAutonomous = false;
  //     break;
  //   case 'L': // Driver Skills
  //     break;
  //   default: // The default case handles invalid operators
  //     break;
  // }
  
}

void userControl(void) {
  vex::task odometry_Thread(odometry);
  // if (inertialSensor.isCalibrating() == false) {
  //     inertialSensor.setHeading(90, degrees); //sets the heading to 90 degrees to match field orientation
  //     inertialSensor.resetRotation();
  // }
  enableDrivePID = false; //disables PID control during user control
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.setTurnVelocity(100, percent);
  Brain.Screen.print("driver control");
    
  intakeMotor.setVelocity(100, percent);
  hoodMotor.setVelocity(100, percent);
  LeftDriveSmart.setStopping(brake);
  RightDriveSmart.setStopping(brake);
  intakeMotor.setStopping(brake);
    
  // Controller1.ButtonA.pressed(Scraper);
  // Controller1.ButtonB.pressed(Descore);
  // Controller1.ButtonX.pressed(Negus);
  // Controller1.ButtonY.pressed(NoNegus);
  hoodMotor.setStopping(brake);
  intakeMotor.setStopping(brake);
  intakeMotor.setVelocity(100, percent);
    
  // place driver control in this while loop

  while(true){
    // Display position info periodically
    // static int displayCounter = 0;
    // if (displayCounter % 400 == 0) { // Every ~4 seconds (400 * 10ms)
    //   Brain.Screen.clearScreen();
    //   Brain.Screen.print("x: ");
    //   Brain.Screen.print(robotPosition.at(0,0));
    //   Brain.Screen.print(" y: ");
    //   Brain.Screen.print(robotPosition.at(1,0));
    //   Brain.Screen.print(" Heading: ");
    //   Brain.Screen.print(robotPosition.at(2,0));
    // }
    // displayCounter++;
        
    input();

    //testMotionLimits();
        
  vex::task::sleep(10);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // create competition instance
  competition Competition;
  intakeMotor.setVelocity(100, percent);  

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

