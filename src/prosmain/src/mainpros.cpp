#include "main.h"
#include "pros/apix.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

//////////////////////////////////////////////
// Define Settings
//////////////////////////////////////////////
#define PI 3.1415926535897
#define WHEEL_CIRCUMFERENCE (3.25 * PI)  //inches
#define TRACKING_WHEEL_CIRCUMFERENCE (2.0 * PI)  //inches
#define DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL 0.5  //inches
#define DISTANCE_FROM_CENTER_TO_HORIZONTAL_WHEEL 4.0  //inches
#define TRACK_WIDTH 10.75  //inches
#define BUFFER 50 //Last 50 values
#define MM_TO_INCHES 25.4 //Divide by 25.4 to convert mm to inches

#define DIST_LEFT_X 3.0 //X position of left distance sensor relative to robot center (inches) NOT IN USE
#define DIST_LEFT_Y 3.0 //Y position of left distance sensor relative to robot center (inches) NOT IN USE

#define DIST_RIGHT_X 4.5 //X position of right distance sensor relative to robot center (inches)
#define DIST_RIGHT_Y 0.5 //Y position of right distance sensor relative to robot center (inches)

#define DIST_FRONT_X 6.5 //X position of front distance sensor relative to robot center (inches)
#define DIST_FRONT_Y 1.5 //Y position of front distance sensor relative to robot center (inches)

#define DIST_BACK_X 3.0 //X position of back distance sensor relative to robot center (inches) NOT IN USE
#define DIST_BACK_Y 3.0 //Y position of back distance sensor relative to robot center (inches) NOT IN USE



//////////////////////////////////////////////
// Motor and Sensor Declarations
//////////////////////////////////////////////
pros::MotorGroup LeftDriveSmart({-14, -13, -18});
pros::MotorGroup RightDriveSmart({15, 19, 20}); //good
pros::MotorGroup intakeMotor({2, -17}); 
pros::Motor hoodMotor(-11);//good

// Controller
pros::Controller Controller1(pros::E_CONTROLLER_MASTER);

// Pneumatics
//Digital in is scraper
pros::adi::DigitalOut descore('E');
//Digital in is descore and digital out is hood
pros::adi::DigitalOut hood('F');
pros::adi::DigitalOut scraper('H');
pros::adi::DigitalOut odomLift('G');

//Initializes the rotational sensors. Set true to inverse the rotation and velocity to negative values.
pros::Rotation rotationalLateral(10);
pros::Rotation rotationalHorizontal(12);

//Initializes the inertial sensor. starts from 0 degrees and increases by turning clockwise.
pros::IMU inertialSensor(4);

//Initializes the color sensor.
pros::Optical opticalSensor(21);
//Red hue is from 350 to 10
//Blue hue is from 210 to 240

//Initializes the distance sensors (4)
pros::Distance distanceFront(9);
pros::Distance distanceLeft(3);
pros::Distance distanceRight(6);
pros::Distance distanceBack(5);

//sets max rpm of motors
float maxMotorPercentage = 100.0;
//radian conversion variable to reduce number of calculations
double radianConversion = PI / 180.0; //conversion factor from degrees to radians
//Sets gear ratio
double gearRatio = 3.0 / 5.0; // input gear teeth / output gear teeth

//Set motion variables
double maxAcceleration = 15;
double maxJerk = 15;

//////
//Brain screen with lvgl and auton selector
//////////
// class BrainScreen {
//     private:
//         lv_obj_t* frame;
//         lv_obj_t* autonIndicator;
//     struct autonOption {
//         char preAutonSelector;
//         std::string autonName;
//         bool isRedAlliance;
//         bool isSkills;
//         std::string numberOfGoals;

//     };


// };BrainScreen Brain;


//////////////////////
//Input
////////////////////

// Color sort
//true for red, false for blue

//Red hue is from 350 to 10
//Blue hue is from 210 to 240
int colorSort(bool isRed) {
    float hue = opticalSensor.get_hue();
    
    if (isRed) {
        // If red team, reject blue balls (210-240)
        if (hue >= 210 && hue <= 240) {
            hoodMotor.move(-127);
            pros::delay(200);
            return 1;
        }
    } else {
        // If blue team, reject red balls (350-360 or 0-10)
        if (hue >= 350 || hue <= 10) {
            hoodMotor.move(-127);
            pros::delay(200);
            return 1;
        }
    }
    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////
//Controller input functions start here
bool descoreState = false;
bool scraperState = false;
bool hoodState = false;
bool odomState = false;

void Scraper(){
    if (scraperState == true){
        scraper.set_value(false);
        scraperState = false;
    }
    else{
        scraper.set_value(true);
        scraperState = true;
    }
}

void Descore(){
    if (descoreState == true){
        descore.set_value(false);
        descoreState = false;
    }
    else{
        descore.set_value(true);
        descoreState = true;
    }
}

void hoodStopper(){
    if (hoodState == true) {
        hood.set_value(false);
        hoodState = false;
    }
    else{
        hood.set_value(true);
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
        Controller1.clear_line(0);
        Controller1.set_text(0, 0, "Stopping Robot...");
        // PROS doesn't have Brain.programStop(), so we just stop all motors
        LeftDriveSmart.brake();
        RightDriveSmart.brake();
        intakeMotor.brake();
        hoodMotor.brake();
    }
    else{
        Controller1.clear_line(0);
        Controller1.set_text(0, 0, "Stop robot? Y or X");
        NegusConfirmed = true;
    }
}

void NoNegus(){  
    if (NegusConfirmed == true){
        Controller1.clear_line(0);
        Controller1.set_text(0, 0, "Stopping Cancelled!");
        NegusConfirmed = false;
        pros::delay(1500);
        Controller1.clear_line(0);
    }
}

bool isRedTeam = false;
//following codes are for input purpose, if there are some bugs exist, 
//then put the codes back to userControl
void input() {
    static bool L1WasPressed = false;
    static bool XWasPressed = false;
    static bool DownWasPressed = false;
    static bool YWasPressed = false;
    
    // Intake control - R1 forward, R2 reverse
    if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intakeMotor.move(127);
        hoodMotor.move(127);
        colorSort(isRedTeam);
    } 
    else if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        int velocity = slowMode ? 40 : 127;
        intakeMotor.move(-velocity);
        hoodMotor.move(-velocity);
    }
    // ButtonUp for high goal scoring
    else if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        intakeMotor.move(127);
        hoodMotor.move(127);
    }
    // ButtonLeft for middle goal scoring
    else if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        int velocity = slowMode ? 40 : 127;
        intakeMotor.move(velocity);
        hoodMotor.move(-velocity);  // Hood goes opposite direction
    }
    // Only stop if NO motor buttons are pressed
    else {
        intakeMotor.brake();
        hoodMotor.brake();
    }
    
    // L2 for slow mode
    slowMode = Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    
    // Scraper toggle with L1 (debounced)
    if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !L1WasPressed) {
        Scraper();
        L1WasPressed = true;
    } else if (!Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        L1WasPressed = false;
    }
    
    // Descore toggle with X button (debounced)
    if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !XWasPressed) {
        Descore();
        XWasPressed = true;
    } else if (!Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        XWasPressed = false;
    }

    if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && !DownWasPressed) {
        hoodStopper();
        DownWasPressed = true;
    } else if (!Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        DownWasPressed = false;
    }

    // Odom Lift Toggle with Y button
    if(Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && !YWasPressed) {
        odomState = !odomState;
        odomLift.set_value(odomState);
        YWasPressed = true;
    } else if (!Controller1.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        YWasPressed = false;
    }
}


//////////////////////////////////////////////
//Math Functions Start
//////////////////////////////////////////////
//Matrix Class contains multiplication function and creating matrix of given size
class matrix {
private:
    std::vector<std::vector<double>> data;
    int rows;
    int cols;

public:
    // Constructor
    matrix(int r, int c) : rows(r), cols(c) {
        //Assigns memory for each row
        data.resize(rows);
        for(int i = 0; i < rows; i++) {
            // Makes each row get its own column, which is an array
            data[i].resize(cols, 0.0); // Initialize all values to 0
        }
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

//Start Monte Carlo predicted values functions. Covariance, mean, multivariance generation
// class Sample {
//   public: 
//     int numSamples;
//     int auto
    
// }
//Start variables for Monte Carlo Localization
// const int numberOfSamples = 200;
// double mean = 0.0;
// double std = 1.0;
// double time = 0.0; //For timestep
// struct Sample {
//     double x;
//     double y;
// } samples[numberOfSamples];
//Step 1: Prediction Phase
/*  In this step the filter iterates through each particle and predicts how the robot could have moved, 
given the control input or sensor readings, such as drivetrain encoders. */
//Make an array of predicted values with a standard distribution. Only x, y prediction.
// double predictedSamples[numberOfSamples][2]; //200 samples for x and y

// void update() {

// }

// void generateSamples() {
//   for(int i = 0, i < numberOfSamples; i++) {

//   }
// }

//Step 2: Update Phase
/*In this step the filter weighs each particle using the probability that you would get this sensor 
measurement given that the robot is at that position, then there is a resampling step to move 
particles to places with higher weights */

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
    // Note: PROS uses LVGL for screen drawing, not the same API as VEXcode
    // This function would need to be adapted to LVGL if you want graphing
    // For now, this is a placeholder that matches the VEXcode structure
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
void graphTask(void* param) {
    initializeGraph();
    while (true) {
        drawGraph();
        pros::delay(50); // Update every 50ms (20 Hz)
    }
}

//Initial Positional Vector of the Robot
matrix robotPosition(3, 1);  //three rows, 1 column. x, y, heading
double previousAngleRotation = 0;
double previousHeading = 0;
double previousLateralTravelled = 0;
double previousHorizontalTravelled = 0;

//Odometry
void odometry(void* param) {
    // Wait for sensors to settle, then seed previous values from current readings.
    // Without this, the first loop iteration computes a massive false delta
    // from 0 to whatever the sensor already reads.
    pros::delay(100);
    previousAngleRotation = inertialSensor.get_rotation();
    previousHeading = inertialSensor.get_heading();
    previousLateralTravelled = rotationalLateral.get_position() / 100.0;
    previousHorizontalTravelled = rotationalHorizontal.get_position() / 100.0;
    while(true) {
        //code for odometry here
        //Calculating delta values
        double deltaLateral = (rotationalLateral.get_position() / 100.0 - previousLateralTravelled) * (TRACKING_WHEEL_CIRCUMFERENCE / 360.0);
        double deltaHorizontal = (rotationalHorizontal.get_position() / 100.0 - previousHorizontalTravelled) * (TRACKING_WHEEL_CIRCUMFERENCE / 360.0);
        //gets change in orientation as an absolute value. This is for the arc angle
        double deltaOrientation = inertialSensor.get_rotation() - previousAngleRotation; //in degrees
        double arcAngle = fabs(deltaOrientation); //in degrees
        double arcAngleRadians = arcAngle * radianConversion; //in radians
        //Make a check for small changes in order to avoid sensor noise
        const double movementThreshold = 0.01; //inches
        if (fabs(deltaLateral) < movementThreshold 
        && fabs(deltaHorizontal) < movementThreshold 
        && fabs(deltaOrientation) < movementThreshold) {
            //No significant movement detected; skip this iteration
            pros::delay(10);
            continue;
        }
        //gets the radius of the tracking centers arc
        //radius = distance travelled of lateral tracking wheel / arc angle * (PI/180) (to convert to radians)
           double lateralArcRadius = 0;

        //Gets the local chord length
        double localDeltaX, localDeltaY;
        if (fabs(deltaOrientation) <= 0.001) {
            localDeltaX = deltaHorizontal;
            localDeltaY = deltaLateral;
        } else {
            if (deltaOrientation < 0) {
                lateralArcRadius = deltaLateral / arcAngleRadians - DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL;
            } else {
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
        robotPosition.at(2,0) = inertialSensor.get_heading();
        //Updating previous values
        previousAngleRotation = inertialSensor.get_rotation();
        previousHeading = inertialSensor.get_heading();
        previousLateralTravelled = rotationalLateral.get_position() / 100.0;
        previousHorizontalTravelled = rotationalHorizontal.get_position() / 100.0;
        pros::delay(10); //waits 100 milliseconds before next loop
    }
}

//Brain odometry output test
void odometryTest(void* param) {
    while (true) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "x: " + std::to_string(robotPosition.at(0,0)));
        pros::lcd::set_text(2, "y: " + std::to_string(robotPosition.at(1,0)));
        pros::lcd::set_text(3, "Heading: " + std::to_string(robotPosition.at(2,0)));
        pros::lcd::set_text(4, std::to_string(rotationalLateral.get_position() / 100.0));
        pros::lcd::set_text(5, std::to_string(rotationalHorizontal.get_position() / 100.0));
        pros::delay(100);
    }
}

//tune coefficients for distance sensors

bool frontWall;
bool rightWall;
bool backWall;
bool leftWall;

double leftDistanceCoefficient = 1.0;
int getLeftDistance() {
    if (distanceLeft.get_object_size() < 200) { // 2 inches in mm is approximately 50mm
        leftWall = true;
    } else {
        leftWall = false;
        return -1;
    }
    if (leftWall == true) {
        return distanceLeft.get() / MM_TO_INCHES* leftDistanceCoefficient;
    }
    return -1;
}

double frontDistanceCoefficient = 1.0;
int getFrontDistance() {
    if (distanceFront.get_object_size() < 200) { // 2 inches in mm is approximately 50mm
        frontWall = true;
    } else {
        frontWall = false;
        return -1;
    }
    if (frontWall == true) {
        return distanceFront.get() / MM_TO_INCHES * frontDistanceCoefficient;
    }
    return -1;
}

double rightDistanceCoefficient = 1.0;
int getRightDistance() {
    if (distanceRight.get_object_size() < 200) { // 2 inches in mm is approximately 50mm
        rightWall = true;
    } else {
        rightWall = false;
        return -1;
    }
    if (rightWall == true) {
        return distanceRight.get() / MM_TO_INCHES * rightDistanceCoefficient;
    }
    return -1;
}

double backDistanceCoefficient = 1.0;
int getBackDistance() {
    if (distanceBack.get_object_size() < 200) { // 2 inches in mm is approximately 50mm
        backWall = true;
    } else {
        backWall = false;
        return -1;
    }
    if (backWall == true) {
        return distanceBack.get() / MM_TO_INCHES * backDistanceCoefficient;
    }
    return -1;
}

int distanceTest() {
    while (true) {
        pros::lcd::clear();
        pros::lcd::set_text(1, "Distance Front " + std::to_string(getFrontDistance()) 
        + " inches" + " Wall: " + std::to_string(frontWall));
        pros::lcd::set_text(2, "Distance Right " + std::to_string(getRightDistance())
         + " inches" + " Wall: " + std::to_string(rightWall));
        pros::lcd::set_text(3, "Distance Back " + std::to_string(getBackDistance())
         + " inches" + " Wall: " + std::to_string(backWall));
        pros::lcd::set_text(4, "Distance Left " +std::to_string(getLeftDistance())
         + " inches" + " Wall: " + std::to_string(leftWall));
        pros::delay(100);
    }
}

// Field Starts at 0,0 to 144, 144

void distanceReset(char sensorPosition, char wallPosition) {//A top, B right, C bottom, D left
    double frontDistance = getFrontDistance();
    double rightDistance = getRightDistance();
    double backDistance = getBackDistance();
    double leftDistance = getLeftDistance();

    double heading = robotPosition.at(2,0) * radianConversion;

    switch(sensorPosition) {
        case 'A': // Front Sensor
            if (frontDistance != -1) {
                double sensorX = DIST_FRONT_X * cos(heading) - DIST_FRONT_Y * sin(heading);
                double sensorY = DIST_FRONT_X * sin(heading) + DIST_FRONT_Y * cos(heading);
                double totalDist = frontDistance;
            
                switch(wallPosition) {
                    case 'A': // Front Wall
                        robotPosition.at(1,0) = 144 - totalDist - sensorY;
                        break;
                    case 'B': // Right Wall
                        robotPosition.at(0,0) = 144 - totalDist - sensorX;
                        break;
                    case 'C': // Back Wall
                        robotPosition.at(1,0) = totalDist + sensorY;
                        break;
                    case 'D': // Left Wall
                        robotPosition.at(0,0) = totalDist + sensorX;
                        break;
                }
            }
            break;
        case 'B': // Right Sensor
            if (rightDistance != -1) {
                double sensorX = DIST_RIGHT_X * cos(heading) - DIST_RIGHT_Y * sin(heading);
                double sensorY = DIST_RIGHT_X * sin(heading) + DIST_RIGHT_Y * cos(heading);
                double totalDist = rightDistance;
            
                switch(wallPosition) {
                    case 'A': // Front Wall
                        robotPosition.at(1,0) = 144 - totalDist - sensorY;
                        break;
                    case 'B': // Right Wall
                        robotPosition.at(0,0) = 144 - totalDist - sensorX;
                        break;
                    case 'C': // Back Wall
                        robotPosition.at(1,0) = totalDist + sensorY;
                        break;
                    case 'D': // Left Wall
                        robotPosition.at(0,0) = totalDist + sensorX;
                        break;
                }
            }
            break;
        case 'C': // Back Sensor
            if (backDistance != -1) {
                double sensorX = DIST_BACK_X * cos(heading) - DIST_BACK_Y * sin(heading);
                double sensorY = DIST_BACK_X * sin(heading) + DIST_BACK_Y * cos(heading);
                double totalDist = backDistance;
            
                switch(wallPosition) {
                    case 'A': // Front Wall
                        robotPosition.at(1,0) = 144 - totalDist - sensorY;
                        break;
                    case 'B': // Right Wall
                        robotPosition.at(0,0) = 144 - totalDist - sensorX;
                        break;
                    case 'C': // Back Wall
                        robotPosition.at(1,0) = totalDist + sensorY;
                        break;
                    case 'D': // Left Wall
                        robotPosition.at(0,0) = totalDist + sensorX;
                        break;
                }
            }
            break;
        case 'D': // Left Sensor
            if (leftDistance != -1) {
                double sensorX = DIST_LEFT_X * cos(heading) - DIST_LEFT_Y * sin(heading);
                double sensorY = DIST_LEFT_X * sin(heading) + DIST_LEFT_Y * cos(heading);
                double totalDist = leftDistance;
            
                switch(wallPosition) {
                    case 'A': // Front Wall
                        robotPosition.at(1,0) = 144 - totalDist - sensorY;
                        break;
                    case 'B': // Right Wall
                        robotPosition.at(0,0) = 144 - totalDist - sensorX;
                        break;
                    case 'C': // Back Wall
                        robotPosition.at(1,0) = totalDist + sensorY;
                        break;
                    case 'D': // Left Wall
                        robotPosition.at(0,0) = totalDist + sensorX;
                        break;
                }
            }
            break;
    }
}



//Double Park Function (IN PROGRESS)

void doublePark(void* param) {
    while(distanceBack.get_distance() < 127) { // 5 inches in mm is approximately 127mm
        pros::delay(20);
    }
}

//parking zone clear function
//checks inertial sensor for orientation of robot if it is tipped backward then tipped forward then it is over the parking zone
//if it does it again it is then outside of the parking zone
//essentially reverses the state of isParkingZone every time it detects the tipping motion
//it is activated as a function that makes the robot drive forward and slightly left until it detects the tipping motion again
bool isParkingZone = false;

void parkingZone(void* param) {
    odomLift.set_value(true); //Raise odometry lift
    LeftDriveSmart.move_velocity(95);
    RightDriveSmart.move_velocity(100);  //Drives slightly left to make sure robot is gliding against wall
    if (isParkingZone) {
        Scraper();
    } else if (!isParkingZone) {
        Scraper();
    }
    while (true) {
        float pitch = inertialSensor.get_pitch();
        float roll = inertialSensor.get_roll();
        static bool wasTippedBack = false;
        static bool wasTippedForward = false;
        if (pitch > 20) { // Tipped backward
            wasTippedBack = true;
        } else if (pitch < -20) { // Tipped forward
            wasTippedForward = true;
        } else {
            if (wasTippedBack && wasTippedForward) {
                isParkingZone = !isParkingZone; // Toggle parking zone state
            }
            wasTippedBack = false;
            wasTippedForward = false;
        }
        pros::delay(100); // Check every 100 ms
    }
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
void ramseteControl(void* param) {
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
        matrix targetPosition(3,1);
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
        pros::delay(20); //waits 20 milliseconds before next loop
    }
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

void getMotion(void* param) {
    while(true) {
        bufferIndex = motionCounter % BUFFER;
        int prevBufferIdx = (bufferIndex - 1 + BUFFER) % BUFFER;
        double dt = 0.02; //Change in time 20 milliseconds

        //velocity
        double leftVelocity = (LeftDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        double rightVelocity = (RightDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        
        leftRobotVelocity[bufferIndex] = leftVelocity;
        rightRobotVelocity[bufferIndex] = rightVelocity;

        //acceleration delta v over delta t
        if(motionCounter > 0) {
            double leftAcceleration = (leftRobotVelocity[bufferIndex] - leftRobotVelocity[prevBufferIdx]) / dt;
            double rightAcceleration = (rightRobotVelocity[bufferIndex] - rightRobotVelocity[prevBufferIdx]) / dt;

            leftRobotAcceleration[bufferIndex] = leftAcceleration;
            rightRobotAcceleration[bufferIndex] = rightAcceleration;
        } else { //First iteration
            leftRobotAcceleration[bufferIndex] = 0;
            rightRobotAcceleration[bufferIndex] = 0;      
        }
        motionCounter++;
        pros::delay(20);
    }
}

// Get most recent velocity
double getCurrentLeftVelocity() {
    if (motionCounter == 0) return 0.0;
    int idx = (motionCounter - 1) % BUFFER;
    if (idx < 0) idx += BUFFER;
    return leftRobotVelocity[idx];
}

// Get most recent acceleration
double getCurrentLeftAcceleration() {
    if (motionCounter == 0) return 0.0;
    int idx = (motionCounter - 1) % BUFFER;
    if (idx < 0) idx += BUFFER;
    return leftRobotAcceleration[idx];
}

// Get most recent velocity
double getCurrentRightVelocity() {
    if (motionCounter == 0) return 0.0;
    int idx = (motionCounter - 1) % BUFFER;
    if (idx < 0) idx += BUFFER;
    return rightRobotVelocity[idx];
}

// Get most recent acceleration
double getCurrentRightAcceleration() {
    if (motionCounter == 0) return 0.0;
    int idx = (motionCounter - 1) % BUFFER;
    if (idx < 0) idx += BUFFER;
    return rightRobotAcceleration[idx];
}

//inches/second
void motionProfile(void* param) {
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
            pros::delay(20);
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
        LeftDriveSmart.move_voltage(commandedLeftVelocity * 120);
        RightDriveSmart.move_voltage(commandedRightVelocity * 120);
        
        pros::delay(20);
    }
}

//testing max acceleration and jerk then printing it on brain screen
// Add these functions to your code

// Test function to find maximum acceleration
void testMaxAcceleration() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "Testing Max Acceleration...");
    
    // Reset position and stop motors
    LeftDriveSmart.brake();
    RightDriveSmart.brake();
    pros::delay(500);
    
    // Arrays to store velocity samples
    const int samples = 100;
    double velocities[samples];
    double timeStamps[samples];
    
    // Record start time
    double startTime = pros::millis();
    
    // Apply full power and collect velocity data
    LeftDriveSmart.move_voltage(12000);
    RightDriveSmart.move_voltage(12000);
    
    for(int i = 0; i < samples; i++) {
        timeStamps[i] = (pros::millis() - startTime) / 1000.0; // Convert to seconds
        double leftVel = (LeftDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        double rightVel = (RightDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        velocities[i] = (leftVel + rightVel) / 2.0; // Average velocity in inches/sec
        pros::delay(20);
    }
    
    // Stop motors
    LeftDriveSmart.brake();
    RightDriveSmart.brake();
    
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
    pros::lcd::clear();
    pros::lcd::set_text(1, "Max Acceleration: " + std::to_string(maxAccel) + " in/s^2");
    pros::delay(3000);
}

// Test function to find maximum jerk
void testMaxJerk() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "Testing Max Jerk...");
    
    // Reset position and stop motors
    LeftDriveSmart.brake();
    RightDriveSmart.brake();
    pros::delay(500);
    
    // Arrays to store acceleration samples
    const int samples = 100;
    double accelerations[samples];
    double timeStamps[samples];
    double velocities[samples];
    
    // Record start time
    double startTime = pros::millis();
    
    // Apply full power and collect velocity data
    LeftDriveSmart.move_voltage(12000);
    RightDriveSmart.move_voltage(12000);
    
    // First pass: collect velocities
    for(int i = 0; i < samples; i++) {
        timeStamps[i] = (pros::millis() - startTime) / 1000.0; // Convert to seconds
        double leftVel = (LeftDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        double rightVel = (RightDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        velocities[i] = (leftVel + rightVel) / 2.0; // Average velocity in inches/sec
        pros::delay(20);
    }
    
    // Stop motors
    LeftDriveSmart.brake();
    RightDriveSmart.brake();
    
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
    double maxJerkValue = 0;
    for(int i = 2; i < samples; i++) {
        double dt = timeStamps[i] - timeStamps[i-1];
        if(dt > 0) {
            double jerk = (accelerations[i] - accelerations[i-1]) / dt;
            if(fabs(jerk) > maxJerkValue) {
                maxJerkValue = fabs(jerk);
            }
        }
    }
    
    // Display result
    pros::lcd::clear();
    pros::lcd::set_text(1, "Max Jerk: " + std::to_string(maxJerkValue) + " in/s^3");
    pros::delay(3000);
}

// Combined test function that runs both tests
void testMotionLimits() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "Starting Motion Tests...");
    pros::delay(1000);
    
    // Test acceleration
    testMaxAcceleration();
    
    // Wait between tests
    pros::delay(1000);
    
    // Test jerk
    testMaxJerk();
    
    pros::lcd::clear();
    pros::lcd::set_text(1, "Motion tests complete!");
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

        pros::delay(20);
    }
    
    // Stops at target
    desiredLinearVelocity = 0;
    desiredTurningVelocity = 0;
    pros::delay(100); //Let robot come still
}
//Path Making

//Path

void setStartPosition(double x, double y, double heading) {
    robotPosition.at(0,0) = x;
    robotPosition.at(1,0) = y;
    robotPosition.at(2,0) = heading;
    goTo(robotPosition.at(0,0), robotPosition.at(1,0), robotPosition.at(2,0), 0, 0, true);
}

//Red
void redDriveForwardPath() {
    // Simple drive forward for PROS
    LeftDriveSmart.move_relative(400, 100); // ~4 inches
    RightDriveSmart.move_relative(400, 100);
}

void redEast1GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}

void redEast2GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);  
} 

void redWest1GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);  
} 

void redWest2GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
} 

//Blue
void blueDriveForwardPath() {
    // Simple drive forward for PROS
    LeftDriveSmart.move_relative(400, 100); // ~4 inches
    RightDriveSmart.move_relative(400, 100);
}

void blueEast1GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}

void blueEast2GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}

void blueWest1GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}

void blueWest2GoalPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}

void autoSkillsPath() {
    setStartPosition(20, 90, 90);
    //Start Path (x, y, heading, linear velocity, angular velocity, isForward)
    goTo(1,1,1,1,1, true);
}




//PID Controller for tuning velocities given by the Ramsete controller

//Desired right and left velocities given by controller
/*
//PID settings start
const float velocity_kP = 0.0;
const float velocity_kI = 0.0;
const float velocity_kD = 0.0;

//Error variables for velocity PID
//left side
float leftVelocity;
float leftVelocityError;
float leftPrevVelocityError = 0.0;
float leftDerivativeVelocityError;
float leftIntegralVelocityError = 0.0;
//right side
float rightVelocity;
float rightVelocityError;
float rightPrevVelocityError = 0.0;
float rightDerivativeVelocityError;
float rightIntegralVelocityError = 0.0;

//Velocity PID function
void velocityPID(void* param) {
    while(true) {
        //Velocity calculations
        leftVelocity = (LeftDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        rightVelocity = (RightDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
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
        LeftDriveSmart.move_voltage(leftMotorPower * 120);
        RightDriveSmart.move_voltage(rightMotorPower * 120);
        pros::delay(20); //waits 20 milliseconds before next loop
    }
}

//Start Driving Forward PID Controller
//For going straight
const float kP = 6;  //7
const float kI = 0;
const float kD = 0.01; //2 0.03

//For turning
const float turning_kP = 0; //less than .1 
const float turning_kI = 0; //should be a really small number 0.024 example
const float turning_kD = 0; //less than .05 usually

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
float desiredTurnValue = 0;
float targetDistance = 0;

double totalDistanceTravelled = 0;

short timerCount = 0;

//PID function for driving
void drivePID(void* param) {
    while(enableDrivePID == true) {
        if (resetPID_Sensors == true) {
            //Resets the sensors and variables
            rotationalLateral.reset_position();
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
        double robotHeading = inertialSensor.get_heading(); //in degrees
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
        LeftDriveSmart.move_voltage(motorsLeftPower * 120);
        RightDriveSmart.move_voltage(motorsRightPower * 120);
        //Sets previous robot position vector to current robot position vector. Cos and sin in radians because that is what they take as arguments.
        //Getting the sin and cos is like polar coordinates where distance travelled is the radius and robot heading is the angle. 
        //(x,y) == (Rcos(theta),Rsin(theta))
        //Arc length formula (theta in deg)t: theta/180 * pi (radian conversion) * radius
        //Delta distance travelled since last reset
        totalDistanceTravelled = (rotationalLateral.get_position() / 100.0) * radianConversion;
        targetDistance = targetDistance - (totalDistanceTravelled - previousDistanceTravelled);
        addDataPoint(headingError);  // Or headingError, or motorPower, etc.
        if (fabs(targetDistance) < 0.1 && fabs(headingError) < 3 && timerCount < 20) { //If within 1 inches and 3 degree of target, stop motors and exit task
            timerCount += 1;
        } else if (fabs(targetDistance) > 0.1 || fabs(headingError) < 3) {
            timerCount = 0;
        } else if (timerCount >= 20) {
            timerCount = 0;
            LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
            LeftDriveSmart.brake();
            RightDriveSmart.brake();
            resetPID_Sensors = true; //resets sensors when target reached
            enableDrivePID = false;  //disables PID when target reached
            return; //exits the function and TASK
        }
            
    
        //Sets prevError to current error
        previousDistanceTravelled = totalDistanceTravelled;
        prevDistanceError = distanceError;
        prevHeadingError = headingError;
        pros::delay(20); //waits 20 milliseconds before next loop
    }
}*/


//////////////////////////////////////////////
// Preauton Selector
//////////////////////////////////////////////
// Begin project code
bool isAutonomous = true;
bool isSelectingPreAuton = true;

char preAutonSelection;
std::string autonDescription[6] = {"", "Drive Forward", "East 1 Goal", "East 2 Goal", "West 1 Goal", "West 2 Goal"};
std::string autonDescriptionSkills[3] = {"", "Auton", "Driver"};
std::string teamDescription[3] = {"Red", "Blue", "Skills"};

int teamIndex = 0;
int indexOption = 0;
/* 0 = DriveForward, 1 = East 1 Goal, 2 = East 2 Goal, 3 = West 1 Goal, 4 = West 2 Goal*/
char redTypeOptions[5] = {'A', 'B', 'C', 'D', 'E'};
char blueTypeOptions[5] = {'F', 'G', 'H', 'I', 'J'}; // 0 = left, 1 = right, 2 = forward
char skillsTypeOptions[2] = {'K', 'L'}; // 0 = Auton, 1 = Driver

char matchType[3] = {redTypeOptions[indexOption], blueTypeOptions[indexOption], skillsTypeOptions[indexOption]}; // red, blue, skills

void changeGameType() {
    teamIndex = (teamIndex + 1) % 3; // Cycle through matchType indices 0-2
    indexOption = 0; // Reset indexOption when changing game type
    matchType[0] = redTypeOptions[indexOption];
    matchType[1] = blueTypeOptions[indexOption];
    matchType[2] = skillsTypeOptions[indexOption];
}

void changeGameTypeOption() {
    if (teamIndex == 0) {
        indexOption = (indexOption + 1) % 5; // Cycle through redTypeOptions indices 0-4
        matchType[0] = redTypeOptions[indexOption];
    } else if (teamIndex == 1) {
        indexOption = (indexOption + 1) % 5; // Cycle through blueTypeOptions indices 0-4
        matchType[1] = blueTypeOptions[indexOption];
    } else if (teamIndex == 2) {
        indexOption = (indexOption + 1) % 2; // Cycle through skillsTypeOptions indices 0-1
        matchType[2] = skillsTypeOptions[indexOption];
    }
}

void confirmPreAutonSelection(){
    if (teamIndex == 0) {
        preAutonSelection = redTypeOptions[indexOption];
    } else if (teamIndex == 1) {
        preAutonSelection = blueTypeOptions[indexOption];
    } else if (teamIndex == 2) {
        preAutonSelection = skillsTypeOptions[indexOption];
    }
    isSelectingPreAuton = false;
}

int preAutonSelector() {
    while(isSelectingPreAuton) {
        std::string displayText = "PreAuton Selector: " + teamDescription[teamIndex] + " ";
        
        if (teamIndex == 0 || teamIndex == 1) {
            displayText += autonDescription[indexOption + 1];
        } else {
            displayText += autonDescriptionSkills[indexOption + 1];
        }
        
        pros::lcd::print(7, displayText.c_str());
        pros::lcd::register_btn0_cb(changeGameTypeOption);
        pros::lcd::register_btn1_cb(changeGameType);
        pros::lcd::register_btn2_cb(confirmPreAutonSelection);
        pros::delay(100);
    }
    return 0;
}

//////////////////////////////////////////////
// PROS Competition Functions
//////////////////////////////////////////////

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    // actions to do when the program starts
    //Initialize LVGL brain screen

    //Set initial brain state
    pros::lcd::initialize();
    pros::lcd::set_text(1, "pre auton code");

    
    rotationalLateral.reset_position(); //resetting the rotational sensor position to 0
    rotationalHorizontal.reset_position(); //resetting the rotational sensor position to 0
    
    rotationalHorizontal.set_reversed(true);
    //calibrating the inertial sensor MUST DO THIS
    inertialSensor.reset();
    while (inertialSensor.is_calibrating()) {
        pros::delay(100);
    }
    
    // Track touch task for auton selector
    // pros::Task trackTouch_Thread(trackTouch);
    //Calls auton selector
    // firstPage();
    
    pros::lcd::set_text(2, "Calibration Complete");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
    //Robot Position Vector Initialization
    robotPosition.at(0,0) = 0; //x position in inches
    robotPosition.at(1,0) = 0; //y position in inches
    robotPosition.at(2,0) = 0; //heading in degrees
    
    rotationalHorizontal.reset_position(); //resetting the rotational sensor position to 0
    rotationalLateral.reset_position(); //resetting the rotational sensor position to 0
    
    pros::lcd::set_text(1, "autonomous code");
    
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    hoodMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    if (inertialSensor.is_calibrating() == false) {
        inertialSensor.tare_rotation();
        inertialSensor.set_heading(90); //sets the heading to 90 degrees to match field orientation
        robotPosition.at(2,0) = 90;
    }
    
    scraperState = false;
    hood.set_value(false);
    inertialSensor.set_heading(90); //sets the heading to 90 degrees to match field orientation


    // Start tasks
    // pros::Task odometryTest_Thread(odometryTest);
    // pros::Task odometry_Thread(odometry);
    // pros::Task getMotion_Thread(getMotion); // Start motion tracking
    // pros::Task motionProfile_Thread(motionProfile); // Start motion profile
    // pros::Task graph_Thread(graphTask);
    
    // place automonous code here
    while(isSelectingPreAuton == true) {
        switch (preAutonSelection) {
            case 'A': // Blue Drive Forward
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Blue Drive Forward");
                isRedTeam = false;
                colorSort(isRedTeam);
                blueDriveForwardPath();
                isAutonomous = false;
                break;
            case 'B': // Blue East 1 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Blue East 1 Goal");
                isRedTeam = false;
                colorSort(isRedTeam);
                blueEast1GoalPath();
                isAutonomous = false;
                break;
            case 'C': // Blue East 2 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Blue East 2 Goal");
                isRedTeam = false;
                colorSort(isRedTeam);
                blueEast2GoalPath();
                isAutonomous = false;
                break;
            case 'D': // Blue West 1 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Blue West 1 Goal");
                isRedTeam = false;
                colorSort(isRedTeam);
                blueWest1GoalPath();
                isAutonomous = false;
                break;
            case 'E': // Blue West 2 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Blue West 2 Goal");
                isRedTeam = false;
                colorSort(isRedTeam);
                blueWest2GoalPath();
                isAutonomous = false;
                break;
            case 'F': // Red Drive Forward
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Red Drive Forward");
                isRedTeam = true;
                colorSort(isRedTeam);
                redDriveForwardPath();
                isAutonomous = false;
                break;
            case 'G': // Red East 1 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Red East 1 Goal");
                isRedTeam = true;
                colorSort(isRedTeam);
                redEast1GoalPath();
                isAutonomous = false;
                break;
            case 'H': // Red East 2 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Red East 2 Goal");
                isRedTeam = true;
                colorSort(isRedTeam);
                redEast2GoalPath();
                isAutonomous = false;
                break;
            case 'I': // Red West 1 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Red West 1 Goal");
                isRedTeam = true;
                colorSort(isRedTeam);
                redWest1GoalPath();
                isAutonomous = false;
                break;
            case 'J': // Red West 2 Goal
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Red West 2 Goal");
                isRedTeam = true;
                colorSort(isRedTeam);
                redWest2GoalPath();
                isAutonomous = false;
                break;
            case 'K': // Auton Skills
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Auton Skills");
                autoSkillsPath();
                isAutonomous = false;
                break;
            case 'L': // Driver Skills
                isSelectingPreAuton = false;
                pros::lcd::set_text(1, "Driver Skills");
                isAutonomous = false;
                break;
            default: // The default case handles invalid operators
                pros::lcd::set_text(1, "Select Auton");
        }
        pros::delay(100);
    }
}

/**
 * Runs the operator control code.
 */
void opcontrol() {
    robotPosition.at(0,0) = 0; //x position in inches
    robotPosition.at(1,0) = 0; //y position in inches
    robotPosition.at(2,0) = 90; //heading in degrees
    pros::Task odometry_Thread(odometry, nullptr, TASK_PRIORITY_DEFAULT, 16384, "Odometry");    //pros::Task odometryTest_Thread(odometryTest);
    // enableDrivePID = false; //disables PID control during user control
    pros::lcd::set_text(1, "driver control");
        
    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    hoodMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
        
    // place driver control in this while loop

    while(true){
        // Display position info periodically
        static int displayCounter = 0;
        if (displayCounter % 400 == 0) { // Every ~4 seconds (400 * 10ms)
          pros::lcd::clear();
          pros::lcd::set_text(1, "x: " + std::to_string(robotPosition.at(0,0)));
          pros::lcd::set_text(2, "y: " + std::to_string(robotPosition.at(1,0)));
          pros::lcd::set_text(3, "Heading: " + std::to_string(robotPosition.at(2,0)));
        }
        displayCounter++;
        
        // Arcade drive with quadratic curve
        double joyLeft = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        double joyRight = Controller1.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        // Deadband
        if (abs(joyLeft) < 5) joyLeft = 0;
        if (abs(joyRight) < 5) joyRight = 0;
        
        // Quadratic curve for smoother control
        double left = joyLeft + joyRight;
        double right = joyLeft - joyRight;
        int leftSideSpeed = (left * abs(left)) / 127;
        int rightSideSpeed = (right * abs(right)) / 127;
        
        LeftDriveSmart.move(leftSideSpeed);
        RightDriveSmart.move(rightSideSpeed);
            
        input();

        //testMotionLimits();
            
        pros::delay(10);
    }
}