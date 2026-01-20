#include "main.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>

// Define Settings
#define PI 3.1415926535897
#define WHEEL_CIRCUMFERENCE (3.25 * PI)  // inches
#define DISTANCE_FROM_CENTER_TO_LATERAL_WHEEL (7.0 / 16.0)  // inches
#define DISTANCE_FROM_CENTER_TO_HORIZONTAL_WHEEL 3.0  // inches
#define TRACK_WIDTH 10.75  // inches
#define BUFFER 50  // Last 50 values

// Motor and Sensor Declarations
pros::MotorGroup LeftDriveSmart({-12, -14, -13});
pros::MotorGroup RightDriveSmart({15, 19, 18});
pros::MotorGroup intakeMotor({2, -17});
pros::Motor hoodMotor(11);

// Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut descore('H');
pros::adi::DigitalOut hood('A');
pros::adi::DigitalOut scraper('F');

// Rotational sensors
pros::Rotation rotationalLateral(20);
pros::Rotation rotationalHorizontal(4);

// Inertial sensor
pros::IMU inertialSensor(5);

// Distance sensors
pros::Distance distanceFront(15);
pros::Distance distanceLeft(16);
pros::Distance distanceRight(17);
pros::Distance distanceBack(20);

// Setting variables
float maxMotorPercentage = 100.0;
double radianConversion = PI / 180.0;
double gearRatio = 3.0 / 5.0;
double maxVelocity = 61.0;  // inches per second
double maxAcceleration = 15.0;
double maxJerk = 15.0;

// Matrix Class
class matrix {
private:
    double** data;
    int rows;
    int cols;

public:
    matrix(int r, int c) : rows(r), cols(c) {
        data = new double*[rows];
        for(int i = 0; i < rows; i++) {
            data[i] = new double[cols];
            memset(data[i], 0, cols * sizeof(double));
        }
    }

    ~matrix() {
        for(int i = 0; i < rows; i++) {
            delete[] data[i];
        }
        delete[] data;
    }

    double& at(int r, int c) {
        return data[r][c];
    }

    int getRows() { return rows; }
    int getCols() { return cols; }

    matrix multiply(const matrix& other) {
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
        matrix result(rows, cols);
        for(int i = 0; i < rows; i++) {
            for(int j = 0; j < cols; j++) {
                result.data[i][j] = data[i][j] - other.data[i][j];
            }
        }
        return result;
    }
};

// Robot Position
matrix robotPosition(3, 1);
double previousAngleRotation = 0;
double previousHeading = 0;
double previousLateralTravelled = 0;
double previousHorizontalTravelled = 0;

// Odometry Task
void odometry_task(void* param) {
    while(true) {
        double deltaLateral = (rotationalLateral.get_position() / 100.0 - previousLateralTravelled) * radianConversion;
        double deltaHorizontal = (rotationalHorizontal.get_position() / 100.0 - previousHorizontalTravelled) * radianConversion;
        double deltaOrientation = inertialSensor.get_rotation() - previousAngleRotation;
        double arcAngle = fabs(deltaOrientation);
        double arcAngleRadians = arcAngle * radianConversion;

        const double movementThreshold = 0.01;
        if (fabs(deltaLateral) < movementThreshold && 
            fabs(deltaHorizontal) < movementThreshold && 
            fabs(deltaOrientation) < movementThreshold) {
            pros::delay(10);
            continue;
        }

        double lateralArcRadius;
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

        double gridOffset = previousAngleRotation + (deltaOrientation / 2);
        double gridOffsetRadians = gridOffset * radianConversion;
        robotPosition.at(0, 0) += localDeltaX * cos(gridOffsetRadians) - localDeltaY * sin(gridOffsetRadians);
        robotPosition.at(1, 0) += localDeltaX * sin(gridOffsetRadians) + localDeltaY * cos(gridOffsetRadians);
        robotPosition.at(2, 0) = inertialSensor.get_heading();

        previousAngleRotation = inertialSensor.get_rotation();
        previousHeading = inertialSensor.get_heading();
        previousLateralTravelled = rotationalLateral.get_position() / 100.0;
        previousHorizontalTravelled = rotationalHorizontal.get_position() / 100.0;

        pros::delay(10);
    }
}

// Ramsete Controller Variables
const double zeta = 0.7;
const double b = 2.0;
matrix localTransformationMatrix(3, 3);
double leftMotorVelocity = 0;
double rightMotorVelocity = 0;
double desiredX = 0;
double desiredY = 0;
double desiredTheta = 0;
double desiredLinearVelocity = 0;
double desiredTurningVelocity = 0;
bool direction = false;

// Ramsete Control Task
void ramsete_control_task(void* param) {
    while(true) {
        localTransformationMatrix.at(0, 0) = cos(robotPosition.at(2, 0) * radianConversion);
        localTransformationMatrix.at(0, 1) = sin(robotPosition.at(2, 0) * radianConversion);
        localTransformationMatrix.at(0, 2) = 0;
        localTransformationMatrix.at(1, 0) = -sin(robotPosition.at(2, 0) * radianConversion);
        localTransformationMatrix.at(1, 1) = cos(robotPosition.at(2, 0) * radianConversion);
        localTransformationMatrix.at(1, 2) = 0;
        localTransformationMatrix.at(2, 0) = 0;
        localTransformationMatrix.at(2, 1) = 0;
        localTransformationMatrix.at(2, 2) = 1;

        matrix targetPosition(3, 1);
        targetPosition.at(0, 0) = desiredX;
        targetPosition.at(1, 0) = desiredY;
        targetPosition.at(2, 0) = desiredTheta;
        matrix globalError = targetPosition.subtract(robotPosition);
        matrix localError = localTransformationMatrix.multiply(globalError);

        double k = 2 * zeta * sqrt(desiredTurningVelocity * desiredTurningVelocity + 
                                    b * desiredLinearVelocity * desiredLinearVelocity);
        double velocity = desiredLinearVelocity * cos(localError.at(2, 0) * radianConversion) + 
                         k * localError.at(0, 0);
        double angleErrorTerm = localError.at(2, 0) * radianConversion;

        double sinc_term;
        if (fabs(angleErrorTerm) < 0.001) {
            sinc_term = 1.0;
        } else {
            sinc_term = sin(angleErrorTerm) / angleErrorTerm;
        }

        double angularVelocity = desiredTurningVelocity + 
                                b * desiredLinearVelocity * sinc_term * localError.at(1, 0) + 
                                k * localError.at(2, 0) * radianConversion;

        double linearMotorVelocity = velocity / WHEEL_CIRCUMFERENCE;
        double wheelAngularContribution = (angularVelocity * TRACK_WIDTH) / (2 * WHEEL_CIRCUMFERENCE);
        leftMotorVelocity = linearMotorVelocity + wheelAngularContribution;
        rightMotorVelocity = linearMotorVelocity - wheelAngularContribution;

        if (direction == false) {
            leftMotorVelocity = -leftMotorVelocity;
            rightMotorVelocity = -rightMotorVelocity;
        }

        pros::delay(20);
    }
}

// Motion tracking variables
int motionCounter = 0;
std::vector<double> leftRobotVelocity(BUFFER, 0.0);
std::vector<double> rightRobotVelocity(BUFFER, 0.0);
std::vector<double> leftRobotAcceleration(BUFFER, 0.0);
std::vector<double> rightRobotAcceleration(BUFFER, 0.0);

void get_motion_task(void* param) {
    while(true) {
        int bufferIndex = motionCounter % BUFFER;
        int prevBufferIndex = (bufferIndex - 1 + BUFFER) % BUFFER;
        double dt = 0.02;

        double leftVelocity = (LeftDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;
        double rightVelocity = (RightDriveSmart.get_actual_velocity() / 60.0) * WHEEL_CIRCUMFERENCE * gearRatio;

        leftRobotVelocity[bufferIndex] = leftVelocity;
        rightRobotVelocity[bufferIndex] = rightVelocity;

        if(motionCounter > 0) {
            leftRobotAcceleration[bufferIndex] = (leftRobotVelocity[bufferIndex] - leftRobotVelocity[prevBufferIndex]) / dt;
            rightRobotAcceleration[bufferIndex] = (rightRobotVelocity[bufferIndex] - rightRobotVelocity[prevBufferIndex]) / dt;
        } else {
            leftRobotAcceleration[bufferIndex] = 0;
            rightRobotAcceleration[bufferIndex] = 0;
        }

        motionCounter++;
        pros::delay(20);
    }
}

double getCurrentLeftVelocity() {
    int index = (motionCounter - 1) % BUFFER;
    if (index < 0) index += BUFFER;
    return leftRobotVelocity[index];
}

double getCurrentRightVelocity() {
    int index = (motionCounter - 1) % BUFFER;
    if (index < 0) index += BUFFER;
    return rightRobotVelocity[index];
}

// Motion Profile Task
void motion_profile_task(void* param) {
    double targetLeftAccel = 0;
    double targetRightAccel = 0;

    while(true) {
        double currentLeftVelocity = getCurrentLeftVelocity();
        double currentRightVelocity = getCurrentRightVelocity();
        double leftVelocityError = leftMotorVelocity - currentLeftVelocity;
        double rightVelocityError = rightMotorVelocity - currentRightVelocity;

        if (fabs(leftVelocityError) <= 0.1 && fabs(rightVelocityError) <= 0.1) {
            targetLeftAccel = 0;
            targetRightAccel = 0;
            pros::delay(20);
            continue;
        }

        double dt = 0.02;
        double desiredLeftAccel = leftVelocityError / dt;
        double desiredRightAccel = rightVelocityError / dt;
        double leftAccelChange = desiredLeftAccel - targetLeftAccel;
        double rightAccelChange = desiredRightAccel - targetRightAccel;
        double maxJerkStep = maxJerk * dt;

        leftAccelChange = std::max(-maxJerkStep, std::min(leftAccelChange, maxJerkStep));
        rightAccelChange = std::max(-maxJerkStep, std::min(rightAccelChange, maxJerkStep));

        targetLeftAccel += leftAccelChange;
        targetRightAccel += rightAccelChange;

        targetLeftAccel = std::max(-maxAcceleration, std::min(targetLeftAccel, maxAcceleration));
        targetRightAccel = std::max(-maxAcceleration, std::min(targetRightAccel, maxAcceleration));

        double commandedLeftVelocity = currentLeftVelocity + targetLeftAccel * dt;
        double commandedRightVelocity = currentRightVelocity + targetRightAccel * dt;

        if (leftVelocityError > 0 && commandedLeftVelocity > leftMotorVelocity) {
            commandedLeftVelocity = leftMotorVelocity;
        } else if (leftVelocityError < 0 && commandedLeftVelocity < leftMotorVelocity) {
            commandedLeftVelocity = leftMotorVelocity;
        }

        if (rightVelocityError > 0 && commandedRightVelocity > rightMotorVelocity) {
            commandedRightVelocity = rightMotorVelocity;
        } else if (rightVelocityError < 0 && commandedRightVelocity < rightMotorVelocity) {
            commandedRightVelocity = rightMotorVelocity;
        }

        commandedLeftVelocity = (((commandedLeftVelocity / WHEEL_CIRCUMFERENCE) * 60.0 / gearRatio) / 600.0) * 100.0;
        commandedRightVelocity = (((commandedRightVelocity / WHEEL_CIRCUMFERENCE) * 60.0 / gearRatio) / 600.0) * 100.0;

        LeftDriveSmart.move_voltage(commandedLeftVelocity * 120);
        RightDriveSmart.move_voltage(commandedRightVelocity * 120);

        pros::delay(20);
    }
}

// Go To Function
void goTo(double x, double y, double heading, double linearVelocity, double turningVelocity, bool forward) {
    desiredX = x;
    desiredY = y;
    desiredTheta = heading;
    desiredLinearVelocity = linearVelocity;
    desiredTurningVelocity = turningVelocity;
    direction = forward;

    while(true) {
        double xError = desiredX - robotPosition.at(0, 0);
        double yError = desiredY - robotPosition.at(1, 0);
        double positionError = sqrt(xError * xError + yError * yError);

        double headingError = desiredTheta - robotPosition.at(2, 0);
        headingError = atan2(sin(headingError * radianConversion), cos(headingError * radianConversion)) / radianConversion;
        headingError = fabs(headingError);

        if (positionError < 0.5 && headingError < 2.0) {
            break;
        }

        pros::delay(20);
    }

    desiredLinearVelocity = 0;
    desiredTurningVelocity = 0;
    pros::delay(100);
}

// Pneumatic control variables
bool descoreState = false;
bool scraperState = false;
bool hoodState = false;
bool slowMode = false;

void toggleScraper() {
    scraperState = !scraperState;
    scraper.set_value(scraperState);
}

void toggleDescore() {
    descoreState = !descoreState;
    descore.set_value(descoreState);
}

void toggleHood() {
    hoodState = !hoodState;
    hood.set_value(hoodState);
}

// Input handling
void input() {
    static bool L1WasPressed = false;
    static bool XWasPressed = false;
    static bool DownWasPressed = false;

    // Intake control
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intakeMotor.move_velocity(100);
        hoodMotor.move_velocity(100);
    } 
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        int velocity = slowMode ? 25 : 100;
        intakeMotor.move_velocity(-velocity);
        hoodMotor.move_velocity(-velocity);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        intakeMotor.move_velocity(100);
        hoodMotor.move_velocity(100);
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        int velocity = slowMode ? 25 : 100;
        intakeMotor.move_velocity(velocity);
        hoodMotor.move_velocity(-velocity);
    }
    else {
        intakeMotor.brake();
        hoodMotor.brake();
    }

    // Slow mode
    slowMode = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    // Scraper toggle
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !L1WasPressed) {
        toggleScraper();
        L1WasPressed = true;
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        L1WasPressed = false;
    }

    // Descore toggle
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !XWasPressed) {
        toggleDescore();
        XWasPressed = true;
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        XWasPressed = false;
    }

    // Hood toggle
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && !DownWasPressed) {
        toggleHood();
        DownWasPressed = true;
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        DownWasPressed = false;
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");

    // Reset sensors
    rotationalLateral.reset_position();
    rotationalHorizontal.reset_position();

    // Calibrate inertial sensor
    inertialSensor.reset();
    while (inertialSensor.is_calibrating()) {
        pros::delay(100);
    }

    pros::lcd::set_text(2, "Calibration Complete");
}

void disabled() {}

void competition_initialize() {}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
    // Initialize robot position
    robotPosition.at(0, 0) = 0;
    robotPosition.at(1, 0) = 0;
    robotPosition.at(2, 0) = 0;

    rotationalHorizontal.reset_position();
    rotationalLateral.reset_position();

    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    hoodMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    inertialSensor.set_heading(90);
    robotPosition.at(2, 0) = 90;

    scraperState = false;
    hood.set_value(false);

    // Start tasks
    pros::Task odometry(odometry_task);
    pros::Task ramsete(ramsete_control_task);
    pros::Task motion(get_motion_task);
    pros::Task profile(motion_profile_task);

    // Simple autonomous - drive forward
    LeftDriveSmart.move_relative(5 * 360 / WHEEL_CIRCUMFERENCE, 100);
    RightDriveSmart.move_relative(5 * 360 / WHEEL_CIRCUMFERENCE, 100);
}

/**
 * Runs the driver control code.
 */
void opcontrol() {
    // Start odometry task
    pros::Task odometry(odometry_task);

    LeftDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    RightDriveSmart.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    hoodMotor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    while (true) {
        // Arcade drive
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        LeftDriveSmart.move(forward + turn);
        RightDriveSmart.move(forward - turn);

        input();

        pros::delay(10);
    }
}