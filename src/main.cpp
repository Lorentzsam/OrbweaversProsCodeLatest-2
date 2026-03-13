#include "main.h"
#include "liblvgl/llemu.h"
#include "pros/screen.hpp"
#include "pros/colors.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/llemu.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <vector>
#include "pros/link.hpp"


//ps: trying to make every line clear

// -------------------------------
// CONTROLLER
// -------------------------------
pros::Controller master(pros::E_CONTROLLER_MASTER);

// -------------------------------
// PNEUMATICS
// -------------------------------
pros::adi::Pneumatics tube_piston('G', false, false);
bool tubeExtended = false;

// -------------------------------
// DRIVE MOTOR GROUPS
// -------------------------------
pros::MotorGroup left_motors({3, -11, 12});
pros::MotorGroup right_motors({15, -16, 17});
pros::MotorGroup drivetrain({3, -11, 12, 15, -16, 17});

// -------------------------------
// MECHANISM MOTORS
// -------------------------------
pros::Motor motorIntake(18);
pros::Motor motorArm(20);
pros::Motor wing(14);

// -------------------------------
// ARM CONTROL
// -------------------------------
const int ARM_UP_DURATION   = 25;
const int ARM_KEEP_DURATION = 10;
const int ARM_DOWN_DURATION = 25;

enum ArmState {
    UP,
    KEEP,
    DOWN,
    IDLE
};

ArmState armState = IDLE;
int armCounter = 0;

// -------------------------------
// ODOMETRY SENSORS
// -------------------------------
pros::Rotation forwardOdom(4);   // tracking wheel
pros::Imu imu(6);                // IMU

// -------------------------------
// ODOMETRY STATE
// -------------------------------
double robotX = 0.0;
double robotY = 0.0;
double robotHeadingDeg = 0.0;

// -------------------------------
// CONSTANTS
// -------------------------------
const double PI = 3.141592653589793;

// >>> YOU TUNE THIS <<<
const double TRACK_WHEEL_DIAMETER_IN = 2.75;

const double DIST_PER_DEG =
    (TRACK_WHEEL_DIAMETER_IN * PI) / 360.0;

// PROS Rotation returns centidegrees (1/100 degree), convert to degrees
const double CENTIDEG_TO_DEG = 1.0 / 100.0;

// -------------------------------
// PID CONSTANTS (START VALUES)
// -------------------------------

// >>> YOU TUNE THESE <<<
double DRIVE_kP = 6.0;
double DRIVE_kI = 0.0;
double DRIVE_kD = 0.4;

double TURN_kP = 2.2;
double TURN_kI = 0.0;
double TURN_kD = 0.15;

// Integral cap to prevent windup
const double DRIVE_INTEGRAL_CAP = 30.0;
const double TURN_INTEGRAL_CAP = 15.0;

// Screen dimensions
const int SCREEN_W = 480;
const int SCREEN_H = 240;

const double DRIVE_ERROR_SCALE = 5.0;
const double TURN_ERROR_SCALE  = 2.0;

// -------------------------------
// PID Speed Curve Visualization (V5 Brain Screen)
// -------------------------------
const int GRAPH_WIDTH    = 400;
const int GRAPH_HEIGHT   = 200;
const int GRAPH_LEFT     = 20;
const int GRAPH_TOP      = 15;
const int GRAPH_CENTER_Y = GRAPH_TOP + GRAPH_HEIGHT / 2;  // 115

const double TARGET_SCALE     = (GRAPH_HEIGHT / 2.0 - 5) / 127.0;
const double ACTUAL_RPM_SCALE = (GRAPH_HEIGHT / 2.0 - 5) / 200.0;

static std::vector<int16_t> targetYHistory(GRAPH_WIDTH, GRAPH_CENTER_Y);
static std::vector<int16_t> actualYHistory(GRAPH_WIDTH, GRAPH_CENTER_Y);
static int graphWriteIndex = 0;

// -------------------------------
// DRAW VELOCITY GRAPH
// Draw two speed curves on the Brain screen (red=target, green=actual)
// -------------------------------
void drawVelocityGraph(int targetVel, double actualRPM) {
    int targetY = GRAPH_CENTER_Y - (int)(targetVel * TARGET_SCALE);
    int actualY = GRAPH_CENTER_Y - (int)(actualRPM * ACTUAL_RPM_SCALE);

    if (targetY < GRAPH_TOP) targetY = GRAPH_TOP;
    if (targetY > GRAPH_TOP + GRAPH_HEIGHT) targetY = GRAPH_TOP + GRAPH_HEIGHT;
    if (actualY < GRAPH_TOP) actualY = GRAPH_TOP;
    if (actualY > GRAPH_TOP + GRAPH_HEIGHT) actualY = GRAPH_TOP + GRAPH_HEIGHT;

    targetYHistory[graphWriteIndex] = (int16_t)targetY;
    actualYHistory[graphWriteIndex] = (int16_t)actualY;

    int nextIdx = (graphWriteIndex + 1) % GRAPH_WIDTH;

    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase_rect(GRAPH_LEFT, GRAPH_TOP,
                             GRAPH_LEFT + GRAPH_WIDTH,
                             GRAPH_TOP + GRAPH_HEIGHT);

    pros::screen::set_pen(pros::Color::grey);
    pros::screen::draw_line(GRAPH_LEFT, GRAPH_CENTER_Y,
                            GRAPH_LEFT + GRAPH_WIDTH, GRAPH_CENTER_Y);

    pros::screen::set_pen(pros::Color::red);
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
        pros::screen::draw_line(
            GRAPH_LEFT + i,     targetYHistory[i],
            GRAPH_LEFT + i + 1, targetYHistory[i + 1]);
    }

    pros::screen::set_pen(pros::Color::green);
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
        pros::screen::draw_line(
            GRAPH_LEFT + i,     actualYHistory[i],
            GRAPH_LEFT + i + 1, actualYHistory[i + 1]);
    }

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_SMALL, GRAPH_LEFT + GRAPH_WIDTH + 8, GRAPH_TOP,      "R:Target");
    pros::screen::print(pros::E_TEXT_SMALL, GRAPH_LEFT + GRAPH_WIDTH + 8, GRAPH_TOP + 12, "G:Actual");
    pros::screen::print(pros::E_TEXT_SMALL, GRAPH_LEFT + GRAPH_WIDTH + 8, GRAPH_TOP + 28, "T:%d", targetVel);
    pros::screen::print(pros::E_TEXT_SMALL, GRAPH_LEFT + GRAPH_WIDTH + 8, GRAPH_TOP + 40, "A:%.0f", actualRPM);

    graphWriteIndex = nextIdx;
}

// -------------------------------
// INITIALIZE
// -------------------------------
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "CALIBRATING IMU");

    imu.reset();
    while (imu.is_calibrating()) {
        pros::delay(10);
    }

    forwardOdom.reset_position();
    robotX = robotY = 0.0;

    pros::lcd::set_text(0, "READY");
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Run Autonomous for PID curve");
}

// -------------------------------
// ODOMETRY UPDATE
// -------------------------------
void updateOdometry() {
    static double prevDeg = 0.0;

    double currDeg = forwardOdom.get_position() * CENTIDEG_TO_DEG;
    double dDeg    = currDeg - prevDeg;
    prevDeg        = currDeg;

    double dInches = dDeg * DIST_PER_DEG;

    robotHeadingDeg  = imu.get_rotation();
    double headingRad = robotHeadingDeg * (PI / 180.0);

    robotX += dInches * cos(headingRad);
    robotY += dInches * sin(headingRad);
}

// -------------------------------
// DRIVE STRAIGHT (PID + IMU HOLD)
// FIX: added imu.tare() so heading correction is relative to start of each move
// FIX: removed unused prevX, prevY, iter variables
// -------------------------------
void driveDistance(double inches) {
    forwardOdom.reset_position();
    imu.tare();                      // <<< FIX: zero heading at start of each drive
    double prevError = inches;
    double integral  = 0.0;

    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(0, GRAPH_CENTER_Y, SCREEN_W, GRAPH_CENTER_Y);
    pros::screen::set_pen(pros::Color::green);

    while (true) {
        updateOdometry();

        double traveled =
            forwardOdom.get_position() * CENTIDEG_TO_DEG * DIST_PER_DEG;
        double error      = inches - traveled;
        double derivative = error - prevError;
        prevError         = error;

        integral += error;
        if      (integral >  DRIVE_INTEGRAL_CAP) integral =  DRIVE_INTEGRAL_CAP;
        else if (integral < -DRIVE_INTEGRAL_CAP) integral = -DRIVE_INTEGRAL_CAP;

        double power =
            DRIVE_kP * error + DRIVE_kI * integral + DRIVE_kD * derivative;
        if      (power >  100.0) power =  100.0;
        else if (power < -100.0) power = -100.0;

        double headingError = imu.get_rotation();   // now relative to start of this move
        double turn         = headingError * 1.2;   // >>> YOU TUNE this multiplier

        left_motors.move(power - turn);
        right_motors.move(power + turn);

        pros::lcd::print(0, "Drv E:%d in:%d", (int)error, (int)traveled);
        pros::lcd::print(1, "Pwr:%d",          (int)power);

        if (fabs(error) < 0.5) break;
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
    pros::lcd::set_text(0, "Drv OK");
}

// -------------------------------
// TURN TO ANGLE (PID)
// FIX: removed unused prevX, prevY, iter variables
// -------------------------------
void turnToAngle(double targetDeg) {
    double prevError = targetDeg;
    double integral  = 0.0;

    pros::screen::set_eraser(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(0, GRAPH_CENTER_Y, SCREEN_W, GRAPH_CENTER_Y);
    pros::screen::set_pen(pros::Color::blue);

    while (true) {
        double curr       = imu.get_rotation();
        double error      = targetDeg - curr;
        double derivative = error - prevError;
        prevError         = error;

        integral += error;
        if      (integral >  TURN_INTEGRAL_CAP) integral =  TURN_INTEGRAL_CAP;
        else if (integral < -TURN_INTEGRAL_CAP) integral = -TURN_INTEGRAL_CAP;

        double power =
            TURN_kP * error + TURN_kI * integral + TURN_kD * derivative;
        if      (power >  90.0) power =  90.0;
        else if (power < -90.0) power = -90.0;

        left_motors.move(-power);
        right_motors.move(power);

        pros::lcd::print(0, "Turn E:%d deg", (int)error);
        pros::lcd::print(1, "cur:%d Pwr:%d", (int)curr, (int)power);

        if (fabs(error) < 1.0) break;
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
    pros::lcd::set_text(0, "Turn OK");
}

// -------------------------------
// AUTONOMOUS
// FIX: sensor check loop added at start — watch LCD for 2 sec before robot moves
//      Push robot by hand during this window:
//        - Odom should change when pushed forward/back
//        - IMU should change when rotated
//        - Calibrating should read 0 (not 1)
//      If any sensor stays stuck at 0, that's your problem.
// -------------------------------
void autonomous() {

    // --- SENSOR CHECK (2 seconds before any movement) ---
    for (int i = 0; i < 100; i++) {
        pros::lcd::print(0, "IMU: %d deg",     (int)imu.get_rotation());
        pros::lcd::print(1, "Odom: %d",        (int)forwardOdom.get_position());
        pros::lcd::print(2, "Calibrating: %d", (int)imu.is_calibrating());
        pros::delay(20);
    }
    // --- END SENSOR CHECK ---

    // STEP 1: Drive forward
    driveDistance(12);              // >>> TUNE: change to your actual first distance (inches)
    pros::delay(200);

    // STEP 2: Intake
    motorIntake.move(-127);         // intake forward — check direction is correct
    pros::delay(800);               // >>> TUNE: ms to run intake
    motorIntake.move(0);
    pros::delay(200);

    // STEP 3: Turn
    turnToAngle(90);                // >>> TUNE: positive=right, negative=left
    pros::delay(200);

    // STEP 4: Drive forward again
    driveDistance(24);              // >>> TUNE: change to your actual second distance (inches)
    pros::delay(200);

    // STEP 5: Outtake
    motorIntake.move(127);          // reverse intake = outtake
    pros::delay(800);               // >>> TUNE: ms to run outtake
    motorIntake.move(0);
}

// -------------------------------
// OPERATOR CONTROL
// Keypad: Left/Right Joystick Y = Chassis | R1/R2 = Intake Forward/Reverse
//         L1 = Airway Switch | RIGHT/LEFT = Wing | UP = Arm sequence
//         X = Arm up | Y/DOWN = Arm down
// -------------------------------
void opcontrol() {
    // Wait up to ~3 sec for controller to connect
    for (int wait = 0; wait < 150 && !master.is_connected(); wait++) {
        pros::delay(20);
    }

    while (!master.is_connected()) {
        left_motors.move(0);
        right_motors.move(0);
        motorIntake.move(0);
        motorArm.move(0);
        wing.move(0);
        pros::lcd::set_text(0, "WAITING FOR CONTROLLER");
        pros::delay(20);
    }

    pros::lcd::set_text(0, "CONTROLLER CONNECTED");

    while (true) {

        // -------------------------------
        // DRIVE (TANK)
        // -------------------------------
        int leftPower  = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        left_motors.move(-leftPower);
        right_motors.move(rightPower);

        // -------------------------------
        // PID Speed Curve: Target vs Actual (Brain Screen)
        // -------------------------------
        int targetVel = (leftPower + rightPower) / 2;
        double actualRPM = 0;
        auto leftVel  = left_motors.get_actual_velocity_all();
        auto rightVel = right_motors.get_actual_velocity_all();
        size_t totalMotors = leftVel.size() + rightVel.size();
        if (totalMotors > 0) {
            for (double v : leftVel)  actualRPM -= v;  // left motors are reversed
            for (double v : rightVel) actualRPM += v;
            actualRPM /= (double)totalMotors;
        }
        drawVelocityGraph(targetVel, actualRPM);
        // FIX: removed duplicate right_motors.move(rightPower * 0.8) that was here

        // -------------------------------
        // INTAKE
        // -------------------------------
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            motorIntake.move(127);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            motorIntake.move(-127);
        else
            motorIntake.move(0);

        // -------------------------------
        // TUBE TOGGLE
        // -------------------------------
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            tubeExtended = !tubeExtended;
        }
        tube_piston.set_value(tubeExtended);

        // -------------------------------
        // WING (RIGHT = extend, LEFT = retract)
        // -------------------------------
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
            wing.move(127);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
            wing.move(-127);
        else
            wing.move(0);

        // -------------------------------
        // ARM AUTO TRIGGER (UP button)
        // -------------------------------
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            armState  = UP;
            armCounter = 0;
        }

        // -------------------------------
        // ARM STATE MACHINE
        // -------------------------------
        armCounter++;

        switch (armState) {
            case UP:
                motorArm.move(-127);
                if (armCounter > ARM_UP_DURATION) {
                    armState   = KEEP;
                    armCounter = 0;
                }
                break;

            case KEEP:
                motorArm.move(-15);
                if (armCounter > ARM_KEEP_DURATION) {
                    armState   = DOWN;
                    armCounter = 0;
                }
                break;

            case DOWN:
                motorArm.move(64);
                if (armCounter > ARM_DOWN_DURATION) {
                    armState   = IDLE;
                    armCounter = 0;
                }
                break;

            case IDLE:
                if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
                    motorArm.move(-127);
                else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) ||
                         master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
                    motorArm.move(64);
                else
                    motorArm.move(0);
                break;
        }

        // -------------------------------
        // LCD DISPLAY
        // -------------------------------
        pros::lcd::print(1, "X:%d Y:%d",   (int)robotX, (int)robotY);
        pros::lcd::print(2, "odom deg:%d", (int)(forwardOdom.get_position() * CENTIDEG_TO_DEG));

        pros::delay(20);
    }
}