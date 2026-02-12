#include "main.h"
#include "liblvgl/llemu.h"
#include "pros/screen.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>

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

// 积分限幅，防止积分饱和
const double DRIVE_INTEGRAL_CAP = 30.0;
const double TURN_INTEGRAL_CAP = 15.0;

// Error 曲线图：V5 屏幕 480x240，水平零线在中心，error 映射为 Y 偏移
const int SCREEN_W = 480;
const int SCREEN_H = 240;
const int GRAPH_CENTER_Y = 120;
const double DRIVE_ERROR_SCALE = 5.0;   // 像素/英寸
const double TURN_ERROR_SCALE = 2.0;   // 像素/度

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
}

// -------------------------------
// ODOMETRY UPDATE
// -------------------------------
void updateOdometry() {
    static double prevDeg = 0.0;

    double currDeg = forwardOdom.get_position();
    double dDeg = currDeg - prevDeg;
    prevDeg = currDeg;

    double dInches = dDeg * DIST_PER_DEG;

    robotHeadingDeg = imu.get_rotation();
    double headingRad = robotHeadingDeg * (PI / 180.0);

    robotX += dInches * cos(headingRad);
    robotY += dInches * sin(headingRad);
}

// -------------------------------
// DRIVE STRAIGHT (PID + IMU HOLD)
// -------------------------------
void driveDistance(double inches) {
    forwardOdom.reset_position();
    double prevError = inches;
    double integral = 0.0;

    // Error 曲线：清屏、画零线（目标），波形收敛即 PID 良好
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(0, GRAPH_CENTER_Y, SCREEN_W, GRAPH_CENTER_Y);
    pros::screen::set_pen(pros::Color::green);
    int prevX = -1, prevY = GRAPH_CENTER_Y;
    int iter = 0;

    while (true) {
        updateOdometry();

        double traveled =
            forwardOdom.get_position() * DIST_PER_DEG;
        double error = inches - traveled;
        double derivative = error - prevError;
        prevError = error;

        integral += error;
        integral = std::clamp(integral, -DRIVE_INTEGRAL_CAP, DRIVE_INTEGRAL_CAP);

        double power =
            DRIVE_kP * error + DRIVE_kI * integral + DRIVE_kD * derivative;
        power = std::clamp(power, -100.0, 100.0);

        double headingError = imu.get_rotation();
        double turn = headingError * 1.2; // >>> YOU TUNE <<<

        left_motors.move(power - turn);
        right_motors.move(power + turn);

        // 将 error 映射为屏幕 Y：零线在上方，正 error 向下
        int x = iter % SCREEN_W;
        int y = GRAPH_CENTER_Y + (int)(error * DRIVE_ERROR_SCALE);
        y = std::clamp(y, 0, SCREEN_H - 1);
        if (prevX >= 0)
            pros::screen::draw_line(prevX, prevY, x, y);
        else
            pros::screen::draw_pixel(x, y);
        prevX = x;
        prevY = y;
        iter++;

        if (fabs(error) < 0.5) break;
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
}

// -------------------------------
// TURN TO ANGLE (PID)
// -------------------------------
void turnToAngle(double targetDeg) {
    double prevError = targetDeg;
    double integral = 0.0;

    // Error 曲线：清屏、零线 + 波形
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::gray);
    pros::screen::draw_line(0, GRAPH_CENTER_Y, SCREEN_W, GRAPH_CENTER_Y);
    pros::screen::set_pen(pros::Color::blue);
    int prevX = -1, prevY = GRAPH_CENTER_Y;
    int iter = 0;

    while (true) {
        double curr = imu.get_rotation();
        double error = targetDeg - curr;
        double derivative = error - prevError;
        prevError = error;

        integral += error;
        integral = std::clamp(integral, -TURN_INTEGRAL_CAP, TURN_INTEGRAL_CAP);

        double power =
            TURN_kP * error + TURN_kI * integral + TURN_kD * derivative;
        power = std::clamp(power, -90.0, 90.0);

        left_motors.move(-power);
        right_motors.move(power);

        // error 映射为 Y（度 -> 像素）
        int x = iter % SCREEN_W;
        int y = GRAPH_CENTER_Y + (int)(error * TURN_ERROR_SCALE);
        y = std::clamp(y, 0, SCREEN_H - 1);
        if (prevX >= 0)
            pros::screen::draw_line(prevX, prevY, x, y);
        else
            pros::screen::draw_pixel(x, y);
        prevX = x;
        prevY = y;
        iter++;

        if (fabs(error) < 1.0) break;
        pros::delay(20);
    }

    left_motors.move(0);
    right_motors.move(0);
}

// -------------------------------
// AUTONOMOUS
// -------------------------------
void autonomous() {
    driveDistance(24);
    turnToAngle(90);
    driveDistance(12);

    motorIntake.move(100);
    pros::delay(500);
    motorIntake.move(0);
}

// -------------------------------
// OPERATOR CONTROL
// -------------------------------
void opcontrol() {

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
        updateOdometry();
        double traveledIn = forwardOdom.get_position() * DIST_PER_DEG;

        if (!master.is_connected()) {
            left_motors.move(0);
            right_motors.move(0);
            motorIntake.move(0);
            motorArm.move(0);
            wing.move(0);

            pros::lcd::set_text(0, "CONTROLLER LOST");
            pros::delay(20);
            continue;
        }

        // -------------------------------
        // DRIVE (TANK)
        // -------------------------------
        int leftPower =
            master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightPower =
            master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        left_motors.move(-leftPower);
        right_motors.move(rightPower * 0.8);

        // -------------------------------
        // INTAKE
        // -------------------------------
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            motorIntake.move(100);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            motorIntake.move(-100);
        else
            motorIntake.move(0);

        // -------------------------------
        // TUBE TOGGLE
        // -------------------------------
        if (master.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_L1)) {
            tubeExtended = !tubeExtended;
        }
        tube_piston.set_value(tubeExtended);

        // -------------------------------
        // WING
        // -------------------------------
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
            wing.move(127);
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
            wing.move(-127);
        else
            wing.move(0);

        // -------------------------------
        // ARM AUTO TRIGGER
        // -------------------------------
        if (master.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_UP)) {
            armState = UP;
            armCounter = 0;
        }

        // -------------------------------
        // ARM CONTROL
        // -------------------------------
        armCounter++;

        switch (armState) {
            case UP:
                motorArm.move(-127);
                if (armCounter > ARM_UP_DURATION) {
                    armState = KEEP;
                    armCounter = 0;
                }
                break;

            case KEEP:
                motorArm.move(-15);
                if (armCounter > ARM_KEEP_DURATION) {
                    armState = DOWN;
                    armCounter = 0;
                }
                break;

            case DOWN:
                motorArm.move(64);
                if (armCounter > ARM_DOWN_DURATION) {
                    armState = IDLE;
                    armCounter = 0;
                }
                break;

            case IDLE:
                if (master.get_digital(
                        pros::E_CONTROLLER_DIGITAL_X))
                    motorArm.move(-127);
                else if (master.get_digital(
                             pros::E_CONTROLLER_DIGITAL_Y) ||
                         master.get_digital(
                             pros::E_CONTROLLER_DIGITAL_DOWN))
                    motorArm.move(64);
                else
                    motorArm.move(0);
                break;
        }

        // -------------------------------
        // LCD 测试显示：航向、已走距离、坐标、追踪轮
        // -------------------------------
        pros::lcd::print(0, "H:%.1f deg  in:%.1f", robotHeadingDeg, traveledIn);
        pros::lcd::print(1, "X:%.1f  Y:%.1f", robotX, robotY);
        pros::lcd::print(2, "odom deg:%d", (int)forwardOdom.get_position());

        pros::delay(20);
    }
}
