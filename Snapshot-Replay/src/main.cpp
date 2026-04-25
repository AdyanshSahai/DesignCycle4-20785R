#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdio>
#include <cstring>

/*
Ports:
Left Drivetrain: 10, 3, 4
Right Drive: 5, 7, 8
Bottom rollers: 13
Middle roller: -14
Top roller: -16
Scraper: A (Port 1)
Wing: B (Port 2)
Radio: 21
Optical: 20
Y axis encoder: -17
Gyro: 19
*/

// button edge states for pressed()
bool prevUp    = false;
bool prevDown  = false;
bool prevLeft  = false;
bool prevRight = false;
bool prevB     = false;
bool prevY     = false;

// pose snapshot, updated each loop tick
double posX;
double posY;
double posTheta;

// roller velocity snapshots for recording
int bottomSpeed = 0;
int topSpeed    = 0;


// ---- HARDWARE ----

pros::Controller controller(pros::E_CONTROLLER_MASTER);  // master controller

pros::Motor bottomRoller(-3);    // port 13
pros::Motor topRoller(-2);      // port 16 rev

pros::MotorGroup leftMotors({10, 9, 8}, pros::MotorGearset::blue);  // left side
pros::MotorGroup rightMotors({-7, -6, -5},    pros::MotorGearset::blue);  // right side

pros::Imu     imu(19);            // IMU port 19
pros::Optical optical_sensor(20); // optical port 20

pros::adi::DigitalOut scraper('B', false); // scraper port C
pros::adi::DigitalOut wing('A', false);    // wing port A

bool wingState = false;  // track state
bool scraperState = false; // track state

pros::Rotation verticalEnc(-17);                                                   // encoder port 17
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.88);   // 2.75" wheel


// ---- LEMLIB SETUP ----

lemlib::Drivetrain drivetrain(&leftMotors,             // left motor group
                              &rightMotors,            // right motor group
                              13.75,                   // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // new 3.25" omnis
                              450,                     // drivetrain rpm
                              6                        // horizontal drift
);

lemlib::ControllerSettings linearController(15,  // kP
                                            0,   // kI
                                            2,   // kD
                                            0,   // anti windup
                                            0,   // small error range (in)
                                            0,   // small error timeout (ms)
                                            0,   // large error range (in)
                                            0,   // large error timeout (ms)
                                            0    // slew
);

lemlib::ControllerSettings angularController(2,    // kP
                                             0,    // kI
                                             10,   // kD
                                             0,    // anti windup
                                             0.5,  // small error range (deg)
                                             100,  // small error timeout (ms)
                                             2,    // large error range (deg)
                                             500,  // large error timeout (ms)
                                             0     // slew
);

lemlib::OdomSensors sensors(nullptr,  // vertical tracking wheel 1
                            nullptr,  // vertical tracking wheel 2
                            nullptr,  // horizontal tracking wheel
                            nullptr,  // horizontal tracking wheel 2
                            &imu      // inertial sensor
);

lemlib::ExpoDriveCurve throttleCurve(5, 10, 1.04); // throttle curve
lemlib::ExpoDriveCurve steerCurve(5, 10, 1.04);    // steer curve

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


// returns true only on the rising edge of a button press
bool pressed(bool current, bool &previous) {
    if (current && !previous) {
        previous = current;
        return true;
    }
    previous = current;
    return false;
}


// ---- REPLAY EXECUTION ----
// takes one line from the SD card file and runs the matching action.
// token format is whatever opcontrol writes, e.g. "MOVE_POSE_FWD 12.00 -5.00 90.00"
// newline is stripped before any comparisons so fgets doesn't mess up strcmp.
void execute_command(char* line) {
    line[strcspn(line, "\n")] = 0;  // strip newline
    if (line[0] == '\0') return;    // skip blank

    if (strncmp(line, "MOVE_POSE_FWD", 13) == 0) {        // forward move
        double x, y, theta;
        if (sscanf(line, "MOVE_POSE_FWD %lf %lf %lf", &x, &y, &theta) == 3) {
            chassis.moveToPose(x, y, theta, 4000, {.forwards = true});
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "MOVE_POSE_BWD", 13) == 0) { // backward move
        double x, y, theta;
        if (sscanf(line, "MOVE_POSE_BWD %lf %lf %lf", &x, &y, &theta) == 3) {
            chassis.moveToPose(x, y, theta, 4000, {.forwards = false});
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "TURN_TO", 7) == 0) {        // heading turn
        double theta;
        if (sscanf(line, "TURN_TO %lf", &theta) == 1) {
            chassis.turnToHeading(theta, 1000);
            chassis.waitUntilDone();
        }
    } else if (strncmp(line, "INTAKE_STATE", 12) == 0) {  // roller state
        int l1, l2, r1, r2;
        if (sscanf(line, "INTAKE_STATE %d %d %d %d", &l1, &l2, &r1, &r2) == 4) {
            if (l2) {
                bottomRoller.move(-127); topRoller.brake();
            } else if (l1) {
                bottomRoller.move(127);  topRoller.move(127);
            } else if (r2) {
                bottomRoller.move(-127); topRoller.brake();
            } else if (r1) {
                bottomRoller.move(-127); topRoller.move(-127);
            } else {
                bottomRoller.brake(); topRoller.brake();
            }
            pros::delay(1000);  // hold state
        }
    } else if (strcmp(line, "SCRAPER_TOGGLE") == 0) {      // flip scraper
        scraperState = !scraperState;
        scraper.set_value(scraperState);
        pros::delay(200);   // piston settle
    } else if (strcmp(line, "WING_TOGGLE") == 0) {        // flip wing
        wingState = !wingState;
        wing.set_value(wingState);
        pros::delay(200);   // piston settle
    }
}


void initialize() {
    pros::lcd::initialize();                              // init screen
    topRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);  // hold top
    chassis.calibrate();                                  // calibrate IMU
    optical_sensor.set_led_pwm(100);                      // full brightness

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f",     chassis.getPose().x);     // print x
            pros::lcd::print(1, "Y: %f",     chassis.getPose().y);     // print y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // print heading
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::lcd::print(4, "Bat: %f",  pros::battery::get_capacity(), "%");    // brain battery
            pros::lcd::print(5, "Ctrl: %f", controller.get_battery_capacity(), "%"); // ctrl battery
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}


void autonomous() {
    // ---- REPLAY ----
    // open the token file that was built during driver control and run each
    // line through execute_command. if the file isn't there, nothing happens.
    FILE* f = fopen("/usd/replay.txt", "r"); // open replay
    if (f) {
        pros::lcd::print(0, "Replaying...");    // show status
        char line[256];                         // line buffer
        while (fgets(line, sizeof(line), f) != NULL) {
            execute_command(line);              // run command
        }
        fclose(f);                              // close file
        pros::delay(1000);                      // brief settle
        pros::lcd::print(0, "Replay Done");     // done message
    }
}


void opcontrol() {
    while (true) {

        // ---- DRIVE ----
        // left stick is throttle, right stick is turn, curvature mode
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);   // throttle value
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);  // steering value
        chassis.curvature(leftY, -rightX);                                       // move robot


        // ---- BUTTON READS ----
        // snapshot everything once per tick so each button is checked consistently
        bool L1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);    // intake full
        bool L2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);    // hold outtake
        bool R1    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);    // top out
        bool R2    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);    // bottom out
        bool up    = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);    // save fwd
        bool down  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);  // save bwd
        bool right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT); // save intake
        bool left  = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);  // save turn
        bool b     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);     // scraper toggle
        bool y     = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);     // wing toggle


        // ---- INTAKE / OUTTAKE ----
        // L2 dumps bottom only (top holds), L1 runs both in,
        // R2 same as L2, R1 runs both out
        if (L2) {
            bottomRoller.move(-127);  // outtake bottom
            topRoller.brake();        // hold top
        } else if (L1) {
            bottomRoller.move(-100);   // intake bottom
            topRoller.move(127);      // intake top
        } else if (R2) {
            bottomRoller.move(127);   // intake bottom
            topRoller.move(127);      // intake top
        } else if (R1) {
            bottomRoller.move(-127);  // outtake bottom
            topRoller.move(-127);     // outtake top
        } else {
            bottomRoller.brake();     // stop bottom
            topRoller.brake();        // stop top
        }


        // ---- SCRAPER ----
        // B toggles the scraper each press, same pattern as wing on Y
        if (pressed(b, prevB)) {
            scraperState = !scraperState;
            scraper.set_value(scraperState);    // toggle scraper
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "SCRAPER_TOGGLE\n"); // scraper token
                fclose(f);
                pros::lcd::print(6, "Saved Scraper");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }


        // ---- WING ----
        // Y flips the wing each press. toggle and record both happen here so
        // pressed() is only called once — calling it twice would miss the record.
        if (pressed(y, prevY)) {
            wingState = !wingState;
            wing.set_value(wingState);          // toggle wing
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "WING_TOGGLE\n");    // wing token
                fclose(f);
                pros::lcd::print(6, "Saved Wing");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }


        // ---- POSE SNAPSHOT ----
        // grab pose once so all the recording blocks below see the same values
        lemlib::Pose pose = chassis.getPose(); // read pose
        posX     = pose.x;      // store x
        posY     = pose.y;      // store y
        posTheta = pose.theta;  // store heading

        bottomSpeed = bottomRoller.get_actual_velocity(); // bottom rpm
        topSpeed    = topRoller.get_actual_velocity();    // top rpm


        // ---- RECORDING ----
        // d-pad writes tokenized commands to the SD card for replay in auto.
        // UP = forward moveToPose, DOWN = backward, LEFT = turn, RIGHT = intake state.
        // scraper on/off go in a separate chain so they don't block the d-pad.
        if (pressed(up, prevUp)) {
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "MOVE_POSE_FWD %.2f %.2f %.2f\n", posX, posY, posTheta); // forward token
                fclose(f);
                pros::lcd::print(6, "Saved FWD");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(down, prevDown)) {
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "MOVE_POSE_BWD %.2f %.2f %.2f\n", posX, posY, posTheta); // backward token
                fclose(f);
                pros::lcd::print(6, "Saved BWD");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(left, prevLeft)) {
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "TURN_TO %.2f\n", posTheta);  // turn token
                fclose(f);
                pros::lcd::print(6, "Saved Turn");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }

        } else if (pressed(right, prevRight)) {
            FILE* f = fopen("/usd/replay.txt", "a");
            if (f) {
                fprintf(f, "INTAKE_STATE %d %d %d %d\n", (int)L1, (int)L2, (int)R1, (int)R2); // intake token
                fclose(f);
                pros::lcd::print(6, "Saved Intake");
            } else { pros::lcd::print(6, "USD WRITE FAILED"); }
        }



        pros::delay(20);  // loop rate
    }
}
