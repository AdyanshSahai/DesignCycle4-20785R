#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

/*
Below are all of the ports on the brain:

Left Drivetrain motors: 2, 3, 4 
Right Drive: 7, 8, 10

Bottom rollers: 11
Middle Roller: -13
Top roller: -19
Piston scraper: ?

Radio: 5

Optical Sensor: 7

Y axis encoder: 16
Gyro: 15

*/


int timeNew=0;
int timeOld=0;
int changeTime=0;

int leftSpeed=0;
int rightSpeed=0;
int bottomSpeed=0;
int middleSpeed=0;
int topSpeed=0;
int sortSpeed=0;
int hue;

bool wingToggle = false;
bool scraperToggle = false;
bool wingLast = false;
bool scraperLast = false;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//roller motors
pros::Motor bottomRoller(3);
pros::Motor topRoller(2);

// motor groups
pros::MotorGroup leftMotors({10, 9, 8}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-7, -6, -5}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// pneumatics
pros::ADIDigitalOut wing('B', false);
pros::ADIDigitalOut scraper('A', false);


// Inertial Sensor on port 10
pros::Imu imu(21);

//pros::Optical optical_sensor(7);

bool risingEdgeToggle(bool pressing, bool &lastState, bool &toggleState){
    bool risingEdge = pressing && !lastState; // detect rising edge
    if (risingEdge) {
        toggleState = !toggleState; // toggle state on rising edge
    }
    lastState = pressing; // update last state
    return toggleState;
}

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain     rpm is 450
                              8 // horizontal drift is 2 for drift drive. If we have traction wheels, it is 8.
);

// lateral motion controller
lemlib::ControllerSettings linearController(14, // proportional gain (kP) how fast it goes
                                            0, // integral gain (kI) static adjustment
                                            75, // derivative gain (kD) easing
                                            0, // anti windup
                                            0.1, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(6, // proportional gain (kP) speed
                                             0, // integral gain (kI) stitac  adjust
                                             50, // derivative gain (kD) easing
                                             0, // anti windup
                                             0.05, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(5, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.04 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(5, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.05 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen

    topRoller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.calibrate(); // calibrate sensors

    //optical_sensor.set_led_pwm(100);

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

			pros::lcd::print(4, "Brain Battery %f", pros::battery::get_capacity(), "%"); // brain battery
			pros::lcd::print(5, "Controller Battery: %f", controller.get_battery_capacity(), "%"); // controller battery
            //pros::lcd::print(6, "Hue: %f", optical.get_hue());

            // pros::delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}


// get a path used for pure pursuit
// this needs to be put outside a function
//ASSET(path27pt1_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.curvature(leftY, rightX);

		bool L1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
		bool L2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

		bool R1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
		bool R2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

        bool B = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool Y = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);


		//intake and hold
		if (L2==1){
			bottomRoller.move(127);
            topRoller.brake();
		}else if (L1==1){
			bottomRoller.move(127);
            topRoller.move(127);

		//bottom outtake
		}else if (R2==1){
            bottomRoller.move(-127);
            topRoller.move(-127);
		
		// top outtake
        }else if (R1==1){
            bottomRoller.move(127);
            topRoller.move(127);

		// or brake.
        }else{
            bottomRoller.brake();
            topRoller.brake();
        }

        //wing toggle
        risingEdgeToggle(Y, wingLast, wingToggle);
        wing.set_value(wingToggle); 
        //scraper toggle
        risingEdgeToggle(B, scraperLast, scraperToggle);
        scraper.set_value(scraperToggle);



        //Rerun


		// leftSpeed = leftMotors.get_actual_velocity();
		// rightSpeed = rightMotors.get_actual_velocity();
		// bottomSpeed = bottomRoller.get_actual_velocity();
		// middleSpeed = middleRoller.get_actual_velocity();
		// topSpeed = topRoller.get_actual_velocity();

		// FILE* usd_file_write = fopen("/usd/rerun.txt", "a");
		// fprintf(usd_file_write, "leftMotors.move_velocity(%i); \n", leftSpeed);
		// fprintf(usd_file_write, "rightMotors.move_velocity(%i); \n", rightSpeed);
		// fprintf(usd_file_write, "bottomRoller.move_velocity(%i); \n", bottomSpeed);
		// fprintf(usd_file_write, "middleRoller.move_velocity(%i); \n", middleSpeed);
		// fprintf(usd_file_write, "topRoller.move_velocity(%i); \n", topSpeed);

		// timeNew = pros::millis();
		// changeTime = timeNew - timeOld;
		// timeOld = pros::millis();



		// fprintf(usd_file_write, "pros::delay(%d); \n", changeTime);

		// fclose(usd_file_write);

        // end the thing


        // pros::delay to save resources
        pros::delay(10);

		
    }
}

