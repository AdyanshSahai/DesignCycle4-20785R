#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

/*
Below are all of the ports on the brain:

Left Drivetrain motors: 7, 6, 5
Right Drive: 10, 9, 14


Bottom rollers: +3
Middle roller: -
Top roller: -

Radio: 4

Optical Sensor: 20

Gyro: 21

Wing: B
Scraper: H

*/


int leftSpeed=0;
int rightSpeed=0;
int bottomSpeed=0;
int middleSpeed=0;
int topSpeed=0;
int sortSpeed=0;
int hue;

bool prevUp = false;
bool prevDown = false;
bool prevLeft = false;
bool prevRight = false;
bool prevB = false;
bool prevX = false;

double posX;    
double posY;
double posTheta;


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//roller motors
pros::Motor bottomRoller(3);
pros::Motor middleRoller(2);
pros::Motor topRoller(6);

// motor groups
pros::MotorGroup leftMotors({7, -14, -5}, pros::MotorGearset::blue); // left motor group - ports 2, 3, 4 (all reversed)
pros::MotorGroup rightMotors({-10, 9, 8}, pros::MotorGearset::blue); // right motor group - ports 5, 7, 8 

// Inertial Sensor on port 19
pros::Imu imu(21);

// Optical Sensor on port 20
pros::Optical optical_sensor(20);

// scraper pneumatics
pros::ADIDigitalOut scraper (H, 0);
pros::ADIDigitalOut wing (B, 0);

// // tracking wheel
// // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-17);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.88);	

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              13.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain     rpm is 450
                              6 // horizontal drift is 2 for drift drive. If we have traction wheels, it is 8.
);

// lateral motion controller
lemlib::ControllerSettings linearController(15, // proportional gain (kP) how fast it goes
                                            0, // integral gain (kI) static adjustment
                                            2, // derivative gain (kD) easing
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP) speed
                                             0, // integral gain (kI) stitac  adjust
                                             10, // derivative gain (kD) easing
                                             0, // anti windup
                                             0.5, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             2, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
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
                                  1.04 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

bool pressed(bool current, bool &previous) {
    if (current && !previous) {
        previous = current;
        return true;
    }
    previous = current;
    return false;
}


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

    optical_sensor.set_led_pwm(100);

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

 
/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */

void autonomous() {
    chassis.moveToPose(-0.68,-39.0,10.8, 4000, {.forwards=false});
    chassis.waitUntilDone();

    bottomRoller.move(-127);
    middleRoller.move(-127);
    topRoller.move(-127);
    


}

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

        bool up = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool right = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool left = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);

        bool b = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool x = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

		// Example: Toggle scraper on B press (instead of hold)
    	if (pressed(b, prevB)) {
        	scraper.set_value(!scraper.get_value());  // Toggle: if on, turn off; if off, turn on
    	}

    	// Example: Toggle wing on X press
    	if (pressed(x, prevX)) {
        	wing.set_value(!wing.get_value());
    	}

		//intake and hold
		if (L2==1){
			bottomRoller.move(-127);
            middleRoller.move(-127);
            topRoller.brake();
		}else if (L1==1){
			bottomRoller.move(127);
            middleRoller.move(127);
            topRoller.move(127);

		//bottom outtake
		}else if (R2==1){
            bottomRoller.move(-127);
            middleRoller.move(-127);
            topRoller.brake();
		
		// top outtake
        }else if (R1==1){
            bottomRoller.move(-127);
            middleRoller.move(-127);
            topRoller.move(-127);

        }else{
            bottomRoller.brake();
            middleRoller.brake();
            topRoller.brake();
        }





        //pros::delay to save resources
        pros::delay(10);

		
    }
}



