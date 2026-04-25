#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

/*

1, 6, 9, 12, 20
Below are all of the ports on the brain:

Left Drivetrain motors: 10, 3, 4
Right Drive: 5, 7, 8


Bottom rollers: +13
Middle roller: -14
Top roller: -16
Hammer scraper: A

Radio: 21

Optical Sensor: 20

Y axis encoder: 17
Gyro: 19

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
pros::Motor bottomRoller(13);
pros::Motor middleRoller(-14);
pros::Motor topRoller(-16);

// motor groups
pros::MotorGroup leftMotors({-10, -3, -4}, pros::MotorGearset::blue); // left motor group - ports 2, 3, 4 (all reversed)
pros::MotorGroup rightMotors({5, 7, 8}, pros::MotorGearset::blue); // right motor group - ports 5, 7, 8 

// Inertial Sensor on port 19
pros::Imu imu(19);

// Optical Sensor on port 20
pros::Optical optical_sensor(20);

// scraper pneumatics
pros::ADIDigitalOut scraper (1, 0);

// // tracking wheel
// // vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-17);
// // vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 6.88);	

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
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
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

        // scraper code for loading zone
        if(b==1){
            scraper.set_value(1);
        }else if(x==1){
            scraper.set_value(0);
        }


        // // TEAM ROGUE REPLAY       


		lemlib::Pose pose = chassis.getPose();

        posX = pose.x;
        posY = pose.y;
        posTheta = pose.theta;

        bottomSpeed = bottomRoller.get_actual_velocity();
		middleSpeed = middleRoller.get_actual_velocity();
		topSpeed = topRoller.get_actual_velocity();

        if (pressed(up, prevUp)){
            

            FILE* usd_file_write = fopen("/usd/replay.txt", "a");

            if (usd_file_write != NULL) {
                fprintf(usd_file_write, "   bottomRoller.move_velocity(%i); \n", bottomSpeed);
                fprintf(usd_file_write, "   middleRoller.move_velocity(%i); \n", middleSpeed);
		        fprintf(usd_file_write, "   topRoller.move_velocity(%i); \n", topSpeed);
                fprintf(usd_file_write, "   chassis.moveToPose(%.2f, %.2f, %.2f, 4000); \n \n", posX, posY, posTheta);
                fclose(usd_file_write);
            } else {pros::lcd::print(6, "USD WRITE FAILED");}


        }
        
        else if (pressed(down, prevDown)){

            FILE* usd_file_write = fopen("/usd/replay.txt", "a");
            if (usd_file_write != NULL) {
                fprintf(usd_file_write, "   bottomRoller.move_velocity(%i); \n", bottomSpeed);
                fprintf(usd_file_write, "   middleRoller.move_velocity(%i); \n", middleSpeed);
                fprintf(usd_file_write, "   topRoller.move_velocity(%i); \n", topSpeed);
                fprintf(usd_file_write, "   chassis.moveToPose(%.2f, %.2f, %.2f, 4000, {.forwards=false}); \n \n", posX, posY, posTheta);
                fclose(usd_file_write);
            } else {pros::lcd::print(6, "USD WRITE FAILED");}
        }
        
        // WORKING
        else if (pressed(right, prevRight)){
        
            FILE* usd_file_write = fopen("/usd/replay.txt", "a");
            if (usd_file_write != NULL) {
                fprintf(usd_file_write, "   bottomRoller.move_velocity(%i); \n", bottomSpeed);
                fprintf(usd_file_write, "   middleRoller.move_velocity(%i); \n", middleSpeed);
                fprintf(usd_file_write, "   topRoller.move_velocity(%i); \n", topSpeed);
                fprintf(usd_file_write, "   chassis.turnToHeading(%.2f, 1000); \n \n", posTheta);
                fclose(usd_file_write);
            } else {pros::lcd::print(6, "USD WRITE FAILED");}
        } 

        if (pressed(b, prevB)){
            FILE* usd_file_write = fopen("/usd/replay.txt", "a");
            if (usd_file_write != NULL) {
                fprintf(usd_file_write, "   scraper.set_value(1); \n");
                fclose(usd_file_write);
            } else {pros::lcd::print(6, "USD WRITE FAILED");}
        } else if (pressed(x, prevX)){
            FILE* usd_file_write = fopen("/usd/replay.txt", "a");
            if (usd_file_write != NULL) {
                fprintf(usd_file_write, "   scraper.set_value(0); \n");
                fclose(usd_file_write);
            } else {pros::lcd::print(6, "USD WRITE FAILED");}
        }



        // END OF ROGUE REPLAY




        //pros::delay to save resources
        pros::delay(10);

		
    }
}



