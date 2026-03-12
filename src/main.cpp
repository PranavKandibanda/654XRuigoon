#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/draw/lv_draw_rect.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/rotation.hpp"
#include "pros/imu.hpp"
#include "pros/distance.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-2, -13, -4},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, 9, 8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::adi::Pneumatics loading_piston('A',false);
pros::adi::Pneumatics scoring_piston('B',false);
pros::adi::Pneumatics scraper_piston('D',false);
pros::adi::Pneumatics hook_piston('E',false);

pros::MotorGroup intake ({1,-5}, pros::MotorGearset::blue);

void load()
{
    loading_piston.set_value(true);
    scoring_piston.set_value(true);
    intake.move(127);
}

void middle_score()
{
    loading_piston.set_value(false);
    scoring_piston.set_value(true);
    intake.move(127);
}

void score()
{
    loading_piston.set_value(true);
    scoring_piston.set_value(false);
    intake.move(127);
}

void skills_score()
{
    loading_piston.set_value(true);
    scoring_piston.set_value(false);
    intake.move(117);
}

void reverse()
{
    loading_piston.set_value(true);
    scoring_piston.set_value(false);
    intake.move(-127);
}

void reverse_skills()
{
    loading_piston.set_value(true);
    scoring_piston.set_value(false);
    intake.move(-77);
}


// Inertial Sensor on port 10
pros::Imu imu(12);

double front_sensor_offset = 8;
double left_sensor_offset = 5.5;
double right_sensor_offset = 5.5;
double back_sensor_offset = 6;
double field_half_size = 72; // Half of the field size in inches

pros::Distance front_sensor(6);
pros::Distance right_sensor(20);
pros::Distance left_sensor(17);
pros::Distance back_sensor(11);



// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            9, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            15 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14.5, // derivative gjain (kD)
                                             3, // anti windup
                                             4, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             6, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking hwheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal trackbhhing wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void resetPositionWithSensor(pros::Distance& sensor, double sensor_offset, double sensor_angle_offset, double field_half_size) {
    double sensorReading = sensor.get_distance()*0.0393701; //mm to inches
  
    // Check for invalid reading (distance sensors return -1 or very large values when no object detected)
    if (sensorReading < 0 || sensorReading > 200) {
        pros::lcd::print(0,"invalid sensor reading"); // x
        return;
    }
    
    // Get current pose
    double current_heading_deg = chassis.getPose().theta;
    double robot_heading_deg = current_heading_deg + sensor_angle_offset;
    
    // Normalize heading to 0-360 range
    int headingDeg = (int)(robot_heading_deg);
    headingDeg = (headingDeg + 360) % 360;
    
    // Determine which wall we're facing and which axis to reset
    bool resettingX = false;
    double wallSign = 1.0;
    
    if (315 <= headingDeg || headingDeg <= 45) {
        // Top wall - reset Y position
        resettingX = false;
        wallSign = 1.0;
    }
    else if (45 < headingDeg && headingDeg <= 135) {
        // Right wall - reset X position
        resettingX = true;
        wallSign = 1.0;
    }
    else if (135 < headingDeg && headingDeg <= 225) {
        // Bottom wall - reset Y position
        resettingX = false;
        wallSign = -1.0;
    }
    else {
        // Left wall - reset X position
        resettingX = true;
        wallSign = -1.0;
    }
    
    // Calculate distance from wall to robot center
    double wallToCenter = sensorReading + sensor_offset;
    
    // Calculate actual position
    double actualPos = wallSign * (field_half_size - wallToCenter);
    
    // Update position (only reset the appropriate axis)
    if (resettingX) {
        chassis.setPose(actualPos, chassis.getPose().y, chassis.getPose().theta);

    } else {
        chassis.setPose(chassis.getPose().x, actualPos, chassis.getPose().theta);
    }
}

/*
 * resetPositionFront
 * Resets position using the front distance sensor.
 * Remember to only use these when perpendicular to the wall!
 * - sensor: Front distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionFront() {
    resetPositionWithSensor(front_sensor, front_sensor_offset, 0.0, field_half_size);
}

/*
 * resetPositionBack
 * Resets position using the back distance sensor.
 * Remember to only use these when perpendicular to the wall!
 * - sensor: Back distance sensor 
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionBack() {
    resetPositionWithSensor(back_sensor, back_sensor_offset, 180.0, field_half_size);
}

/*
 * resetPositionLeft
 * Resets position using the left distance sensor.
 * Remember to only use these when perpendicular to the wall!
 * - sensor: Left distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionLeft() {
    resetPositionWithSensor(left_sensor, left_sensor_offset, 270.0, field_half_size);
}

/*
 * resetPositionRight
 * Resets position using the right distance sensor.
 * Remember to only use these when perpendicular to the wall!
 * - sensor: Right distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionRight() {
    resetPositionWithSensor(right_sensor, right_sensor_offset, 90.0, field_half_size);
}


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
            // delay to save resources
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

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void skills() {
    chassis.setPose(-44.457,7,90);
    resetPositionBack();
    hook_piston.set_value(false);

    //getting four balls in the center
    load();
    chassis.swingToHeading(45, DriveSide::LEFT, 500,{},false);
    chassis.moveToPoint(-24, 24, 1000,{.maxSpeed=87}, true);
    chassis.waitUntil(20);
    scraper_piston.set_value(true);
    chassis.waitUntilDone();
    //getting four balls in the center

    
    //scoring middle
    chassis.turnToHeading(315, 2000,{}, false);
    chassis.moveToPoint(-14.5,13.5, 2000,{.forwards=false}, false);
    middle_score(); 
    pros::delay(750);
    load();
    //scoring middle

    //going to long and scoring excess
    chassis.moveToPoint(-48, 48, 1000,{.maxSpeed = 97}, false);
    chassis.turnToHeading(266, 1000,{}, false);
    resetPositionRight();
    resetPositionFront();
    
    chassis.moveToPoint(-30,48,500,{.forwards=false,.maxSpeed = 87},false);
    //scraper_piston.set_value(true);
    score();
    pros::delay(750);
    load();
    //going to long and scoring excess

    //going to 1st loader
    chassis.moveToPoint(-50, chassis.getPose().y, 2000,{},false);
    chassis.moveToPoint(-60.1, chassis.getPose().y, 2000,{.maxSpeed = 57},false);
    chassis.tank(97,97);
    pros::delay(1750);
    chassis.cancelAllMotions();
    
    //going to 1st loader


    //exiting loader
    chassis.moveToPoint(-48, chassis.getPose().y, 2000,{.forwards=false},true);
    chassis.waitUntil(5);
    intake.move(0);
    chassis.waitUntilDone();
    resetPositionRight();
    resetPositionFront();
    //exiting loader

    //going to the side of first long goal
    chassis.moveToPose(-36, 59,46+180,2000,{.forwards=false},false);
    chassis.turnToHeading(270, 1000,{},false);
    scraper_piston.set_value(false);
    resetPositionRight();
    resetPositionFront();
    //going to the side of first long goal

    //traversing the side of the first long goal
    chassis.moveToPoint(36, chassis.getPose().y, 2000,{.forwards=false,.maxSpeed = 87},false);
    resetPositionRight();
    resetPositionFront();
    //traversing the side of the first long goal

    //aligning with the end of the first long goal
    chassis.moveToPose(36, 48, 311, 2000,{.forwards=false,.lead = .4,.maxSpeed = 87},false);
    chassis.turnToHeading(90, 2000,{},false);
    //aligning with the end of the first long goal

    //going to score on the first long goal
    chassis.moveToPoint(26.75,48, 1000,{.forwards = false},false);
    scraper_piston.set_value(true);
    skills_score();
    pros::delay(2000);
    resetPositionFront();
    resetPositionLeft();
    load();
    //going to score on the first long goal*/

    //chassis.setPose(30,49.37,90);
    //pros::lcd::print(4,"Chassis pose: %2f %2f", chassis.getPose().x, chassis.getPose().y);

    
    //going to second loader
    chassis.moveToPoint(49.75, chassis.getPose().y, 2000,{},false);
    chassis.moveToPoint(60, chassis.getPose().y, 2000,{.maxSpeed = 73},false);
    chassis.tank(97,97);
    pros::delay(1500);
    chassis.tank(0, 0);
    intake.move(0);
    resetPositionLeft();
    resetPositionFront();
    //going to second loader

    //scoring all of those blocks
    chassis.moveToPoint(31, chassis.getPose().y-1, 2000,{.forwards= false,.maxSpeed = 100},false);
    scraper_piston.set_value(true);
    skills_score();
    pros::delay(1500);
    resetPositionFront();
    resetPositionLeft();
    chassis.setPose(chassis.getPose().x,chassis.getPose().y,90);
    scraper_piston.set_value(false);
    //scoring all of those blocks

    //scoring lower 4
    load();
    chassis.moveToPoint(48, 48, 1000,{.maxSpeed = 67},false);
    chassis.turnToHeading(223, 1000,{},false);
    //chassis.moveToPoint(26,24,1000,{.maxSpeed = 67},false);
    //chassis.turnToHeading(223, 1000,{},false);
    chassis.moveToPoint(19,18,1000,{.maxSpeed=77},false);
    //pros::lcd::print(4,"Brett is code daddy x2");
    reverse_skills();
    pros::delay(2000);
    //scoring lower 4

    //clearing second parking zone
    load();
    chassis.moveToPoint(24,24,1000,{.forwards = false},false);
    chassis.turnToHeading(50,1000,{},false);
    chassis.moveToPose(72, 24,180, 1000,{.lead = .4},false);
    chassis.turnToHeading(180, 1000,{},false);

    chassis.tank(87, 87);
    pros::delay(500);
    scraper_piston.set_value(true);
    pros::delay(300);
    scraper_piston.set_value(false);
    chassis.tank(97, 97);
    pros::delay(2500);
    chassis.cancelAllMotions();

    chassis.setPose(66, -36, 180);
    resetPositionLeft();
    chassis.moveToPoint(66, -36, 1000,{},false);
    chassis.turnToHeading(90,1000,{},false);
    resetPositionRight();
    //clearing second parking zone

    
    //scoring seven on upper center goal
    chassis.moveToPose(48,-24,135,2000,{.forwards=false},false);
    //chassis.turnToHeading(45+90, 1000,{},false);
    middle_score();
    pros::delay(2000);
    load();
    //scoring seven on upper center goal

    //going to long and scoring excess
    /*chassis.moveToPoint(48, -48, 2000,{.maxSpeed = 97},false);
    pros::delay(500);
    chassis.turnToHeading(90, 1000,{},false);
    pros::delay(500);
    resetPositionFront();
    resetPositionRight();
    chassis.moveToPoint(30,-48,500,{.forwards=false},false);
    score();
    pros::delay(500);
    //going to long and scoring excess

    //going to 3rd loader
    scraper_piston.set_value(true);
    chassis.moveToPoint(60.25, -48, 2000,{.maxSpeed = 67},false);
    pros::delay(1500);
    resetPositionFront();
    resetPositionRight();
    //going to 3rd loader

    //exiting loader
    chassis.moveToPoint(48, chassis.getPose().y, 2000,{.forwards=false},false);
    resetPositionRight();
    resetPositionFront();
    //exiting loader

    //going to the side of second long goal
    chassis.moveToPose(36, -58.5,140,2000,{},false);
    chassis.turnToHeading(90, 1000,{},false);
    scraper_piston.set_value(false);
    resetPositionRight();
    resetPositionFront();
    //going to the side of second long goal

    //traversing the side of the second long goal
    chassis.moveToPoint(-36, chassis.getPose().y, 2000,{.forwards=false,.maxSpeed = 87},false);
    resetPositionRight();
    resetPositionBack();
    //traversing the side of the second long goal

    //aligning with the end of the second long goal
    chassis.moveToPose(-36, -48, 45, 2000,{.lead = .4,.maxSpeed = 87},false);
    chassis.turnToHeading(270, 2000,{},false);
    //aligning with the end of the second long goal

    //going to score on the second long goal
    chassis.moveToPoint(-28.5,-48, 1000,{.forwards = false},false);
    score();
    pros::delay(1000);
    resetPositionFront();
    resetPositionLeft();
    load();
    //going to score on the second long goal

    //going to 4th loader
    scraper_piston.set_value(true);
    chassis.moveToPoint(-60.25, chassis.getPose().y, 2000,{.maxSpeed = 87},false);
    pros::delay(500);
    resetPositionFront();
    resetPositionLeft();
    //going to 4th loader

    //scoring all of those blocks
    chassis.moveToPoint(-29.5, -48, 2000,{.forwards= false},false);
    score();
    pros::delay(1000);
    resetPositionFront();
    resetPositionLeft();
    scraper_piston.set_value(false);
    //scoring all of those blocks

    //parking at the end
    load();
    chassis.moveToPose(-60,-24,0,2000,{},false);
    resetPositionBack();
    resetPositionLeft();
    chassis.turnToHeading(325, 1000,{},false);
    chassis.moveToPoint(-62.5, -22, 2000,{},false);
    chassis.swingToHeading(0, DriveSide::RIGHT, 1000);
    load();
    chassis.tank(97,97);
    pros::delay(1000);
    //end skills
    */
    
}

void left_center_4_3()
{
    chassis.setPose(47.563,-15,0);
    resetPositionRight();
    resetPositionBack();
    load();

    chassis.moveToPoint(chassis.getPose().x, -48, 1000,{.forwards = false},false);
    chassis.turnToHeading(90, 1000);
    resetPositionFront();
    resetPositionRight();
    scraper_piston.set_value(true);

    chassis.moveToPoint(62, chassis.getPose().y, 1000,{},false);
    pros::delay(800);
    resetPositionFront();
    resetPositionRight();

    chassis.moveToPoint(32.034+.1, -48, 1000,{.forwards=false},true);
    chassis.waitUntil(20);
    score();
    scraper_piston.set_value(false);
    chassis.waitUntilDone();
    pros::delay(1000);

    resetPositionFront();
    resetPositionRight();

    load();
    chassis.moveToPoint(48,-48, 1000,{},false);
    chassis.turnToHeading(314, 1000,{},false);

    chassis.moveToPoint(22.545, -22.689, 1000,{},false);
    chassis.turnToHeading(217.5, 2000,{},false);
    chassis.moveToPoint(7.879, -42.818, 1000,{},false);
    chassis.waitUntil(19);
    scraper_piston.set_value(true);
    chassis.waitUntilDone() ;

    chassis.moveToPoint(24, -24, 1000,{.forwards = false},true);
    chassis.turnToHeading(140, 1000,{},false);
    chassis.moveToPoint(12.343,-12.062,1000,{.forwards=false},false);
    middle_score();
    pros::delay(1500);

    scraper_piston.set_value(false);
    chassis.moveToPoint(36.635,-34.916, 1000,{},false);
    chassis.turnToHeading(270, 1000,{},false);
    resetPositionLeft();
    resetPositionBack();

    hook_piston.set_value(false);
    chassis.moveToPoint(11.905, -35.916, 1000,{},false);
}

void right_center_4_3()
{
    chassis.setPose(47.563,15,0);
    resetPositionLeft();
    resetPositionBack();
    load();

    chassis.moveToPoint(chassis.getPose().x, 48, 1000,{.forwards = false},false);
    chassis.turnToHeading(90, 1000);
    resetPositionFront();
    resetPositionRight();
    scraper_piston.set_value(true);

    chassis.moveToPoint(62, chassis.getPose().y, 1000,{},false);
    pros::delay(800);
    resetPositionFront();
    resetPositionRight();

    chassis.moveToPoint(32.034, 48, 1000,{.forwards=false},true);
    chassis.waitUntil(20);
    score();
    scraper_piston.set_value(false);
    chassis.waitUntilDone();
    pros::delay(1000);

    resetPositionFront();
    resetPositionRight();

    load();
    chassis.moveToPoint(48,48, 1000,{},false);
    chassis.turnToHeading(225, 1000,{},false);

    chassis.moveToPoint(22.545, 22.689, 1000,{},false);
    chassis.turnToHeading(142.595, 2000,{},false);
    chassis.moveToPoint(7.879, 42.818, 1000,{},false);
    chassis.waitUntil(19);
    scraper_piston.set_value(true);
    chassis.waitUntilDone();

    chassis.moveToPoint(24, -24, 1000,{.forwards = false},false);
    chassis.turnToHeading(46.824, 1000,{},false);
    chassis.moveToPoint(13.343,14.062,1000,{.forwards=false},false);
    middle_score();
    pros::delay(1500);

    chassis.moveToPoint(36.635,35.916, 1000,{},false);
    chassis.turnToHeading(270, 1000,{},false);
    resetPositionLeft();
    resetPositionBack();

    hook_piston.set_value(false);
    chassis.moveToPoint(11.905, 35.916, 1000,{},false);
}

void sig_sawp()
{
    chassis.setPose(47.563,0,180);

    resetPositionLeft();
    resetPositionBack();

    load();
    chassis.moveToPoint(chassis.getPose().x, chassis.getPose().y - 5, 500,{.minSpeed = 127},false);
    resetPositionLeft();
    resetPositionBack();
    pros::delay(100);

    chassis.moveToPoint(48, 41.5, 1000,{.forwards=false,.maxSpeed = 87},false);
    scraper_piston.set_value(true);

    chassis.turnToHeading(90, 500,{},false);
    resetPositionFront();
    resetPositionLeft();

    chassis.moveToPoint(65.25, 47, 1000,{},false);
    pros::delay(350);
    resetPositionFront();
    resetPositionLeft();
    scraper_piston.set_value(false);

    chassis.moveToPoint(32.5, 47.35, 1000,{.forwards = false},false);
    score();
    pros::delay(1000);
    resetPositionFront();
    resetPositionLeft();

    /*load();
    chassis.moveToPoint(48, 48, 1000,{},false);
    chassis.turnToHeading(90, 500,{},false);
    resetPositionFront();
    resetPositionLeft();
    chassis.turnToHeading(215, 500,{},false);

    chassis.moveToPoint(26, 26, 1000,{},false);

    chassis.turnToHeading(180, 500,{},false);

    chassis.moveToPoint(24,-24,850,{},false);
    chassis.setPose(24,-24,180);
    //resetPositionLeft();
    chassis.turnToHeading(134.118, 500,{},false);

    pros::lcd::print(4,"Chassis pose: %2f %2f", chassis.getPose().x, chassis.getPose().y);
    chassis.moveToPoint(16.343,-18.062,1000,{.forwards = false,.maxSpeed = 87},false);
    middle_score();
    pros::delay(750);
    scraper_piston.set_value(true);

    load();
    chassis.moveToPoint(48, -49, 1000,{},false);
    chassis.turnToHeading(90, 350,{},false);
    resetPositionFront();
    resetPositionRight();

    chassis.moveToPoint(61.5,chassis.getPose().y, 1000,{},false);
    pros::delay(350);
    resetPositionFront();
    resetPositionRight();

    chassis.moveToPoint(36.034, chassis.getPose().y, 1000,{.forwards=false,.minSpeed = 67},false);
    score();
    pros::delay(2000);*/
    
}

void autonomous()
{
    //uncomment the one you want to run
    skills();
    //left_center_4_3();
    //right_center_4_3();
    //sig_sawp();
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
	skills();
    //left_center_4_3();
    //sig_sawp();
    //autonomous();
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            score();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            reverse();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            load();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            middle_score();
        }
        else
        {
            intake.move(0);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            scraper_piston.toggle();
            pros::delay(250); // debounce delay
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            hook_piston.toggle();
            pros::delay(250); // debounce delay
        }
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}