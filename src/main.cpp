#include "main.h"
#include "autons.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
pros::MotorGroup intake({3,-7}, pros::MotorGearset::blue);
pros::adi::Pneumatics hook_piston('G',false);
pros::adi::Pneumatics scraper_piston('H',false);

pros::adi::Pneumatics loading_piston('B',false);
pros::adi::Pneumatics scoring_piston('D',false);

constexpr double FIELD_SIZE_MM  = 3657.6/25.4;
// Positive values = outward from center
// Measured from tracking center to sensor (mm)
// +X = east, +Y = north
constexpr double NORTH_Y_OFFSET_MM = 228.6/25.4;
constexpr double SOUTH_Y_OFFSET_MM = 101.6/25.4;
constexpr double EAST_X_OFFSET_MM  = 152.4/25.4;
constexpr double WEST_X_OFFSET_MM  = 152.4/25.4;

pros::Distance east(16);
pros::Distance north(2);
pros::Distance west(1);
pros::Distance south(11);


void score()
{
    scoring_piston.set_value(true);//true
    loading_piston.set_value(false);//false
    intake.move(127);
}

void outtake()
{
    scoring_piston.set_value(true);//true
    loading_piston.set_value(false);//false
    intake.move(-127);
}

void load()
{
    scoring_piston.set_value(true);//true
    loading_piston.set_value(true);//true
    intake.move(127);
}

void middle()
{
    scoring_piston.set_value(false);//false
    loading_piston.set_value(true);//true
    intake.move(127);
}

void middle_skills()
{
    scoring_piston.set_value(false);//false
    loading_piston.set_value(true);//true
    intake.move(80);
}

void score_setup()
{
  scoring_piston.set_value(true);
  loading_piston.set_value(false);
}

double normalizeAngle(double deg) {
  while (deg < 0) deg += 360;
  while (deg >= 360) deg -= 360;
  return deg;
}

void distanceResetPose() {
  double thetaDeg = chassis.odom_theta_get();
  thetaDeg = normalizeAngle(thetaDeg);

  double x = chassis.odom_x_get();
  double y = chassis.odom_y_get();

  // Facing EAST (wall at +X)
  if (thetaDeg >= 315 || thetaDeg < 45) {
    double d = east.get_distance()/25.4;
    x = FIELD_SIZE_MM - d - EAST_X_OFFSET_MM;
  }
  // Facing NORTH (wall at +Y)
  else if (thetaDeg >= 45 && thetaDeg < 135) {
    if (chassis.odom_y_get() > 0)
    {
        double d = west.get_distance()/25.4;
        y = FIELD_SIZE_MM/2 - d - WEST_X_OFFSET_MM;
    }
    else {
        double d = east.get_distance()/25.4;
        y = (FIELD_SIZE_MM/2 - d - EAST_X_OFFSET_MM)*-1;
    }
  }
  // Facing WEST (wall at 0)
  else if (thetaDeg >= 135 && thetaDeg < 225) {
    double d = west.get_distance()/25.4;
    double d2 = north.get_distance()/25.4;
    x = FIELD_SIZE_MM/2 - d - WEST_X_OFFSET_MM;
    y = -(FIELD_SIZE_MM/2 - d2 - NORTH_Y_OFFSET_MM);
    //pros::lcd::print(5, "West distance: %f", y);
  }
  // Facing SOUTH (wall at 0)
  else {
    double d = south.get_distance()/25.4;
    y = 0.0 - d - SOUTH_Y_OFFSET_MM;
  }
  chassis.odom_xyt_set(x,y, thetaDeg);
}
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-6, -5, -4},     // Left Chassis Ports (negative port will reverse it!)
    {8, 9, 10},  // Right Chassis Ports (negative port will reverse it!)

    21,      // IMU Port
    3.24,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    //{"skills",skills},
    //{"four + three left",four_3_left},
    //{"seven left",seven_left},
     {"Sig Sawp",sig_sawp}
     //{"Four + Three left push",four_3_left},
     //{"Seven push right",seven_right},
     //{"Four push right",four_push_right},
     //{"Four push left",four_push_left}
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
      // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
        scraper_piston.toggle();
        pros::delay(200);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        hook_piston.retract();
    }
    else{
        hook_piston.extend();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        score();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        outtake();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        load();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        middle();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    {
        middle_skills();
    }
    else {
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        intake.brake();
        intake.brake();
    }
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
