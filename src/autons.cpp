#include "autons.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 115;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(14, 0.01, 240);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.1, 0.18, 50, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 4_deg, 500_ms, 6_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 4_in, 300_ms, 6_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

void four_3_left()
{
  chassis.odom_xyt_set(0_in, 24_in, 180_deg);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  scraper_piston.set_value(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  score();
  pros::Task::delay(800);
  intake.move(0);
  scraper_piston.set_value(false);

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(308_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(145_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-27.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  middle();
  pros::Task::delay(750);
  intake.move(0);

  chassis.pid_drive_set(33.65_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(true);
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}



void seven_right()
{
  chassis.odom_xyt_set(0_in, 24_in, 0_deg);
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  scraper_piston.set_value(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(130);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);
  score();
  pros::Task::delay(800);
  intake.move(0);
  scraper_piston.set_value(false);

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(228_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(34_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  
  score();
  pros::Task::delay(800);
  intake.move(0);

  hook_piston.set_value(false);
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  hook_piston.set_value(false);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void sig_sawp()
{
  chassis.odom_xyt_set(23_in, 0_in, 180_deg);
  load();
  chassis.pid_drive_set(8_in, DRIVE_SPEED, false); //push bot
  chassis.pid_wait();

  chassis.pid_drive_set(-50_in, DRIVE_SPEED, false); //go to loader
  chassis.pid_wait();

  scraper_piston.set_value(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(150);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);
  score();
  pros::Task::delay(850);
  intake.move(0);
  scraper_piston.set_value(false);

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(185_deg, TURN_SPEED);//center 3
  chassis.pid_wait();

  load();
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true); //center 3 get
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(47_in, 80, true); //going to other center 3
  chassis.pid_wait();

  chassis.pid_turn_set(140_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-26_in,DRIVE_SPEED,true);
  chassis.pid_wait();

  middle();
  pros::Task::delay(800);

  //this is the second loader starting
  load();
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true); //back to starting zone
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  scraper_piston.set_value(true);
  chassis.pid_wait();

  chassis.pid_drive_set(16_in, DRIVE_SPEED, true); //park
  chassis.pid_wait();

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  score();
  pros::Task::delay(800);










}