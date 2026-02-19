#include "autons.hpp"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
int DRIVE_SPEED = 115;
int TURN_SPEED = 90;
int SWING_SPEED = 110;

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
  chassis.pid_swing_exit_condition_set(90_ms, 4_deg, 300_ms, 6_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(100_ms, 4_in, 300_ms, 6_in, 300_ms, 300_ms);//chassis.pid_drive_exit_condition_set(100_ms, 4_in, 300_ms, 6_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 90);//70 used to be
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
  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  scraper_piston.set_value(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(120);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  score();
  pros::Task::delay(1000);
  intake.move(0);
  scraper_piston.set_value(false);

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(315_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  outtake();
  pros::Task::delay(200);
  middle();
  pros::Task::delay(750);
  intake.move(0);

  chassis.pid_drive_set(38.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(true);
  chassis.pid_drive_set(26.8_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void seven_right()
{
  DRIVE_SPEED=127;
  TURN_SPEED=127;
  chassis.odom_xyt_set(7.25_in, 48_in, 270_deg);//intilizes starting position
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(317_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(21_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);

  chassis.pid_turn_set(27_deg, TURN_SPEED); //tune angle
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(false);
  scraper_piston.set_value(true);

  load();
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true); // going into the loader
  chassis.pid_wait();

  pros::delay(200);
  chassis.pid_wait();


  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true); // going into the loader
  chassis.pid_wait();
  score();
  pros::Task::delay(800);
  chassis.pid_wait();

  score();
  pros::Task::delay(800);
  intake.move(0);
  hook_piston.set_value(true);
  scraper_piston.set_value(false);
  chassis.pid_wait();
  //all this is the hook
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void seven_left()
{
  DRIVE_SPEED=127;
  TURN_SPEED=127;
  chassis.odom_xyt_set(7.25_in, 48_in, 270_deg);//intilizes starting position
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(230_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(21_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);

  chassis.pid_turn_set(129_deg, TURN_SPEED,true); //tune angle
  chassis.pid_wait();
  intake.move(0);

  scraper_piston.set_value(true);
  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(false);
  

  load();
  chassis.pid_drive_set(8_in, DRIVE_SPEED, true); // going into the loader
  chassis.pid_wait();

  pros::delay(200);
  chassis.pid_wait();


  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true); // going into the loader
  chassis.pid_wait();
  score();
  pros::Task::delay(800);
  chassis.pid_wait();

  score();
  scraper_piston.set_value(false);
  pros::Task::delay(800);
  intake.move(0);
  hook_piston.set_value(true);
  chassis.pid_wait();
  //all this is the hook
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-13.9_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void sig_sawp()
{
  DRIVE_SPEED=127;
  TURN_SPEED=127;
  chassis.pid_drive_exit_condition_set(100_ms, 10_in, 300_ms, 14_in, 300_ms, 300_ms);//chassis.pid_drive_exit_condition_set(100_ms, 4_in, 300_ms, 6_in, 500_ms, 500_ms);
  chassis.pid_turn_exit_condition_set(100_ms, 10_deg, 500_ms, 14_deg, 500_ms, 500_ms);
  chassis.odom_xyt_set(23_in, 0_in, 180_deg);
  load();
  /*chassis.pid_drive_set(8_in, DRIVE_SPEED, false); //push bot
  chassis.pid_wait();*/

  chassis.pid_drive_set(-45.5_in, DRIVE_SPEED, false); //go to loader
  chassis.pid_wait();

  scraper_piston.set_value(true);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(14.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();
  pros::Task::delay(360);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  //pros::Task::delay(100);
  score();
  pros::Task::delay(850);
  intake.move(0);
  scraper_piston.set_value(false);

  //chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  /*chassis.pid_turn_set(215_deg, TURN_SPEED);//center 3
  chassis.pid_wait_until_point({24,36});*/
  load();

  chassis.pid_drive_set(12_in,DRIVE_SPEED,false);
  chassis.pid_wait();

  chassis.pid_turn_set(215_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(28_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  /*chassis.pid_drive_set(20_in, DRIVE_SPEED, false); //center 3 get
  chassis.pid_wait_quick();*/

  

  /*chassis.pid_drive_set(41.5_in, 80, false); //going to other center 3
  chassis.pid_wait_quick();*/

  chassis.pid_drive_set(43.5_in, 80, false,false); //going to other center 3
  chassis.pid_wait_quick();
  scraper_piston.set_value(true);

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  intake.move(0);
  //chassis.pid_drive_set(-21.75_in,DRIVE_SPEED,false);
  chassis.pid_drive_set(-16.5_in,DRIVE_SPEED,false); //middle goal distance
  chassis.pid_wait();
  middle();
  pros::delay(1500);
  
  //this is the second loader starting
  
  load();
  //chassis.pid_drive_set(49.5_in, DRIVE_SPEED, false); //back to starting zone
  chassis.pid_drive_set(50_in, DRIVE_SPEED, false); //back to starting zone
  chassis.pid_wait_quick();

  chassis.pid_turn_set(92_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  load();
  chassis.pid_drive_set(14_in, DRIVE_SPEED, false); //park
  chassis.pid_wait_quick();
  pros::delay(250);
  
  chassis.pid_drive_set(-28_in, DRIVE_SPEED, false);
  chassis.pid_wait_quick();
  score();
  pros::delay(2000);
  scraper_piston.set_value(false);
  
}

void four_push_right()
{
  DRIVE_SPEED=127;
  TURN_SPEED=127;
  chassis.odom_xyt_set(7.25_in, 48_in, 270_deg);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(317_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(21_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(100);

  chassis.pid_turn_set(27_deg, TURN_SPEED); //tune angle
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(false);
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // going into the long goal
  chassis.pid_wait();

  score();
  pros::Task::delay(1000);
  intake.move(0);
  hook_piston.set_value(true);
  chassis.pid_wait();
  //all this is the hook
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-14.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in, DRIVE_SPEED, false);
  chassis.pid_wait();
}

void four_push_left()
{
  chassis.odom_xyt_set(-7.25_in, 48_in, 270_deg);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(223_deg, TURN_SPEED);
  chassis.pid_wait();
  load();

  chassis.pid_drive_set(22_in, 60, true);
  chassis.pid_wait();
  pros::Task::delay(100);

  chassis.pid_turn_set(153.3_deg, TURN_SPEED); //tune angle
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  hook_piston.set_value(false);
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // going into the long goal
  chassis.pid_wait();

  score();
  pros::Task::delay(1000);
  intake.move(0);
  hook_piston.set_value(true);
  chassis.pid_wait();
  //all this is the hook
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(35_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-28_in, DRIVE_SPEED, false);
  chassis.pid_wait();
}

void skills()
{
  load();
  hook_piston.set_value(false);
  chassis.odom_xyt_set(47_in, 90_in, 270_deg);
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, false);
  chassis.pid_wait();
  chassis.pid_drive_set(33_in,70,false);
  chassis.pid_wait();

  chassis.pid_drive_set(8_in,127,false,false);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-26_in,DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in,30);
  chassis.pid_wait();

  //start middle goal
  chassis.odom_theta_set(270_deg);
  intake.move(0);

  chassis.pid_drive_set(-21.5_in,DRIVE_SPEED,false);
  chassis.pid_wait();

  chassis.pid_turn_set(225_deg,TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-17_in,DRIVE_SPEED,false);
  chassis.pid_wait();

  chassis.pid_turn_set(315_deg,TURN_SPEED);
  chassis.pid_wait();

  load();
  chassis.pid_drive_set(8_in,DRIVE_SPEED,false);
  chassis.pid_wait();
  intake.move(0);

  chassis.pid_drive_set(-10_in,DRIVE_SPEED,false);
  chassis.pid_wait();
  middle();
  pros::delay(2500);
  intake.move(0);

  load();
  chassis.pid_drive_set(48_in,DRIVE_SPEED,false);
  chassis.pid_wait();
  scraper_piston.set_value(true);

  //first long goal
  load();
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(14.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(1500);
  intake.move(0);

  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(225_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  scraper_piston.set_value(false);
  chassis.pid_drive_set(-58_in, DRIVE_SPEED, true);//-55 in
  chassis.pid_wait();
  
  
  chassis.pid_turn_set(315_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  //ending first half long goal
  score();
  pros::Task::delay(2000);
  intake.move(0);

  scraper_piston.set_value(true);
  load();
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(1000);

  chassis.pid_drive_set(-28_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  score();
  pros::Task::delay(2000);
  intake.move(0);

  //ends here for first long goal

  //second long goal
  chassis.pid_drive_set(12_in,DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(87_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  scraper_piston.set_value(true);
  load();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(1000);

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  scraper_piston.set_value(false);

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-19_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  scraper_piston.set_value(false);

  chassis.pid_drive_set(-60_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  score();
  pros::Task::delay(2000);
  intake.move(0);

  scraper_piston.set_value(true);
  load();
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::Task::delay(1000);

  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  score();
  pros::Task::delay(2000);
  intake.move(0);
  //ends here for second long goal
  
  //code for the park
  scraper_piston.set_value(false);
  chassis.pid_drive_set(12_in,DRIVE_SPEED,true);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(42_in,DRIVE_SPEED,true);
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg,TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(33_in,70,false);
  chassis.pid_wait();



}