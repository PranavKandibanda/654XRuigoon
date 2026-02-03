#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;
extern pros::adi::Pneumatics hook_piston;
extern pros::adi::Pneumatics scraper_piston;

extern pros::adi::Pneumatics loading_piston;
extern pros::adi::Pneumatics scoring_piston;
extern pros::MotorGroup intake;

extern void outtake();
extern void score();
extern void load();
extern void middle();
extern void score_setup();
// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');