#ifndef GLOBAL_H_
#define GLOBAL_H_

#define LEFT_FRONT_WHEELS_PORT 17
#define LEFT_CENTER_WHEELS_PORT 11
#define LEFT_BACK_WHEELS_PORT 15
#define RIGHT_FRONT_WHEELS_PORT 18
#define RIGHT_CENTER_WHEELS_PORT 12
#define RIGHT_BACK_WHEELS_PORT 16
#define LIFT_PORT 6
#define AWNING_PORT 7

#define REVERSED true
#define BLUE true
#define RED false
#define PARK true
#define NO_PARK false
#define FRONT true
#define BACK false


extern pros::Motor left_front_wheels;
extern pros::Motor left_center_wheels;
extern pros::Motor left_back_wheels;
extern pros::Motor right_front_wheels;
extern pros::Motor right_center_wheels;
extern pros::Motor right_back_wheels;
extern pros::Motor lift_motor;
extern pros::Motor awning_motor;
extern pros::Controller master;

extern bool colour;
extern bool side;
extern bool parking;
#endif
