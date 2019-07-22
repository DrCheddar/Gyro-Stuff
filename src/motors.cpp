#include "main.h"
#include "global.h"

/**
 * All motors defined here
 */
pros::Motor left_front_wheels (LEFT_FRONT_WHEELS_PORT, MOTOR_GEARSET_18);
pros::Motor left_center_wheels (LEFT_CENTER_WHEELS_PORT, MOTOR_GEARSET_18);
pros::Motor left_back_wheels (LEFT_BACK_WHEELS_PORT, MOTOR_GEARSET_18);
pros::Motor right_front_wheels (RIGHT_FRONT_WHEELS_PORT, MOTOR_GEARSET_18, REVERSED);
pros::Motor right_center_wheels (RIGHT_CENTER_WHEELS_PORT, MOTOR_GEARSET_18, REVERSED);
pros::Motor right_back_wheels (RIGHT_BACK_WHEELS_PORT, MOTOR_GEARSET_18, REVERSED);
pros::Motor lift_motor (LIFT_PORT, MOTOR_GEARSET_18);
pros::Motor awning_motor (AWNING_PORT, MOTOR_GEARSET_36);
pros::Controller master (CONTROLLER_MASTER);
