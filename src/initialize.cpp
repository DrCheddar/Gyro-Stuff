#include "main.h"
#include "global.h"
#include "constants.h"
#include "pid.h"


bool colour = true;
bool side = true;
bool parking = true;

void on_left_button () {
    if (colour == true) {
        colour = false;
    } else {
        colour = true;
    }
    pros::lcd::set_text(3, "blue?: " + std::to_string(colour));
}

void on_center_button () {
    if (side == true) {
        side = false;
    } else {
        side = true;
    }
    pros::lcd::set_text(4, "front?: " + std::to_string(side));
}

void on_right_button () {
    if (parking == true) {
        parking = false;
    } else {
        parking = true;
    }
    pros::lcd::set_text(5, "park?: " + std::to_string(parking));
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 *
 * Initializes the display and lets the user toggle the AutonPicker values
 * Also displays current motor positions and velocities
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(0, "Ports: " + 
                           std::to_string(LEFT_FRONT_WHEELS_PORT) + "," +
                           std::to_string(LEFT_CENTER_WHEELS_PORT) + "," +
                           std::to_string(LEFT_BACK_WHEELS_PORT) + "," +
                           std::to_string(RIGHT_FRONT_WHEELS_PORT) + "," +
                           std::to_string(RIGHT_CENTER_WHEELS_PORT) + "," +
                           std::to_string(RIGHT_BACK_WHEELS_PORT) + "," +
                           std::to_string(LIFT_PORT) + "," +
                           std::to_string(AWNING_PORT));
    pros::lcd::set_text(3, "blue?: " + std::to_string(colour));
    pros::lcd::set_text(4, "front?: " + std::to_string(side));
    pros::lcd::set_text(5, "park?: " + std::to_string(parking));
	awning_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	awning_motor.tare_position();
    lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift_motor.tare_position();
    pros::lcd::register_btn0_cb(on_left_button);
    pros::lcd::register_btn1_cb(on_center_button);
    pros::lcd::register_btn2_cb(on_right_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
}
