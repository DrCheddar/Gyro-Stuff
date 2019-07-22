#include "main.h"
#include "global.h"
#include "autonomous.h"
#include "cmath"

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

// Abstractions

using namespace std;

void CustomDrive(int ticks, int L_speed, int R_speed) {
	left_front_wheels.move_relative(ticks, L_speed);
	left_center_wheels.move_relative(ticks, L_speed);
	left_back_wheels.move_relative(ticks, L_speed);

	right_front_wheels.move_relative(ticks, R_speed);
	right_center_wheels.move_relative(ticks, R_speed);
	right_back_wheels.move_relative(ticks, R_speed);
}

void move_left(int L_speed) {
	left_front_wheels.move_velocity(L_speed);
	left_center_wheels.move_velocity(L_speed);
	left_back_wheels.move_velocity(L_speed);
}

void move_right(int R_speed) {
	right_front_wheels.move_velocity(R_speed);
	right_center_wheels.move_velocity(R_speed);
	right_back_wheels.move_velocity(R_speed);
}

void TimedDrive(int millis, int L_speed, int R_speed) {
	left_front_wheels.move_velocity(L_speed);
	left_center_wheels.move_velocity(L_speed);
	left_back_wheels.move_velocity(L_speed);

	right_front_wheels.move_velocity(R_speed);
	right_center_wheels.move_velocity(R_speed);
	right_back_wheels.move_velocity(R_speed);

	pros::delay(millis);

	left_front_wheels.move_velocity(0);
	left_center_wheels.move_velocity(0);
	left_back_wheels.move_velocity(0);

	right_front_wheels.move_velocity(0);
	right_center_wheels.move_velocity(0);
	right_back_wheels.move_velocity(0);

	pros::delay(200);

}
int get_L_pos() {
	return (left_back_wheels.get_position() + left_center_wheels.get_position() + left_front_wheels.get_position()) / 3;
}

int get_R_pos() {
	return (right_back_wheels.get_position() + right_center_wheels.get_position() + right_front_wheels.get_position()) / 3;
}

void DistanceDrive(int L_Dist, int R_Dist, int L_speed, int R_speed) { // speed is always positive, dist can be neg or pos; travels for a number of ticks
	int R_Start = get_R_pos();
	int L_Start = get_L_pos();

	int R_diff = (get_R_pos() - R_Start);
	int L_diff = (get_L_pos() - L_Start);

	int constant = 0.003; //at speed when 300 ticks away, slower when below 300 ticks away, faster when above 300 ticks

	while (abs(R_diff - R_Dist) > 30 || abs(L_diff - L_Dist) > 30) {
		if (abs(R_diff - R_Dist) > 30) {
			int speed = R_speed * (R_diff - R_Dist) * constant; //speed * direction and progress * constant

			if (speed > R_speed) //caps max speed at set speed
				speed = R_speed;

			move_right(speed); 
		}

		if (abs(L_diff - L_Dist) > 30) {
			int speed = L_speed * (L_diff - L_Dist) * constant; 

			if (speed > L_speed)
				speed = L_speed;

			move_right(speed);
		}

		R_diff = (get_R_pos() - R_Start);
		L_diff = (get_L_pos() - L_Start);

		pros::delay(10);
	}

	//stops drive motors
	move_right(0);
	move_left(0);
}

void FlipCap() {
	lift_motor.move_velocity(200);
	pros::delay(1000);
	lift_motor.move_velocity(-200);
	pros::delay(1000);
	lift_motor.move_velocity(0);
	pros::delay(500);
}

// Various Autonomous Programs
void BlueFrontAuton (bool parking) {
    if (parking == PARK) {

    }
}

void RedBackAuton (bool parking) {
	lift_motor.move_velocity(-150);
	pros::delay(150);
	lift_motor.move_velocity(0);

	TimedDrive(1000, 200, 200); //drive up to cap

	TimedDrive(250, -75, -75);

	TimedDrive(600, -75, 75);

	lift_motor.move_velocity(-150);
	pros::delay(200);
	lift_motor.move_velocity(0);

	TimedDrive(1500, 200, 200);

	/*
	lift_motor.move_velocity(150);
	pros::delay(400);
	lift_motor.move_velocity(-150);
	pros::delay(400);
	lift_motor.move_velocity(0);
	pros::delay(200);
	
	TimedDrive(500, -75, -75); //back off
	TimedDrive(450, 75, -75); //turn and face second cap
	TimedDrive(1000, 75, 75); //drive upnto second cap
	FlipCap();
	*/


	if (parking == PARK) {
		TimedDrive(450, 75, -75);//turn parrallel to wall
		TimedDrive(350, -75, -75); // drive back
		TimedDrive(700, 75, -75); // turn	
		TimedDrive(700, -150, -150); //align against wall
		lift_motor.move_velocity(100);
		pros::delay(200);
		lift_motor.move_velocity(0);
		pros::delay(200);

		awning_motor.move_velocity(-200);
		TimedDrive(1800, -200, -200); //onto platform
	}
}

void RedFrontAuton (bool parking) {
	TimedDrive(2000, -100, -100);
	
	
	/*
	TimedDrive(2300, -75, -75);
	TimedDrive(700, -75, 75); // turn
	TimedDrive(500, 150, 150);
	TimedDrive(3000, -75, -75); //drive up to cap

	lift_motor.move_velocity(150);
	pros::delay(100);
	lift_motor.move_velocity(-150);
	pros::delay(100);
	lift_motor.move_velocity(0);
	pros::delay(200);
	*/
    if (parking == PARK) {

    }
}

void BlueBackAuton (bool parking) {
	lift_motor.move_velocity(-150);
	pros::delay(150);
	lift_motor.move_velocity(0);

	TimedDrive(1000, 200, 200); //drive up to cap

	TimedDrive(250, -75, -75);

	TimedDrive(600, 75, -75);

	lift_motor.move_velocity(-150);
	pros::delay(200);
	lift_motor.move_velocity(0);

	TimedDrive(1500, 200, 200);
	
	TimedDrive(2850, 75, 75); //drive up to cap

	lift_motor.move_velocity(150);
	pros::delay(100);
	lift_motor.move_velocity(-150);
	pros::delay(100);
	lift_motor.move_velocity(0);
	pros::delay(200);

	TimedDrive(500, 75, 75); //back off
	TimedDrive(450, -75, 75); //turn and face second cap
	TimedDrive(1000, -75, -75); //drive upnto second cap
	FlipCap();
	TimedDrive(450, 75, -75);//turn parrallel to wall
	TimedDrive(280, 75, 75); // drive back
	TimedDrive(700, 75, -75); // turn
	TimedDrive(700, 150, 150); //align against wall
	lift_motor.move_velocity(100);
	pros::delay(200);
	lift_motor.move_velocity(0);
	pros::delay(200);

	if (parking == PARK) {
		awning_motor.move_velocity(-200);
		TimedDrive(1800, -200, -200); //onto platform
	}

}

void AutonPicker (bool colour, bool side, bool parking) {

   if (colour == BLUE and side == FRONT) {
        BlueFrontAuton(parking);
   } else if (colour == BLUE and side == BACK) {
        BlueBackAuton(parking);
   } else if (colour == RED and side == FRONT) {
        RedFrontAuton(parking);
   } else {
        RedBackAuton(parking);
   }
}

void Skills() {
	TimedDrive(3000, -75, -75); //drive up to cap
	FlipCap();
	TimedDrive(500, 75, 75); //back off
	TimedDrive(450, 75, -75); //turn and face second cap
	TimedDrive(1000, -75, -75); //drive upnto second cap
	FlipCap();
	TimedDrive(450, -75, 75);//turn parrallel to wall
	TimedDrive(4000, 75, 75); // drive back
}


void autonomous() {
    AutonPicker(RED,BACK,NO_PARK);
	// Skills();
}
