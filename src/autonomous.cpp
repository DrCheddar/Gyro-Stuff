#include "main.h"
#include "global.h"
#include "pid.h"
#include "constants.h"
#include <math.h>
#include <fstream>
#include <string>

/**
 * Drives according to the left and right velocities inputted
 */
void Drive (double left_velocity, double right_velocity) {
    left_front_wheels.move_velocity(left_velocity);
    left_center_wheels.move_velocity(left_velocity);
    left_back_wheels.move_velocity(left_velocity);

    right_front_wheels.move_velocity(right_velocity);
    right_center_wheels.move_velocity(right_velocity);
    right_back_wheels.move_velocity(right_velocity);
    
}

/**
 * Reverses the drive
 */
void DriveDirection(int is_reversed){
	if(is_reversed == 1){
		left_back_wheels.set_reversed(false);
		left_center_wheels.set_reversed(false);
		left_front_wheels.set_reversed(false);

		right_back_wheels.set_reversed(true);
		right_center_wheels.set_reversed(true);
		right_front_wheels.set_reversed(true);
	} else {
		left_back_wheels.set_reversed(true);
		left_center_wheels.set_reversed(true);
		left_front_wheels.set_reversed(true);

		right_back_wheels.set_reversed(false);
		right_center_wheels.set_reversed(false);
		right_front_wheels.set_reversed(false);
	}
}

/*
 * Creates array of the csv
 */
double** CreateArray(int* line_number_out, int* array_width_out, std::string filename){
	const std::string file_path = filename;

	std::string line = "";
	std::ifstream csv_file;
	csv_file.open(file_path);

	int array_width = 5; //number of values that need to be parsed per line
	int* array_width_ptr = &array_width;

	int line_count = 0;
	int* line_count_ptr = &line_count;

	//counts number of lines in file
	while (getline(csv_file, line)) {
		line_count++;
	}

	line_count = ((line_count + 1) / 2); //adjust for getline() buffer and add extra line to store array length

	//creates array of correct size
	double** profile_info = new double*[line_count];

	for (int i = 0; i < line_count; i++){
		profile_info[i] = new double[array_width];
	}

	//resets file
	csv_file.clear();
	csv_file.seekg(0);

	//start to take things from CSV
	int current_line = 0;
	while (getline(csv_file, line)) {
		if (!(current_line & 1)) { //accounts for getline() buffer
			for (int current_val = 0; current_val < array_width; current_val++) {
				std::size_t comma_location = line.find(","); //finds and stores location of comma

				if (comma_location != std::string::npos) {//if comma exists
					profile_info[current_line/2][current_val] = stod(line.substr(0, comma_location));
					line = line.substr(comma_location + 1, line.length() - 1 - comma_location);
				}
				else {//if comma does not exist
					profile_info[current_line/2][current_val] = stod(line);
				}
				
			}
		}
		current_line++;
	}	

	array_width_out = array_width_ptr;
	line_number_out = line_count_ptr;

return profile_info;

delete profile_info;
}

/**
 * Converts inches into encoder units
 */
double InchToEncoder(double inches){
	double encoder_ticks;
	double circumference = M_PI * 4.125;
    double ticks_per_revolution = 900;
	encoder_ticks = ticks_per_revolution * (inches / circumference);
	return encoder_ticks;
}

/**
 * Takes in lines of a csv periodically
 * Drives along the path given by the csv
 * PID to stay on path with expected values from csv
 */
void MotionProfiling (std::string filename) {
    double left_current_position, right_current_position;    
    double left_intended_position, right_intended_position;    
    double left_velocity, right_velocity;  
    double current_heading, intended_heading;
    double output_left_velocity, output_right_velocity;
    double left_drive_pid_output, right_drive_pid_output, heading_pid_output;
	double** profile_info;

	int profile_info_length;
	int* profile_info_length_ptr = &profile_info_length;

	int profile_info_width;
	int* profile_info_width_ptr = &profile_info_width;

    const double rpm_max = 200.0; //rpm
    const double diameter_of_omni = 4.125; //inches
    const double seconds_per_minute = 60.0; //sec/min
    const double chassis_width = 14.125;//inches
    //const double v_max = M_PI * diameter_of_omni * rpm_max / seconds_per_minute;
    const double v_max = 0.863937979737; //calculated inches/sec

    Constants* constants = Constants::GetInstance();
    std::uint32_t now = pros::millis();

    auto left_drive_pid = new Pid(&constants->left_drive_kp, &constants->left_drive_ki,
                                  &constants->left_drive_kd);
    auto right_drive_pid = new Pid(&constants->right_drive_kp, &constants->right_drive_ki,
                                   &constants->right_drive_kd);
    auto heading_pid = new Pid(&constants->heading_kp, &constants->heading_ki,
                               &constants->heading_kd);
	profile_info = CreateArray(profile_info_length_ptr, profile_info_width_ptr, filename);

    for(int i = 0; i < profile_info_length; i++) {
        for (int j = 0; j < profile_info_width; j++) {
            switch(j) {
                case 0: left_intended_position = profile_info[i][j];
                case 1: left_velocity = profile_info[i][j];
                case 2: right_intended_position = profile_info[i][j];
                case 3: right_velocity = profile_info[i][j];
                case 4: intended_heading = profile_info[i][j];
            }
        //std::cout << std::to_string(profile_info[i][j]) + ",";
        }
        //std::cout << std::endl;

        left_current_position = left_front_wheels.get_position();
        right_current_position = right_front_wheels.get_position();
        current_heading = (right_current_position - left_current_position)
                          / chassis_width;
    
        left_drive_pid_output = left_drive_pid->Update(InchToEncoder(
                                left_intended_position), left_current_position);
        right_drive_pid_output = right_drive_pid->Update(InchToEncoder(
                                 right_intended_position), right_current_position);
        heading_pid_output = heading_pid->Update(intended_heading, current_heading);
        output_left_velocity = rpm_max * (left_velocity / v_max) + left_drive_pid_output
                               - heading_pid_output;
        output_right_velocity = rpm_max * (right_velocity / v_max) + right_drive_pid_output
                                + heading_pid_output;

        std::cout << "Current counter: " + std::to_string(i) << std::endl;         
        //std::cout << std::to_string(output_left_velocity) + "," + std::to_string(output_right_velocity) + std::endl;

        Drive(output_left_velocity, output_right_velocity);

        pros::Task::delay_until(&now, 20);
    }
    std::cout << filename + "Complete" << std::endl;
}

std::string BlueFrontAuton (bool parking) {
    std::string filename;
    if (parking == PARK) {
        filename = "/test.csv";
    } else {
        filename = "/test.csv";
    }
    return filename;
}

std::string BlueBackAuton (bool parking) {
    std::string filename;
    if (parking == PARK) {
        filename = "/test.csv";
    } else {
        filename = "/test.csv";
    }
    return filename;
}

std::string RedFrontAuton (bool parking) {
    std::string filename;
    if (parking == PARK) {
        filename = "/test.csv";
    } else {
        filename = "/test.csv";
    }
    return filename;
}

std::string RedBackAuton (bool parking) {
    std::string filename;
    if (parking == PARK) {
        filename = "/test.csv";
    } else {
        filename = "/test.csv";
    }
    return filename;
}

/**
 * Autonomous picker
 */
void AutonPicker (bool colour, bool side, bool parking) {
    std::string filename  = "/test.csv";

    if (colour == BLUE and side == FRONT) {
        filename = BlueFrontAuton(parking);
    } else if (colour == BLUE and side == BACK) {
        filename = BlueBackAuton(parking);
    } else if (colour == RED and side == FRONT) {
        filename = RedFrontAuton(parking);
    } else {
        filename = RedBackAuton(parking);
    }

    MotionProfiling(filename);
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
void autonomous () {
    AutonPicker(colour,side,parking);
}
