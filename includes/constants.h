#pragma once

#include <cmath>
#include <vector>

const double PI = std::acos(-1);
const double DEG_TO_RAD = PI / 180.0;

const int N_MOTOR = 3;
const double WHEEL_RADIUS = 30;
const double ROBOT_RADIUS = 116;
const double TICS_PER_ROTATION = 200; //From Datasheet Servo42C

const double MAX_WHEEL_RADIUS = 120; //To be checked
const double MAX_ROBOT_RADIUS = 200; //To be checked

//Variable used to convert mm/s in a number of steps, given in datasheet
const double SPEED_TO_RPM_FACTOR = 30'000;

//Variable used to scale the calculated value of the speeds
const double SPEED_CORRECTION_FACTOR = 0.25;

const double MSTEP = 64; //Mstep = 16
const double RPM_MINIMUM = 0;
const double RPM_MAXIMUM = 2000;
const double MSTEP_MINIMUM = 1;
const double MSTEP_MAXIMUM = 256;

/**
 * @attention Items have to be correctly associated and following counterclockwise ordrer
 */
const std::vector<double> X_AXIS_ANGLE_MOTORS = { 60, 180, 300 };

const double theta = 60*DEG_TO_RAD;
const double a[3] = {0, 120*DEG_TO_RAD, 240*DEG_TO_RAD};

/**
 * @attention Items have to be correctly associated and following counterclockwise ordrer
 */
const std::vector<uint8_t> ADDRESS_MOTORS = { 0xE1, 0xE2, 0xE3 };