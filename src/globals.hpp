#pragma once

#include "../include/main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <vector>

#include "headers/custom_math.hpp"
#include "headers/geometry.hpp"

using namespace std;

// SETTINGS

// drivetrain types
#define TANK_DRV 1
#define X_DRV 2
// movable part types
#define TYPE_MTR 1
#define TYPE_PNEU 2

// DEFINITIONS

// controller
inline pros::Controller master(pros::E_CONTROLLER_MASTER);

// motor and adi constants
const int MTR_MAX = 127;
const int MTR_MAXmV = 12000;
const double GRN_RPM = 200;
const double GRN_RPS = GRN_RPM/60;
const double BLU_RPM = 600;
const double BLU_RPS = BLU_RPM/60;
const double RED_RPM = 100;
const double RED_RPS = RED_RPM/60;
inline map<int,int> gear_mp = {
    {pros::E_MOTOR_GEAR_GREEN, GRN_RPM},
    {pros::E_MOTOR_GEAR_BLUE, BLU_RPM},
    {pros::E_MOTOR_GEAR_RED, RED_RPM}
};
const int ADI_MAX = 4095;

// drivetrain
const double WHEEL_R  = 1.0*60/84; // inches
const double WHEEL_C = WHEEL_R*M_PI*2;
inline double WHEEL_RPM = 0; // initialize later
inline double WHEEL_RPS = 0; // initialize later
inline double WHEEL_LSPD = 0; // initialize later
inline pros::Motor flmotor(17, pros::E_MOTOR_GEAR_BLUE, true);
inline pros::Motor frmotor(20, pros::E_MOTOR_GEAR_BLUE);
inline pros::Motor rlmotor(18, pros::E_MOTOR_GEAR_BLUE, true);
inline pros::Motor rrmotor(19, pros::E_MOTOR_GEAR_BLUE);

// sensing
const int TILE = 24; // inches
const int FIELD = TILE*6;
const double GRVTY = 9.8;
inline pros::IMU inertial(16);
inline pros::Rotation trackx(9);
inline pros::Rotation tracky(8);

// FUNCTIONS

// time
#define TIME_IMPL 1
#if TIME_IMPL == 0 // uses native timer
inline double time() {
    timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    return t.tv_sec+t.tv_nsec*0.000000001;
}
#elif TIME_IMPL == 1 // uses pros timer
inline double time() {
    return pros::micros()*0.000001;
}
#endif