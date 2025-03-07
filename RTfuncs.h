
#pragma once

#include <Romi32U4.h>
#include <LSM6.h>
#include <ArduinoSTL.h>
#include "vector"
using namespace std;

// This constant represents a turn of 45 degrees.
const int32_t turnAngle45 = 0x20000000;

// This constant represents a turn of 90 degrees.
const int32_t turnAngle90 = turnAngle45 * 2;

// This constant represents a turn of approximately 1 degree.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

void turnSensorSetup();
void turnSensorReset();
void turnSensorUpdate();
void encoderReset();
void move();
void moveDist();
void turnSafety();
void preMove();
void trapezoidal();

extern int32_t turnSpeed;
extern char moves[];
extern int turnCnt;
extern vector<pair<int, char>> turns;
extern vector<float> numMoves;
extern float totDist;
extern bool constMove;
extern float givenTime;
extern bool useGyro;
extern float offsetConst;

extern Romi32U4ButtonA buttonA;
extern Romi32U4LCD lcd;
extern LSM6 imu;
extern Romi32U4Encoders encoders;
extern Romi32U4Motors motors;
