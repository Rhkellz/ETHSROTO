
#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include "RTfuncs.h"
#include <Thread.h>
#include <ThreadRunOnce.h>
#include <ArduinoSTL.h>
#include "vector"
using namespace std;

Romi32U4ButtonA buttonA;
Romi32U4Motors motors;
LSM6 imu;
Romi32U4Encoders encoders;

ThreadRunOnce moveBot = ThreadRunOnce();
char moves[200] = "";
vector<pair<int, char>> turns;
vector<float> numMoves;
int turnCnt = 0;
float totDist = 0;
bool constMove = false;
float givenTime = 0;
bool useGyro = false;
float offsetConst = 0;

void setup() {
  moveBot.onRun(move);
  moveBot.setRunOnce(1);
  buttonA.waitForButton();
  bool isEnd = false;
  bool isStart = false;
  int k = 0;
  int j = 0;
  char moves[200] = "";
//B R F L F L F F L F R F R F F R F L F R F F R F L F L F L L F R F L F F L F L F L L F R F R F F L F F R F E
//F L F L F L F E
//B L F R F L F R F R F L F L F O F R F R F L F R F F L F R F O F R F F O F L F L F R F L F R F E
  char UImoves[200] = "B F F F R F L F L F O F R F L F R F R F O F F R F F O F F F L F R F O F L F R F F R F E";
  givenTime = 80;
  constMove = false;
  useGyro = true;
  int distAdjust = 0;
  offsetConst = 5.00;//- if its leaning left, + if its leaning right
  if (useGyro) {
    delay(500);
    turnSensorSetup();
  } else {
    delay(2000);
  }
  turnSensorReset();
  
  while (!isEnd) {
      if (UImoves[k] != ' ') {
          moves[j] = UImoves[k];
          j++;
      }
      k++;
      if (UImoves[k] == 'E') {
          isEnd = true;
          moves[j] = UImoves[k];
      }
  }
  isEnd = false;
  int i = 0;
  int l = 0;
  while (!isEnd) {
      if (moves[i] == 'R' || moves[i] == 'L' || moves[i] == 'O') {
          turnCnt++;
      }
      i++;
      if (moves[i] == 'E') {
          isEnd = true;
      }
  }
  isEnd = false;
  i = 0;

  while (!isEnd) {
      if (moves[i] == 'R') {
          turns.push_back({i, 'R'});
          l++;
      }
      else if (moves[i] == 'L') {
          turns.push_back({i, 'L'});
          l++;
      } else if (moves[i] == 'O') {
          turns.push_back({i, 'O'});
          l++;
      }
      i++;
      if (moves[i] == 'E') {
          isEnd = true;
      }
  }
  isEnd = false;
  i = 0;
  l = 0;
  numMoves.resize(turnCnt+1);
  numMoves[l] = 0;
  for (int l = 0; l < turnCnt; l++) {
      while (i != turns[l].first) {
          if (moves[i] == 'F') {
              numMoves[l] += 50;
          }
          else if (moves[i] == 'B') {
              numMoves[l] += 33;
          }
          else if (moves[i] == 'E') {
              numMoves[l] += -8 + distAdjust;
          }
          i++;
      }
      i = turns[l].first+1;
  }

  while (!isEnd) {
      if (moves[i] == 'F') {
          numMoves[turnCnt] += 50;
      } else if (moves[i] == 'B') {
          numMoves[turnCnt] += 33; 
      } else if (moves[i] == 'E') {
          numMoves[turnCnt] += -8;
          isEnd = true;
      }
      i++;
  }
  for (int i = 0;i<turnCnt+1;i++) {
    totDist += numMoves[i];
  }
  preMove();
}

void loop() {
  if(moveBot.shouldRun()) {
    moveBot.run();
  }
  motors.setSpeeds(0,0);
}
