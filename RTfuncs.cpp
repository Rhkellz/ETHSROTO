
#include <Wire.h>
#include <Romi32U4.h>
#include <LSM6.h>
#include "RTfuncs.h"
#include <Thread.h>
#include <ThreadRunOnce.h>
#include <ArduinoSTL.h>
#include "vector"
using namespace std;

uint32_t turnAngle = 0;
uint32_t turnAngleInt = 0;
int32_t turnSpeed;

float turnRate;

float gyroOffset;

uint16_t gyroLastUpdate = 0;
uint16_t encLastUpdate = 0;
float lastEncError = 0;
float speed = 0;
float totalDist;
float turnTime;
float targetVel;
int Rencoder = 0;
int Lencoder = 0;
int RencTurn = 0;
int LencTurn = 0;

void turnSensorSetup() {//this function calibrates the gyro, do no touch robot while yellow LED is on
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.writeReg(LSM6::CTRL2_G, 0b10001000);

  ledYellow(1);
  delay(500);
  float total = 0;
  for (uint16_t i = 0; i < 2000; i++) {//gets average reading as the offset
    while(!imu.readReg(LSM6::STATUS_REG) & 0x08);
    imu.read();
    total += imu.g.z;
  }
  ledYellow(0); 
  gyroOffset = total/2000 + offsetConst;
}


void turnSensorReset() {
  gyroLastUpdate = micros();
  encLastUpdate = micros();
  turnAngle = 0;
  turnAngleInt = 0;
}

void encoderReset() {
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
  LencTurn = 0;
  RencTurn = 0;
}


void turnSensorUpdate() {//PID controller for the gyro
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;
  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;
  int32_t d = (int32_t)turnRate * dt;
  
  turnAngle += (int64_t)d * 7340032 / 17578125;//unit stuff idk
  turnAngleInt += d * dt;
  turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 12)
    - (turnRate / 100)- turnAngleInt/(turnAngle1*100);

  turnSpeed = constrain(turnSpeed, -80 , 80);

}
 
void encturnSensorUpdate() {//PID controller for the encoders
  Rencoder = encoders.getCountsRight() + RencTurn;
  Lencoder = encoders.getCountsLeft() + LencTurn;
  float encError = Lencoder - Rencoder;//if positive, leaning right, if negative, leaning left
  uint16_t m = micros();
  float dt = m - encLastUpdate;
  encLastUpdate = m;

  float dx = encError - lastEncError;
  turnSpeed = encError * encK + dx/dt * 75;
  turnSpeed = constrain(turnSpeed, -80 , 80);
  Serial.println(turnSpeed);//load bearing maybe????? dont touch
}

void pmoveDist(float cmVal) {
  totalDist+=cmVal;
}

void pturnRight() {
  turnTime += 0.74;
}

void pturnLeft() {
  turnTime += 0.74;
}

void moveDist(float cmVal, float totTime) {//trapezoidal motion profile, yes its a mess
  encoderReset();
  float tVel = 0;
  float encoderVal = cmVal * 65.3485659;
  speed = 0;
  int count = 0;
  float lastt = millis();
  float initT =  millis();
  float kp = 94.0;
  float lastd = 0;
  float dx = 0;
  float dt = 0;
  float vel = 0;
  float diff = 0;
  float accl = 9.0*cmVal/100.0 * 1.0/(2.0 * totTime*totTime);
    Serial.println(accl);
    Serial.println("stage 1");
    while ((millis()-initT)/1000 < totTime/3) {
      if (count % 4 == 0) {
        dx = ((encoders.getCountsRight() - lastd) / 65.3485659)/100.0;
        dt = (millis()-lastt)/1000.0;
        lastd = encoders.getCountsRight();
        lastt = millis();
        if (dt > 0.0) {
          vel = dx / dt;
        } else {
          vel = tVel;
        }
        diff = tVel - vel;
        tVel = accl * (millis()-initT)/1000;
        speed += diff * (kp+20);
        constrain(speed, 0, 300);
        if (tVel < 0.03) {
          tVel = 0.03;
        }
      }
      if (useGyro) {
        turnSensorUpdate();
      } else {
        encturnSensorUpdate();
      }
      motors.setSpeeds(speed-turnSpeed, speed+turnSpeed);
      count++;
    }
    Serial.println(encoders.getCountsRight());
    Serial.println(encoderVal/4);
    Serial.println("stage 2");
    while ((millis()-initT)/1000 < 2*totTime/3) {
      if (count % 8 == 0) {
        dx = ((encoders.getCountsRight() - lastd) / 65.3485659)/100.0;
        dt = (millis()-lastt)/1000.0;
        lastd = encoders.getCountsRight();
        lastt = millis();
        if (dt > 0.0) {
          vel = dx / dt;
        } else {
          vel = tVel;
        }
        diff = tVel - vel;
        tVel = totTime/3 * accl;
        speed += diff * kp;
        constrain(speed, 0, 300);
      }
      if (useGyro) {
        turnSensorUpdate();
      } else {
        encturnSensorUpdate();
      }
      motors.setSpeeds(speed-turnSpeed, speed+turnSpeed);
      count++;
    }
    Serial.println(encoders.getCountsRight());
    Serial.println(3*encoderVal/4);
    Serial.println("stage 3");
    while ((millis()-initT)/1000 < totTime) {
      if (count % 8 == 0) {
        dx = ((encoders.getCountsRight() - lastd) / 65.3485659)/100.0;
        dt = (millis()-lastt)/1000.0;
        lastd = encoders.getCountsRight();
        lastt = millis();
        if (dt > 0.0) {
          vel = dx / dt;
        } else {
          vel = tVel;
        }
        tVel = accl*(totTime-((millis()-initT)/1000.0));
        diff = tVel - vel;
        speed += diff * 115;
        constrain(speed, 0, 300);
      }
      if (useGyro) {
        turnSensorUpdate();
      } else {
        encturnSensorUpdate();
      }
      motors.setSpeeds(speed-turnSpeed, speed+turnSpeed);
      count++;
    }
  Serial.println("time: ");
  Serial.println((millis()-initT)/1000);
  Serial.println("dist: ");
  Serial.println(encoders.getCountsRight()/65.348);
  encoderReset();
}

void constMoveDist(float cmVal, float totTime) {//constant velocity profile
  encoderReset();
  float tVel = (cmVal / 100)/totTime;
  Serial.println(tVel);
  float encoderVal = cmVal * 65.3485659;
  speed = 70;
  int count = 0;
  float lastt = millis();
  float initT = millis();
  float kp = 120.0;
  float lastd = 0;
  float dx = 0;
  float dt = 0;
  float vel = 0;
  float diff = 0;
  while (encoders.getCountsRight() < encoderVal) {
      if (count % 8 == 0) {
        dx = ((encoders.getCountsRight() - lastd) / 65.3485659)/100.0;
        dt = (millis()-lastt)/1000.0;
        lastd = encoders.getCountsRight();
        lastt = millis();
        if (dt > 0.0) {
          vel = dx / dt;
        } else {
          vel = 0.0;
        }
        diff = tVel - vel;
        speed += diff * (kp);
        if (tVel < 0.04) {
          tVel = 0.04;
        }
      }
      if (useGyro) {
        turnSensorUpdate();
      } else {
        encturnSensorUpdate();
      }

      motors.setSpeeds(speed-turnSpeed, speed+turnSpeed);
      count++;
    }
  Serial.println("time: ");
  Serial.println((millis()-initT)/1000);
  Serial.println("tTime: ");
  Serial.println(totTime);
  Serial.println("dist: ");
  Serial.println(encoders.getCountsRight()/65.348);
  Serial.println(tVel);
  encoderReset();
}

void turnSafety() {
  if (useGyro) {
    turnSensorUpdate();
  } else {
    encturnSensorUpdate();
  }
  while (abs(turnSpeed) > 3) {
    if (useGyro) {
      turnSensorUpdate();
    } else {
      encturnSensorUpdate();
    }
    motors.setSpeeds(-turnSpeed, turnSpeed);
  }
}
int encTurnConst = 726;
void turnLeft() {
  if (useGyro) {
    gyroLastUpdate = micros();
    turnAngle = 3*turnAngle90;
  } else {
    LencTurn += 729;
    RencTurn -= 729;
  }
  turnSafety();
}

void turnRight() {
  if (useGyro) {
    gyroLastUpdate = micros();
    turnAngle = turnAngle90;
  } else {
    RencTurn += 722;
    LencTurn -= 722;
  }
  turnSafety();
}
void turn180() {
  if (useGyro) {
    gyroLastUpdate = micros();
    turnAngle = turnAngle90*2;
  } else {
    RencTurn += encTurnConst*2;
    LencTurn -= encTurnConst*2;
  }
  turnSafety();
}

void move() {
encoderReset();
for (int i = 0; i < turnCnt; i++) {
      if (!constMove) {
        moveDist(numMoves[i], numMoves[i]/totDist * (givenTime-turnTime));
      } else {
        constMoveDist(numMoves[i], numMoves[i]/totDist * (givenTime-turnTime));
      }
    if (turns[i].second == 'R') {
        turnRight();
    } else if (turns[i].second == 'L') {
        turnLeft();
    } else if (turns[i].second == 'O') {
        turn180();
    }
  }
  if (!constMove) {
    moveDist(numMoves[turnCnt], numMoves[turnCnt]/totDist * (givenTime-turnTime));
  } else {
    constMoveDist(numMoves[turnCnt], numMoves[turnCnt]/totDist * (givenTime-turnTime));
  }
}

void preMove() {
  for (int i = 0; i < turnCnt; i++) {
    pmoveDist(numMoves[i]);
    if (turns[i].second == 'R') {
        pturnRight();
    } else if (turns[i].second == 'L') {
        pturnLeft();
    } else if (turns[i].second == 'O') {
        pturnLeft();
        pturnLeft();
    }
}
  pmoveDist(numMoves[turnCnt]);
  targetVel = (totalDist/100) / (givenTime-turnTime);
  Serial.println("move time: ");
  Serial.println(givenTime-turnTime);
  Serial.println("target velocity: ");
  Serial.println(targetVel);
}
