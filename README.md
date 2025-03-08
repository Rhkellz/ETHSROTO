# This is a guide on operation of my Robot Tour robot

## General Info:
 * The robot is a Pololu Romi
 * The code is based on 'Rotation Resist' from the example programs in Pololu's github
 * This project uses the Arduino IDE (make sure to download all necessary libraries).


## Dos and Don'ts:
 * **DON'T!!** update the board 'Arduino AVR Boards' past 1.8.6 (what its currently at) even if it asks nicely. This will cause the library 'ArduinoSTL' to break.
 * Do keep all the files in a folder with the same name as the INO file (RTmain).
 * Don't mess with the code. (if it ain't broke dont fix it).
 * Do make sure you keep the batteries at high charge.
 * Don't touch or move the robot when the yellow LED is on, its calibrating the gyro.
 * Do keep the wheels/axle clean of hair and debris, and take off the wheels to clean the axle if necessary. 
 * Do have a backup of the files somewhere.

## How To Use the Robot:
 * Input the desired movements into the char array UImoves, which is near the top of the RTmain.ino file.
 * Always start with B for Begin, which moves it forward 33 cm.
 * F moves it forward 50 cm.
 * R and L turns it left and right, and O turns it 180 degrees.
 * Always end with E for End, which cuts the movement short by 8 cm.
 * Input the desired time in seconds in the float givenTime.
 * Choose whether to use trapezoidal motion profiles or to move with a constant velocity by setting boolean constMove to true/false. This should basically always be false. Trapezoidal is slightly less accurate theoretically but stops slippage, so maybe try constant movement on really slow tracks?
 * Choose whether to use the gyroscope for sensing heading, or the encoders in the motors by setting boolean useGyro to true/false. This should usually be set to true. Both are relatively accurate, but if the gyro is really bad for some reason, definitely switch. When starting, it will take ~5 seconds to calibrate the gyroscope if useGyro is set to true.
 * if you are using the gyro, edit the value of float offsetConst if the robot is leaning to the right/left. You usually only want to change it by ~.10.
 * If the robot ended a few cm before or after the target, edit the value of int distAdjust. This value in cm will get added to the end of the movement. 
 * Upload the code to the robot, the LEDs on it will flash if its really uploading, besides what the computer tells you.
 * Press the on button, which is the leftmost one. A blue LED should turn on.
 * When ready to go, press the button directly to the right of the on button.

## Troubleshooting:
 * The IDE stuck on 'Compiling...'?: If there's no progress bar, close and reopen the IDE. It can be pretty finicky. Otherwise wait.
 * Code not uploading to the robot?: Check if its connected in the bottom right corner of the IDE. If not, go to Tools *> Port.
 * The IDE is just not working?: Make sure the board is 'Pololu AStar 32U4' in Tools -> Boards.
 * Robot doesnt move when you start it? just reupload and try again, this happens sometimes idk why.
 * Help??: try to contact me maybe.


## Here's some practice tracks:
* B R F L F L F F L F R F R F F R F L F R F F R F L F L F L L F R F L F F L F L F L L F R F R F F L F F R F E

* B L F R F L F R F R F L F L F O F R F R F L F R F F L F R F O F R F F O F L F L F R F L F R F E

* B F F F R F L F L F O F R F L F R F R F O F F R F F O F F F L F R F O F L F R F F R F E

(I know the code is kinda shitty but if it works it works)

**Rowan Kelly 2025**
