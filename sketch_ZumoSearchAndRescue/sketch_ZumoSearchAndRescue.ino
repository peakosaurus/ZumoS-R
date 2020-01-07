#include <Zumo32U4.h>
#include <Wire.h>


#define TURNING_SPEED    150
#define MOVEMENT_TIME    150


//Serial1 communicates over XBee
//Serial communicates over USB cable
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];
#define STRAIGHTFACTOR 1
//Andrew Brown. (February 25, 2017 ) Zumo 32U4 Synchronize Motor | A. Brown Design. Retrieved January 06, 2020, from http://www.abrowndesign.com/2017/02/25/zumo-32u4-synchronize-motor/
void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  150  // ms
#define TURN_DURATION     150  // ms
  lineSensors.initThreeSensors();
  calibrateSensors();

}

void loop() {
  // put your main code here, to run repeatedly:
  getInput();
}

void getInput() {
  motors.setSpeeds(0, 0);
  char cmd; // for incoming serial data

  // send data only when you receive data:
  while (Serial1.available() > 0) {
    // read the incoming byte:
    cmd = (char)Serial1.read();

    if (cmd == '1') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Manual Control ");
      manualMode();
    }
    else if (cmd == '2') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Automatic Control ");
      lineDetect();
    }
    else if (cmd == '0') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Sensor Calibrate ");
      calibrateSensors();
    }
  }
}
// taken from the line follower example
void calibrateSensors()
{

  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    ledYellow(1);
    buzzer.playNote(NOTE_G(3), 500, 15);
  }
  
  delay(1000);
  ledGreen(1);
  ledYellow(0);
  buzzer.playNote(NOTE_G(4), 500, 15);
  delay(1000);
  ledGreen(0);

  for (uint16_t i = 0; i < 120; i++)
  {
    if (i > 30 && i <= 90)
    {
      motors.setSpeeds(-200, 200);
    }
    else
    {
      motors.setSpeeds(200, -200);
    }

    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0);
  getInput();
}

void manualMode() {
  motors.setSpeeds(0, 0);
  int zumoMovement = ' ';
  while (zumoMovement != 'z') {
    zumoMovement = Serial1.read();
    if (zumoMovement == 'z') {
      Serial1.println(" Returning to Input Menu ");
      getInput();
    }
    switch (zumoMovement) {
      case 'w': case 'W': motors.setSpeeds(200, 200); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0, 0); Serial1.println(" Moving forward "); break;
      case 's': case 'S': motors.setSpeeds(-200, -200); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0, 0); Serial1.println(" Moving backwards ");  break;
      case 'a': case 'A': motors.setSpeeds(-TURNING_SPEED, TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0, 0); Serial1.println(" Turning Left "); break;
      case 'd': case 'D': motors.setSpeeds(TURNING_SPEED, -TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0, 0); Serial1.println(" Turning Right "); break;
      case 'q': case'Q': motors.setLeftSpeed(0); motors.setRightSpeed(0); Serial1.println(" Stopping "); break;
    }
  }
}

void lineDetect() {
  int countsLeft = encoders.getCountsAndResetLeft();
  int countsRight = encoders.getCountsAndResetRight();
  int currentSpeedLeft = FORWARD_SPEED;
  int currentSpeedRight = FORWARD_SPEED;
  int error;
  int correction;
  int cmd = ' ';

  motors.setSpeeds(0, 0);
  while (cmd != 'z') {
    cmd = Serial1.read();
    if (cmd == 'z') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      getInput();     
    }
    lineSensors.read(lineSensorValues);

    if (lineSensorValues[1] > lineSensors.calibratedMaximumOn[1] || (lineSensorValues[0] > lineSensors.calibratedMaximumOn[0] && lineSensorValues[2] > lineSensors.calibratedMaximumOn[2])) {
      // if the middle sensor or any other sensor detects a line stop.
      motors.setSpeeds(0, 0);
      countsRight = encoders.getCountsAndResetRight();
      countsRight = encoders.getCountsAndResetLeft();
      stopped();
      return;
    }
    else if ((lineSensorValues[0] > lineSensors.calibratedMaximumOn[0]) && (lineSensorValues[1] < lineSensors.calibratedMaximumOn[1]))
    {
      // If leftmost sensor detects line, reverse and turn to the right.
      delay(50);
      lineSensors.read(lineSensorValues);
      // delay to check the middle as it's not as far forward as the left and right.
      if (lineSensorValues[1] > lineSensors.calibratedMaximumOn[1] || (lineSensorValues[0] > lineSensors.calibratedMaximumOn[0] && lineSensorValues[2] > lineSensors.calibratedMaximumOn[2]))
      {
        motors.setSpeeds(0, 0);
        stopped();
        return;
      }
      else {
        motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
        delay(REVERSE_DURATION);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED + 15);
        delay(TURN_DURATION);
        countsLeft = encoders.getCountsAndResetLeft();
        countsRight = encoders.getCountsAndResetRight();
        motors.setSpeeds(0, 0);

      }
    }
    else if ((lineSensorValues[2] > lineSensors.calibratedMaximumOn[2]) && (lineSensorValues[1] < lineSensors.calibratedMaximumOn[1]))
    {
      // If rightmost sensor detects line, reverse and turn to the left.
      delay(50);
      lineSensors.read(lineSensorValues);
      if (lineSensorValues[1] > lineSensors.calibratedMaximumOn[1] || (lineSensorValues[0] > lineSensors.calibratedMaximumOn[0] && lineSensorValues[2] > lineSensors.calibratedMaximumOn[2]))
      {
        motors.setSpeeds(0, 0);
        stopped();
        return;
      }
      else {
        motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
        delay(REVERSE_DURATION);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(TURN_DURATION);
        countsLeft = encoders.getCountsAndResetLeft();
        countsRight = encoders.getCountsAndResetRight();
        motors.setSpeeds(0, 0);
      }
    }
    else if (lineSensorValues[0] < lineSensors.calibratedMaximumOn[0] && lineSensorValues[1] < lineSensors.calibratedMaximumOn[2]) {

      // motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED - 15); // previously used before encoder work
      motors.setSpeeds(currentSpeedLeft, currentSpeedRight);
      countsLeft = encoders.getCountsLeft();
      countsRight = encoders.getCountsRight();
      error = countsLeft - STRAIGHTFACTOR * countsRight;
      correction = 1 * error;
      currentSpeedRight = FORWARD_SPEED + correction;
      motors.setSpeeds(currentSpeedLeft, currentSpeedRight);
    }
  }
}

void stopped() {
  int input = ' ';
  Serial1.println("Stopped At Wall");
  while (input != 'z') {
    input = Serial1.read();
    if (input == 'z') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      return;
    }
    switch (input) {
      case 'l': case 'L':
        motors.setSpeeds(-200, -200);
        delay(MOVEMENT_TIME);
        motors.setSpeeds(0, 0);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(TURN_DURATION * 3);
        motors.setSpeeds(0, 0);
        Serial1.println(" Turning Left ");
        break;
      case 'r': case 'R':
        motors.setSpeeds(-200, -200);
        delay(MOVEMENT_TIME);
        motors.setSpeeds(0, 0);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(TURN_DURATION * 3);
        motors.setSpeeds(0, 0);
        Serial1.println(" Turning Right ");
        break;
      case 'c': case 'C':
        motors.setSpeeds(0, 0);
        Serial1.println(" Going Auto ");
        lineDetect();
        return;
        break;
    }
  }
}
