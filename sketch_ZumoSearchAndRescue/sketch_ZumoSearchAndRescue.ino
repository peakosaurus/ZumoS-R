#include <Zumo32U4.h>
#include <Wire.h>


#define TURNING_SPEED    150
#define MOVEMENT_TIME    150
#define QTR_THRESHOLD    1000  // microseconds


//Serial1 communicates over XBee
//Serial communicates over USB cable

Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];
int calibratedLineSensorValue[3]; // used instead of QTR-THRESHOLD
//bool atWall = false;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  100  // ms
#define TURN_DURATION     100  // ms
  lineSensors.initThreeSensors();
  calibrateSensors();
  for (int i = 0; i < 3 ; i++) {
  calibratedLineSensorValue[i] = lineSensors.calibratedMaximumOn[i];
  // The calibrated maximum values measured for each sensor, with emitters on.
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  getInput();
}

void getInput() {
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
    buzzer.playNote(NOTE_G(3), 200, 15);
  }
  delay(1000);
  buzzer.playNote(NOTE_G(4), 500, 15);
  delay(1000);

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
  int cmd = ' ';

  while (cmd != 'z') {
    cmd = Serial1.read();
    if (cmd == 'z') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      getInput();
    }
    lineSensors.read(lineSensorValues);

    if (lineSensorValues[1] > calibratedLineSensorValue[1] || (lineSensorValues[0] > calibratedLineSensorValue[0] && lineSensorValues[2] > calibratedLineSensorValue[2])) {
      motors.setSpeeds(0, 0);
    }
    else if ((lineSensorValues[0] > calibratedLineSensorValue[0]) && (lineSensorValues[1] < calibratedLineSensorValue[1]))
    {
      // If leftmost sensor detects line, reverse and turn to the
      // right.
      delay(10);
      lineSensors.read(lineSensorValues);
      delay(10);
      if (lineSensorValues[1] > calibratedLineSensorValue[1] || (lineSensorValues[0] > calibratedLineSensorValue[0] && lineSensorValues[2] > calibratedLineSensorValue[2]))
      {
        motors.setSpeeds(0, 0);
        //        atWall = true;
      }
      else {
        motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
        delay(REVERSE_DURATION);
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
        delay(TURN_DURATION);
        motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      }
    }
    else if ((lineSensorValues[2] > calibratedLineSensorValue[2]) && (lineSensorValues[1] < calibratedLineSensorValue[1]))
    {
      // If rightmost sensor detects line, reverse and turn to the
      // left.
      delay(10);
      lineSensors.read(lineSensorValues);
      delay(10);
      if (lineSensorValues[1] > calibratedLineSensorValue[1] || (lineSensorValues[0] > calibratedLineSensorValue[0] && lineSensorValues[2] > calibratedLineSensorValue[2]))
      {
        motors.setSpeeds(0, 0);
        //       atWall = true;
      }
      else {
        motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
        delay(REVERSE_DURATION);
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        delay(TURN_DURATION);
        motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      }
    }
    else {
      motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
      //      atWall = false;
    }
  }
  //    if (atWall) {
  //    Serial1.println(" at Wall press M for manual mode");
  //    if (cmd == 'M' || cmd == 'm') {
  //      Serial1.println(" Manual Control ");
  //      manualMode();
  //    }
}
