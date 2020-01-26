#include "TurnSensor.h"

// For angles measured by the gyro, our convention is that a
// value of (1 << 29) represents 45 degrees.  This means that a
// uint32_t can represent any angle between 0 and 360.
// this was taken from the Maze Solver example
const int32_t gyroAngle45 = 0x20000000;
//Serial1 communicates over XBee
//Serial communicates over USB cable
Zumo32U4Encoders encoders;
Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors; // taken from the sumo proximity example
L3G gyro;

// person found count, rooms searched, bool for indicating if someone was in a room and the location.
int personCount = 0;
int roomCount = 0;
bool personFound = false;
String roomPosition = " ";
// flag for when we reach the t junction.
bool reachedIntersection = false;
// end of corridor flag.
bool endOfCorridor = false;

// counts from the T junction until the user stops (for a search).
int junctionCountsLeft = 0;
int junctionCountsRight = 0;

// second counts for when the motors start again. These figures will be combined to the above, to give the distance to the T junction.
int junctionCountsLeft2 = 0;
int junctionCountsRight2 = 0;

//Line Sensor Variables
#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

// Variables for keeping the zumo driving straight, source below.
// used for keeping the motors in synch
int countsLeft = encoders.getCountsAndResetLeft();
int countsRight = encoders.getCountsAndResetRight();
#define STRAIGHTFACTOR 1
//Andrew Brown.
//(February 25, 2017 ) Zumo 32U4 Synchronize Motor | A. Brown Design.
//Retrieved January 06, 2020, from http://www.abrowndesign.com/2017/02/25/zumo-32u4-synchronize-motor/


/*
  description stuff
*/

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
#define REVERSE_SPEED     200  // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  100  // ms
#define TURN_DURATION     100  // ms
  //alt speeds
#define RIGHT_TURNING_SPEED    225
#define MOVEMENT_TIME    150
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  turnSensorSetup();
  calibrateSensors();

}
/*------------------START OF INPUT MENU-------------------- */
void loop() {
  // enter the main menu system
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

    else if (cmd == '3') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Search Mode ");
      searchRoom();
    }

    /*-----------------DEBUGGING FEATURES----------------  */
    else if (cmd == '8') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Scan Mode ");
      proximityScan();
      Serial1.println (personFound);
    }

    else if (cmd == '9') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Variable print ");
      variablePrint();
    }

    else if (cmd == '0') {
      motors.setSpeeds(0, 0);
      Serial1.println (" Sensor Calibrate ");
      calibrateSensors();
    }
  }
}
/*--------------------END OF INPUT MENU-------------------- */

/*--------------------MANUAL CONTROL-------------------- */
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
      case 'w': case 'W': motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0, 0); Serial1.println(" Moving forward "); break;
      case 's': case 'S': motors.setSpeeds(-FORWARD_SPEED, -FORWARD_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0, 0); Serial1.println(" Moving backwards ");  break;
      case 'a': case 'A': motors.setSpeeds(-TURN_SPEED, TURN_SPEED); delay(TURN_DURATION); motors.setSpeeds(0, 0); Serial1.println(" Turning Left "); break;
      case 'd': case 'D': motors.setSpeeds(TURN_SPEED, -TURN_SPEED); delay(TURN_DURATION); motors.setSpeeds(0, 0); Serial1.println(" Turning Right "); break;
      case 'q': case'Q': motors.setLeftSpeed(0); motors.setRightSpeed(0); Serial1.println(" Stopping "); break;
    }
  }
}

/*--------------------AUTOMATIC MODE AKA LINE DETECT-------------------- */

void lineDetect() {
  int currentSpeedLeft = FORWARD_SPEED;
  int currentSpeedRight = FORWARD_SPEED;
  int error;
  int correction;
  int cmd = ' ';
  countsLeft = encoders.getCountsAndResetLeft();
  countsRight = encoders.getCountsAndResetRight();

  while (cmd != 'z') {
    cmd = Serial1.read();
    if (cmd == 'z') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      if (reachedIntersection) {
        junctionCountsLeft = encoders.getCountsLeft();
        junctionCountsRight = encoders.getCountsRight();
      }
      return;
      //getInput();
    }
    lineSensors.read(lineSensorValues);

    if (lineSensorValues[1] > lineSensors.calibratedMaximumOn[1] || (lineSensorValues[0] > lineSensors.calibratedMaximumOn[0] && lineSensorValues[2] > lineSensors.calibratedMaximumOn[2])) {
      // if the middle sensor or any other sensor detects a line stop.
      // calibratedMaximumOn https://www.pololu.com/docs/0J19/all
      motors.setSpeeds(0, 0);
      stopped();
      return;
    }
    else if ((lineSensorValues[0] > lineSensors.calibratedMaximumOn[0]) && (lineSensorValues[1] < lineSensors.calibratedMaximumOn[1]))
    {
      // If leftmost sensor detects line, reverse and turn to the right.
      //delay for 50 milliseconds
      delay(10);
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
      delay(10);
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

/*--------------------OPTIONS FOR WHEN THE ZUMO HAS STOPPED-------------------- */
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
    // if we previously stopped at an intersection, then we will be stopping at the end of the corridor
    // therefore we get the counts for the wheels for the journey from the room to the end. We'll combine this with the other figures saved
    // to return to to the intersection.
    if (reachedIntersection == true) {
      endOfCorridor = true;
      junctionCountsLeft2 = encoders.getCountsLeft();
      junctionCountsRight2 = encoders.getCountsRight();
      Serial1.println(" At end of corridor return to intersection ");
      atIntersection();
      return;
    };
    switch (input) {
      case 'l': case 'L':
        backUp();
        left90();
        break;
      case 'r': case 'R':
        backUp();
        right90();
        break;
      case 'c': case 'C':
        motors.setSpeeds(0, 0);
        Serial1.println(" Going Auto ");
        lineDetect();
        return;
      case 't': case 'T':
        motors.setSpeeds(0, 0);
        Serial1.println(" At T Junction ");
        atIntersection();
        return;
        break;
    }
  }
}

/*--------------------USER AT T JUNCTION-------------------- */
void atIntersection() {
  bool returningToJunction = false;
  reachedIntersection = true;

  int input = ' ';
  if (!endOfCorridor) {
    Serial1.println("Left or Right?");
  }
  else {
    Serial1.println("Press B to return to junction, ");
  }
  while (input != 'z') {
    input = Serial1.read();
    if (input == 'z' && !returningToJunction) {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      return;
    }
    else if (input == 'z' && returningToJunction) {
      Serial1.println(" Area already searched! ");
      buzzer.playNote(NOTE_G(4), 500, 15);
    }

    if ((input == 'l' && endOfCorridor == false) || (input == 'L' && endOfCorridor == false) ) {
      backUp();
      left90();
      lineDetect();
      return;
    }
    else if ((input == 'r' && endOfCorridor == false) || (input == 'R' && endOfCorridor == false)) {
      backUp();
      right90();
      lineDetect();
      return;
    }

    if (endOfCorridor) {
      returningToJunction = true;
      if (input == 'b' || input == 'B') {
        // combine pre room journey and post room journey to get distance to intersection
        int leftTotal = (junctionCountsLeft += junctionCountsLeft2);
        int rightTotal = (junctionCountsRight += junctionCountsRight2);
        // rotate 180
        right90();
        right90();
        delay(500);
        // Serial1.println("Calculating return journey");
        countsLeft = encoders.getCountsAndResetLeft();
        countsRight = encoders.getCountsAndResetRight();
        delay(500);
        motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
        do {
          countsLeft = encoders.getCountsLeft();
          countsRight = encoders.getCountsRight();
        }  while (countsLeft < leftTotal && countsRight < rightTotal);
        motors.setSpeeds(0, 0);
        Serial1.println(" Returning to Input Menu ");
        reachedIntersection = false;
        endOfCorridor = false;
        leftTotal = 0;
        rightTotal = 0;
        junctionCountsLeft = 0;
        junctionCountsRight = 0;
        junctionCountsLeft2 = 0;
        junctionCountsRight2 = 0;
        reachedIntersection = false;
        return;
      }
    }
  }
}

/*--------------------SEARCH ROOM-------------------- */
void searchRoom() {
  int input = ' ';
  roomCount ++;
  delay(1000);
  Serial1.println("Left or Right?");
  while (input != 'z') {
    input = Serial1.read();
    if (input == 'z') {
      motors.setSpeeds(0, 0);
      Serial1.println(" Returning to Input Menu ");
      return;
    }

    switch (input) {
      case 'l' : case 'L' :
        personFound = false;
        roomPosition = "left";
        left90();
        moveIntoRoom();
        //rotate 45° to the Right and scan
        right45();
        proximityScan();
        // rotate 90° to the left to scan other side of room
        left90();
        proximityScan();
        //rotate 45° back to original position
        right45();
        proximityScan();
        // back out of the room to corridor position
        moveOutRoom();
        //rotate back to starting position
        right90();
        // signal to the user that a object was found.
        if (personFound) {
          buzzer.playNote(NOTE_E(3), 500, 15);
          Serial1.println(" Person detected. ");
          personCount += 1 ;
        }
        /* display if a person was found
           which room
           and where the room is situated
        */
        Serial1.print(personFound);
        Serial1.print(" person found in room ");
        Serial1.print(roomCount);
        Serial1.print (" on the ");
        Serial1.print (roomPosition);

        break;
      case 'r': case 'R' :
        personFound = false;
        roomPosition = "right";
        //rotate 90° to face the room
        right90();
        moveIntoRoom();
        left45();
        proximityScan();
        right90();
        proximityScan();
        left45();
        proximityScan();

        // signal to the user that a object was found.
        if (personFound) {
          buzzer.playNote(NOTE_E(3), 500, 15);
          Serial1.println(" Person detected. ");
          personCount += 1 ;
        }
        // display current totals
        Serial1.print(personFound);
        Serial1.print(" person found in room ");
        Serial1.print(roomCount);
        Serial1.print (" on the ");
        Serial1.print (roomPosition);
        moveOutRoom();
        //rotate back to starting position
        left90();
        break;
    }
  }
}
/*-------------------CALLIBRATE LINE SENSORS-------------------- */
// taken from the line follower example
void calibrateSensors()
{
  // Play audible countdown.
  for (int i = 0; i < 3; i++)
  {
    delay(1000);
    ledYellow(1);
//    buzzer.playNote(NOTE_G(3), 500, 15);
  }

  delay(1000);
  ledGreen(1);
  ledYellow(0);
//  buzzer.playNote(NOTE_G(4), 500, 15);
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
  return;
}


/*-------------------COMMON MOVEMENT FUNCTIONS--------------------
  Movements that I use all the time, right angle turns, backing up from walls
  moving into rooms. etc.
*/

void backUp() {
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(0, 0);
}

void moveIntoRoom() {
  countsLeft = encoders.getCountsAndResetLeft();
  countsRight = encoders.getCountsAndResetRight();
  delay(500);
  motors.setSpeeds(100, 100);
  Serial1.println(" Moving into room ");
  do {
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
  }  while (countsLeft < 1000 && countsRight < 1000);
  motors.setSpeeds(0, 0);
}

void moveOutRoom() {
  countsLeft = encoders.getCountsAndResetLeft();
  countsRight = encoders.getCountsAndResetRight();
  delay(1000);
  motors.setSpeeds(-100, -100);
  do {
    countsLeft = encoders.getCountsLeft();
    countsRight = encoders.getCountsRight();
  }  while (countsLeft > -1000 && countsRight > -1000);
  motors.setSpeeds(0, 0);
}

void left90() {
  Serial1.println(" Turning Left ");
  delay(500);
  motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  while ((int32_t)turnAngle < turnAngle45 * 2)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorReset();
  delay(500);
}

void right90() {
  Serial1.println(" Turning Right ");
  delay(500);
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  while ((int32_t)turnAngle > -turnAngle45 * 2)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorReset();
  delay(500);
}

void left45() {
  Serial1.println(" Turning Left ");
  delay(500);
  motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
  while ((int32_t)turnAngle < turnAngle45)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorReset();
  delay(500);
}

void right45() {
  Serial1.println(" Turning Right ");
  delay(500);
  motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
  while ((int32_t)turnAngle > -turnAngle45)
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorReset();
  delay(500);
}
/*-------------------SCAN FUNCTION--------------------
  VERY SIMPLE SCAN FUNCTION USED IN THE SUMO EXAMPLE.
  WORKS OK WILL NEED TO CHECK AGASINT FULL SIZE MAP.
*/
void proximityScan() {
  proxSensors.read();
  if (proxSensors.countsFrontWithLeftLeds() >= 6
      || proxSensors.countsFrontWithRightLeds() >= 6)
  {
    personFound = true;
  }
}

/*-------------------DEBUGGING FUNCTION--------------------*/
void variablePrint() {
  Serial1.print(" Persons found = ");
  Serial1.println(personCount);
  Serial1.print(" Rooms Searched = ");
  Serial1.println(roomCount);
  Serial1.print(" reachedIntersection = ");
  Serial1.println(reachedIntersection);
  Serial1.print(" endOfCorridor = ");
  Serial1.println(endOfCorridor);
  Serial1.print(" junctionCountsLeft = ");
  Serial1.println(junctionCountsLeft);
  Serial1.print(" junctionCountsRight = ");
  Serial1.println(junctionCountsRight);
  Serial1.print(" junctionCountsLeft2 = ");
  Serial1.println(junctionCountsLeft2);
  Serial1.print(" junctionCountsRight2 = ");
  Serial1.println(junctionCountsRight2);
}
