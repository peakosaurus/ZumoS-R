#include <Zumo32U4.h>
#include <Wire.h>


#define TURNING_SPEED    150
#define MOVEMENT_TIME    150
#define QTR_THRESHOLD     1000  // microseconds

//Serial1 communicates over XBee
//Serial communicates over USB cable

Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;

#define NUM_SENSORS 3
unsigned int lineSensorValues[NUM_SENSORS];

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
}

void loop() {
  // put your main code here, to run repeatedly:
   getInput();
}

void getInput(){
   char cmd; // for incoming serial data
  
  // send data only when you receive data:
  while (Serial1.available() > 0) {
    // read the incoming byte:
    cmd = (char)Serial1.read();
     
    if (cmd == '1'){
      motors.setSpeeds(0,0);
      Serial1.println(" Manual Control ");
      manualMode(); 
    }
    else if (cmd == '2'){
      motors.setSpeeds(0,0);
      Serial1.println (" Automatic Control");
    }
  }
}

void manualMode(){
  
  int zumoMovement = " ";
  while (zumoMovement != 'z'){
    zumoMovement = Serial1.read();
    if (zumoMovement == 'z') {
      Serial1.println(" Returning to Input Menu ");
      getInput();
      } 
    switch (zumoMovement){
     case 'w': case 'W': motors.setSpeeds(200,200); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0); Serial1.println(" Moving forward "); break;
     case 's': case 'S': motors.setSpeeds(-200,-200);delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0);Serial1.println(" Moving backwards ");  break;
     case 'a': case 'A': motors.setSpeeds(-TURNING_SPEED, TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0,0); Serial1.println(" Turning Left "); break;
     case 'd': case 'D': motors.setSpeeds(TURNING_SPEED,-TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0,0); Serial1.println(" Turning Right "); break;
     case 'q': case'Q': motors.setLeftSpeed(0); motors.setRightSpeed(0); Serial1.println(" Stopping "); break;
    }
  }
}

void lineDetect(){
  int zumoMovement = " ";
  while (zumoMovement != 'z'){
    zumoMovement = Serial1.read();
    lineSensors.read(lineSensorValues);
    
    if (zumoMovement == 'z') {
      Serial1.println(" Returning to Input Menu ");
      getInput();
      }
      
   else if (zumoMovement == 'A'){    
   if (lineSensorValues[0] > QTR_THRESHOLD)
  {
    // If leftmost sensor detects line, reverse and turn to the
    // right.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (lineSensorValues[NUM_SENSORS - 1] > QTR_THRESHOLD )
  {
    // If rightmost sensor detects line, reverse and turn to the
    // left.
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  else
  {
    // Otherwise, go straight.
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
  }
}
}
