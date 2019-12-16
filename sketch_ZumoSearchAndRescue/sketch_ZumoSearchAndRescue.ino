#include <Zumo32U4.h>


//Serial1 communicates over XBee
//Serial communicates over USB cable

Zumo32U4Motors motors;
Zumo32U4Buzzer buzzer;
#define TURNING_SPEED    150
#define MOVEMENT_TIME    150


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  int cmd; // for incoming serial data
  
  // send data only when you receive data:
  if (Serial1.available() > 0) {
    // read the incoming byte:
    cmd = Serial1.read();

    String name = "";
    char motor;
    int motorSpeed;
      
    switch (cmd){
      case 'w': case 'W': motors.setSpeeds(200,200); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0); Serial1.println(" Moving forward "); break;
      case 's': case 'S': motors.setSpeeds(-200,-200);delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0);Serial1.println(" Moving backwards ");  break;
      case 'a': case 'A': motors.setSpeeds(-TURNING_SPEED, TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0,0); Serial1.println(" Turning Left "); break;
      case 'd': case 'D': motors.setSpeeds(TURNING_SPEED,-TURNING_SPEED); delay(MOVEMENT_TIME); motors.setSpeeds(0,0); Serial1.println(" Turning Right "); break;
      case 'q': case'Q': motors.setLeftSpeed(0); motors.setRightSpeed(0); Serial1.println(" Stopping "); break;
      //case 'e': case 'E': ledRed(1); ledYellow(1); ledGreen(1); Serial1.print(" LED AND MUSIC TEST "); delay(1000);
      case 'e': case 'E': motors.setSpeeds(50,300); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0); Serial1.println(" Moving diagnol right "); break;
      case 'r': case 'R': motors.setSpeeds(300,50); delay(MOVEMENT_TIME * 5); motors.setSpeeds(0,0); Serial1.println(" Moving diagnol right "); break;
    }
    // end switch 
  }
}
