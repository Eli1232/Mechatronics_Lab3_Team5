#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield motors;
Pixy2 pixy;

/*
1. You need five states: go forward, go backward, turn left, turn right, and turn around.
2. To make the turning movement, you send different PWM signals to the DC motors so
one moves faster and the other move slower (differential speed turning), or you can make
them rotate in opposite direction (point turning). Both methods are useful, but in this lab
the point turning method is suggested. You can also use your encoder signal to track.
*/


//set pins for encoders, pwm, inputs, etc
int ENCPINA_M1 = 2; //motor 1 "inputs"
int ENCPINB_M1 = 3; //motor 1 "inputs"



int ENCPINA_M2 = 18; //motor 2
int ENCPINB_M2 = 19; //motor 2

volatile int counterM1 = 0;
volatile int counterM2 = 0;

float gearRatio = 9.7;
int CPR = 48;
float angResolution;
float angPosM1 = 0;
float angle1 = 0;

float angPosM2 = 0;
float angle2 = 0;




enum Action {
  FORWARD, 
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  STATIONARY,
  PIXY_READ
};

//robot state variables
Action volatile currentState = STATIONARY;

//prototypes
void setCarState(Action newState);






void setup() {
  // put your setup code here, to run once:
  //attach interrupt functions
  Serial.begin(115200); //for the pixy
  Serial.print("Starting...\n"); //for the pixy
  pixy.init(); //for the pixy

  pinMode(ENCPINA_M1, INPUT);
  digitalWrite(ENCPINB_M1, HIGH);

  pinMode(ENCPINA_M2, INPUT);
  digitalWrite(ENCPINB_M2, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCPINA_M1), changeM1A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(ENCPINB_M1), changeM1B, CHANGE); //interrupt channel b

  attachInterrupt(digitalPinToInterrupt(ENCPINA_M2), changeM2A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(ENCPINB_M2), changeM2B, CHANGE); //interrupt channel b

  angResolution = 360./(9.7 * 48);






}

void loop() {
  // put your main code here, to run repeatedly:

  //continuously get the sensor values
  //should we have sensors on the side in case the robot needs to reorient itself by backing up???

  motors.enableDrivers();

  switch(currentState){

    case FORWARD:
      //both motors turn forward at the same speed (no need for feedback control because we'll use distance sensors for that)
      //motors move until certain distance is achieved from wall, turn wheel, then send back to stationary
      break;
    case BACKWARD:
      //both motors turn backward at the same speed, until certain distance 
      //red LED turns on
      break;
    case TURN_LEFT:
      //right motor moves forward, left motor moves back
      //need someway to control how much the robot turns
      //left LED turns on
      break;
    case TURN_RIGHT:
      //left motor moves forward, right motor moves back
      //need someway to control how much the robot turns
      //right LED moves back
      break;
    case STATIONARY:
      //nothing happens
      //if sensor sees certain color, change state to forward
      //if sensor sees another color, turn left/right/backward

      break;
    case PIXY_READ:
      break;

  }

//rotating functions
void RotateCW_M1(){

}
void RotateCW_M2(){

}
void RotateCCW_M1(){

}
void RotateCCW_M2(){

}

//encoder counts
void changeM1A

  

}
  
