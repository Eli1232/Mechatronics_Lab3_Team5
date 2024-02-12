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

TODO:
1) get distance thresholds
2) add LEDs
3) figure out backwards and turna around


*/




//set pins for encoders, pwm, inputs, etc
int M1CHA = 2; //motor 1 "inputs"
int M1CHA = 3; //motor 1 "inputs"

int M2CHA = 18; //motor 2
int M2CHB = 19; //motor 2

volatile int counterM1 = 0;
volatile int counterM2 = 0;

//sensor init
int signal = 52; //digital pin
float distance;
unsigned long pulseDuration; //USS 



float gearRatio = 9.7;
int CPR = 48;
float angResolution;
float angPosM1 = 0;
float angle1 = 0;

float angPosM2 = 0;
float angle2 = 0;

int i = 0; //this is for indexing pixy objects


enum Action {
  FORWARD, 
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT,
  TURN_AROUND,
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

  //sensor pin
  pinMode(signal, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(M1CHA), changeM1A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(M1CHB), changeM1B, CHANGE); //interrupt channel b

  attachInterrupt(digitalPinToInterrupt(M2CHA), changeM2A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(M2CHB), changeM2B, CHANGE); //interrupt channel b

  angResolution = 360./(9.7 * 48);

}

void loop() {
  // put your main code here, to run repeatedly:

  //should we have sensors on the side in case the robot needs to reorient itself by backing up???
  motors.enableDrivers();
  meausreDistance();
  distance = (pulseDuration * 0.0001 * 343)/2; //conversion for the distance


  switch(currentState){
    

    case FORWARD:
      //both motors turn forward at the same speed (no need for feedback control because we'll use distance sensors for that)
      //motors move until certain distance is achieved from wall, turn wheel, then send back to stationary
      RotateCW_M1();
      RotateCW_M2();

      //if distance is within threshold, send state to stationary
      //measure distance
      setCarState(STATIONARY);
      break;
      
    case BACKWARD:
      //both motors turn backward at the same speed, until certain distance 
      //red LED turns on
      RotateCCW_M1();
      RotateCCW_M2();
      setCarState(STATIONARY);
      break;

    case TURN_LEFT:
      //right motor moves forward, left motor moves back
      //need someway to control how much the robot turns
      //left LED turns on
      RotateCCW_M1();
      RotateCW_M2();
      setCarState(STATIONARY);
      break;

    case TURN_RIGHT:
      //left motor moves forward, right motor moves back
      //need someway to control how much the robot turns
      //right LED moves back
      RotateCW_M1();
      RotateCCW_M2();
      setCarState(STATIONARY);
      break;

    case TURN_AROUND:
      //to be implemented
      break;

    case STATIONARY:
      //nothing happens
      //if sensor sees certain color, change state to forward
      //if sensor sees another color, turn left/right/backward
      motors.setM1Speed(0);
      motors.setM2Speed(0);

      meausreDistance();
      distance = (pulseDuration * 0.0001 * 343)/2; //conversion for the distance
      setCarState(PIXY_READ);

      break;

    case PIXY_READ:
      int detected_objects = pixy.ccc.numBlocks;
      if(detected_objects > 0){ //if greater than zero object has been detected
        for(int i = 0; i < detected_objects; i++ ){
          if (pixy.ccc.blocks[i].m_signature == 1){ //signature of detected object 
              setCarState(TURN_RIGHT);

          }
          if (pixy.ccc.blocks[i].m_signature == 2){
              setCarState(TURN_LEFT);

          }
          if (pixy.ccc.blocks[i].m_signature == 3){
              setCarState(TURN_AROUND);
          }
        }
      }
      break;

  }

void measureDistance(){

  //set pin as output so we can send a pulse
  pinMode(signal, OUTPUT);
  //set output to LOW
  digitalWrite(signal, LOW);
  delayMicroseconds(5);

  //now send the 5uS pulse out to activate the PING
  digitalWrite(signal, HIGH);
  delayMicroseconds(5);
  digitalWrite(signal, LOW);


  //now we need to change the digital pin 
  //to input to read the incoming pulse
  pinMode(signal, INPUT);

  //finally, measure the length of the incoming pulse
  pulseDuration = pulseIn(signal, HIGH);
 
}

//rotating functions
void RotateCW_M1(){
  
    for (int speed = 0; speed <= 400; speed++)
  {
    motors.setM1Speed(speed);
    stopIfFault();
    delay(2);
  }

}
void RotateCW_M2(){

    for (int speed = 0; speed <= 400; speed++)
  {
    motors.setM2Speed(speed);
    stopIfFault();
    delay(2);
  }


}
void RotateCCW_M1(){
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    stopIfFault();
    delay(2);
  }


}
void RotateCCW_M2(){
    for (int speed = 400; speed >= 0; speed--)
  {
    motors.setM2Speed(speed);
    stopIfFault();
    delay(2);
  }

}

//encoder counts
void changeM1A(){
  if(digitalRead(M1CHA) != digitalRead(M1CHB)){
    counterM1++;
  }
  else{
    counterM1--;
  }

}
void changeM1B(){
  if(digitalRead(M1CHA) != digitalRead(M1CHB)){
    counterM1--;
  }
  else{
    counterM1++;
  }
  
}
void changeM2A(){
  if(digitalRead(M2CHA) != digitalRead(M1CHB)){
    counterM1++;
  }
  else{
    counterM1--;
  }
  
}
void changeM1B(){
  if(digitalRead(M2CHA) != digitalRead(M1CHB)){
    counterM1--;
  }
  else{
    counterM1++;
  }
  
}

  

}
  
