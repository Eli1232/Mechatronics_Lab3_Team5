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
3) figure out backwards and turn around


*/




//set pins for encoders, pwm, inputs, etc
int M1CHA = 2; //motor 1 "inputs"
int M1CHB = 3; //motor 1 "inputs"

int M2CHA = 18; //motor 2
int M2CHB = 19; //motor 2

volatile int encoderCountRight = 0;
volatile int encoderCountLeft = 0;
float countsPerDegree = (9.7 * 48) / 360.0;
float countsFor90Degrees = countsPerDegree * 90; //may need to be calibrated/changed

//sensor initialization
int signal = 52; //digital pin
float distance;
unsigned long pulseDuration; //USS 

//LEDs for debugging:
const int ledForward = 10;    
const int ledBackward = 11;
const int ledTurnLeft = 12;
const int ledTurnRight = 13;
const int ledStationary = 14;

//pixy signatures
const int SIGNATURE_FORWARD = 1;
const int SIGNATURE_LEFT = 2;
const int SIGNATURE_RIGHT = 3;
const int SIGNATURE_BACKWARD = 4;


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

  pinMode(M1CHA, INPUT);
  digitalWrite(M1CHB, HIGH);

  pinMode(M2CHA, INPUT);
  digitalWrite(M2CHB, HIGH);

  //sensor pin
  pinMode(signal, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(M1CHA), changeM1A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(M1CHB), changeM1B, CHANGE); //interrupt channel b

  attachInterrupt(digitalPinToInterrupt(M2CHA), changeM2A, CHANGE); //interrupt channel a
  attachInterrupt(digitalPinToInterrupt(M2CHB), changeM2B, CHANGE); //interrupt channel b

  pinMode(ledForward, OUTPUT);
  pinMode(ledBackward, OUTPUT);
  pinMode(ledTurnLeft, OUTPUT);
  pinMode(ledTurnRight, OUTPUT);
  pinMode(ledStationary, OUTPUT);

  angResolution = 360./(9.7 * 48);
    

}

void loop() {
  // put your main code here, to run repeatedly:

    // Turn off all LEDs initially
  digitalWrite(ledForward, LOW);
  digitalWrite(ledBackward, LOW);
  digitalWrite(ledTurnLeft, LOW);
  digitalWrite(ledTurnRight, LOW);
  digitalWrite(ledStationary, LOW);

  //should we have sensors on the side in case the robot needs to reorient itself by backing up???
  motors.enableDrivers();
  meausreDistance();
  distance = (pulseDuration * 0.0001 * 343)/2; //conversion for the distance


  switch(currentState){
    

    case FORWARD:
      //both motors turn forward at the same speed (no need for feedback control because we'll use distance sensors for that)
      //motors move until certain distance is achieved from wall, turn wheel, then send back to stationary

      digitalWrite(ledForward, HIGH);

      //if distance is within threshold, send state to stationary
      //measure distance
      if (distance >= 20){ //another trick could be while(distance>=20){rotate, reset distance}
        RotateCW_M1();
        RotateCW_M2();
      }
      else {
        setCarState(STATIONARY);
      }
      
      break;
      
    case BACKWARD:
      //both motors turn backward at the same speed, until certain distance 
      //red LED turns on
      digitalWrite(ledBackward, HIGH);

      if (distance <= 20){
        RotateCCW_M1();
        RotateCCW_M2();
      }
      else{
        setCarState(STATIONARY);
      }
      break;

    case TURN_LEFT:
      //right motor moves forward, left motor moves back
      //need someway to control how much the robot turns
      //left LED turns on
      digitalWrite(ledTurnLeft, HIGH);
      
      turnLeft();
      setCarState(STATIONARY);
      break;

    case TURN_RIGHT:
      //left motor moves forward, right motor moves back
      //need someway to control how much the robot turns
      //right LED moves back
      digitalWrite(ledTurnRight, HIGH);

      turnRight();
      setCarState(STATIONARY);
      break;

    case TURN_AROUND:
      //to be implemented
      break;

    case STATIONARY:
      //nothing happens
      digitalWrite(ledStationary, HIGH);
      //if sensor sees certain color, change state to forward
      //if sensor sees another color, turn left/right/backward
      motors.setM1Speed(0);
      motors.setM2Speed(0);

      meausreDistance();
      distance = (pulseDuration * 0.0001 * 343)/2; //conversion for the distance
      setCarState(PIXY_READ);

      break;

    case PIXY_READ:
      int detected_objects = pixy.ccc.getBlocks();
      if(detected_objects > 0){ //if greater than zero object has been detected
        for(int i = 0; i < detected_objects; i++ ){
          switch (pixy.ccc.blocks[i].m_signature) {
                case SIGNATURE_FORWARD:
                    setCarState(FORWARD);
                    break;
                case SIGNATURE_LEFT:
                    setCarState(TURN_LEFT);
                    break;
                case SIGNATURE_RIGHT:
                    setCarState(TURN_RIGHT);
                    break;
                case SIGNATURE_BACKWARD:
                    setCarState(BACKWARD);
                    break;
                // Add more cases??
                default:
                    // unknown signatures
                    break;
            }
            // Exit the loop after setting the state based on the first recognized signature
            break;
        }
        else{
          //no objects detected
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
    //delay(2);
  }

}
void RotateCW_M2(){

    for (int speed = 0; speed <= 400; speed++)
  {
    motors.setM2Speed(speed);
    stopIfFault();
    //delay(2);
  }


}
void RotateCCW_M1(){
  for (int speed = 400; speed >= 0; speed--)
  {
    motors.setM1Speed(speed);
    stopIfFault();
    //delay(2);
  }
}

  
void turnRight(){
    encoderCountRight = 0; // Reset right encoder count
    encoderCountLeft = 0;  // Reset left encoder count
    motors.setM1Speed(-200); //we want a set speed instead of a ramp up
    motors.setM2Speed(200);
    while(encoderCountRight < countsFor90Degrees && encoderCountLeft < countsFor90Degrees) {
        // Keep turning until the desired encoder count is reached
    }
}
void turnLeft(){
    encoderCountRight = 0; // Reset right encoder count
    encoderCountLeft = 0;  // Reset left encoder count
    motors.setM1Speed(200); //we want a set speed instead of a ramp up
    motors.setM2Speed(-200);
    while(encoderCountRight < countsFor90Degrees && encoderCountLeft < countsFor90Degrees) {
        // Keep turning until the desired encoder count is reached
    }
}

}
void RotateCCW_M2(){
    for (int speed = 400; speed >= 0; speed--)
  {
    motors.setM2Speed(speed);
    stopIfFault();
    //delay(2);
  }

}

//encoder counts
void changeM1A(){
  if(digitalRead(M1CHA) != digitalRead(M1CHB)){
    encoderCountRight++;
  }
  else{
    encoderCountRight--;
  }

}
void changeM1B(){
  if(digitalRead(M1CHA) != digitalRead(M1CHB)){
   encoderCountRight--;
  }
  else{
   encoderCountRight++;
  }
  
}
void changeM2A(){
  if(digitalRead(M2CHA) != digitalRead(M2CHB)){
   encoderCountLeft++;
  }
  else{
   encoderCountLeft--;
  }
  
}
void changeM2B(){
  if(digitalRead(M2CHA) != digitalRead(M2CHB)){
   encoderCountLeft--;
  }
  else{
    encoderCountLeft++;
  }
  
}

  

}
  
