#include <SPI.h>
#include <Pixy2.h>
#include <DualMAX14870MotorShield.h>

DualMAX14870MotorShield motors;
Pixy2 pixy;

#define PID 0
#define NON_PID 1

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
int M1CHA = 19; //yellow, left, motor 1 "inputs"
int M1CHB = 18; //white, left, motor 1 "inputs"

int M2CHA = 2; //yellow, right, motor 2
int M2CHB = 3; //white, right, motor 2

volatile int encoderCountRight = 0;
volatile int encoderCountLeft = 0;
float countsPerDegree = (9.7 * 48) / 360.0;
float countsFor90Degrees = countsPerDegree * 90 * 9.25 / 3.5; //may need to be calibrated/changed, counts for 90 degrees of the robot, not the motor shaft
//value for above should be 230.14, I added a 0 to make it easier to debug the modes. Should determine empirically.
float countsFor180Degrees = countsFor90Degrees * 2.0;
float angResolution;

//sensor initialization
int signal = 49; //digital pin
float distance;
int distThresh = 20;
unsigned long pulseDuration; //USS

//LEDs for debugging:
const int ledForward = 48;
const int ledBackward = 11;
const int ledTurnLeft = 53;
const int ledTurnRight = 46;
//const int ledStationary = 51;
const int ledTurnAround = 47;


//pixy signatures
const int SIGNATURE_LEFT = 1;
const int SIGNATURE_RIGHT = 2;
const int SIGNATURE_TURN_AROUND = 3;



//global variables to store the last encoder count and the last time checked
volatile int lastEncoderCountRight = 0;
volatile int lastEncoderCountLeft = 0;
unsigned long lastTimeChecked = 0;


//speed control
float speedRight = 0;
float speedLeft = 0;
int adjustment = 25; //for adjusting the speed between the two motors
float kp = 0.5; //proportional control for the turning



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
//Action volatile currentState = FORWARD;

//prototypes
void setCarState(Action newState) {
  currentState = newState;
}





void setup() {
  // put your setup code here, to run once:
  //time the speed was last checked
  lastTimeChecked = millis();

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
  pinMode(ledTurnAround, OUTPUT);

  angResolution = 360. / (9.7 * 48);
}

void loop() {
  // put your main code here, to run repeatedly:
  motors.enableDrivers();
  // Turn off all LEDs initially
  ledOff();

  //should we have sensors on the side in case the robot needs to reorient itself by backing up???

  measureDistance();
  distance = (pulseDuration * 0.0001 * 343) / 2; //conversion for the distance
  Serial.print("distance: ");
  Serial.println(distance);
  Serial.print("state: ");
  Serial.println(currentState);



  //SPEED CONTROL NON-PID
  unsigned long currentTime = millis();
  if(currentTime - lastTimeChecked >= 100){ //check every 100 miliseconds
    int deltaCountRight = encoderCountRight - lastEncoderCountRight;
    int deltaCountLeft = encoderCountLeft - lastEncoderCountLeft;
    unsigned long deltaTime = currentTime - lastTimeChecked;

    //calculate speed in counts per second
    speedRight = (deltaCountRight/(float)deltaTime) * 1000;
    speedLeft = (deltaCountRight/(float)deltaTime) * 1000;

    lastEncoderCountRight = encoderCountRight;
    lastEncoderCountLeft = encoderCountLeft;
    lastTimeChecked = currentTime;

    Serial.print("Speed Right: ");
    Serial.println(speedRight);

    Serial.print("Speed Left: ");
    Serial.println(speedLeft);

    //compare distance between the two wheels
    int distanceDifference =  (encoderCountRight - encoderCountLeft);
    if (abs(distanceDifference) > diff_distance_threshold){
      if(distanceDifference > 0){ //this means that the right turned more than the left
        // Right wheel is ahead, slow down right motor or speed up left motor
          adjustMotorSpeeds(speedLeftMotor + adjustment, speedRightMotor - adjustment);
        } else {
          // Left wheel is ahead, slow down left motor or speed up right motor
          adjustMotorSpeeds(speedLeftMotor - adjustment, speedRightMotor + adjustment);

        } 

      }




    } 


  }  


  switch (currentState) {


    case FORWARD:
      ledOff();
      //both motors turn forward at the same speed (no need for feedback control because we'll use distance sensors for that)
      //motors move until certain distance is achieved from wall, turn wheel, then send back to stationary

      digitalWrite(ledForward, HIGH);

      //if distance is within threshold, send state to stationary
      //measure distance
      if (distance >= distThresh) { //another trick could be while(distance>=20){rotate, reset distance}
        RotateCW_M1();
        RotateCW_M2();
      }
      else {
        setCarState(STATIONARY);
      }

      break;

    case BACKWARD:
      ledOff();
      //both motors turn backward at the same speed, until certain distance
      //red LED turns on
      digitalWrite(ledBackward, HIGH);

      if (distance <= distThresh) {
        RotateCCW_M1();
        RotateCCW_M2();
      }
      else {
        setCarState(STATIONARY);
      }
      break;

    case TURN_LEFT:
      ledOff();
      //right motor moves forward, left motor moves back
      //need someway to control how much the robot turns
      //left LED turns on
      digitalWrite(ledTurnLeft, HIGH);

      turnLeft();
      setCarState(STATIONARY);
      break;

    case TURN_RIGHT:
      ledOff();
      //left motor moves forward, right motor moves back
      //need someway to control how much the robot turns
      //right LED moves back
      digitalWrite(ledTurnRight, HIGH);

      turnRight();
      setCarState(STATIONARY);
      break;

    case TURN_AROUND:
      ledOff();
      digitalWrite(ledTurnAround, HIGH);
      turnAround();
      setCarState(STATIONARY);
      break;

    case STATIONARY:
      ledOff();
      //nothing happens
      //  digitalWrite(ledStationary, HIGH);
      //if sensor sees certain color, change state to forward
      //if sensor sees another color, turn left/right/backward
      motors.setM1Speed(0);
      motors.setM2Speed(0);
      if (distance <= distThresh) { //if robot close to the wall, read the color, otherwise, keep going forward
        setCarState(PIXY_READ);
      }
      else {
        setCarState(FORWARD);
      }
      break;

    case PIXY_READ:
      int detected_objects = pixy.ccc.getBlocks();
      if (detected_objects > 0) { //if greater than zero object has been detected
        for (int i = 0; i < detected_objects; i++ ) {
          switch (pixy.ccc.blocks[i].m_signature) {
            case SIGNATURE_LEFT:
              setCarState(TURN_LEFT);
              break;
            case SIGNATURE_RIGHT:
              setCarState(TURN_RIGHT);
              break;
            case SIGNATURE_TURN_AROUND:
              setCarState(TURN_AROUND);
              break;
            default:
              // unknown signatures
              break;
          }
          // Exit the loop after setting the state based on the first recognized signature
          break;
        }

      }
      else {
        //no objects detected
      }
      break;

  }
  delay(5);
}

void measureDistance() {

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
//adjustment for motor speeds
void adjustMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Ensure the speeds are within allowable range
    leftSpeed = constrain(leftSpeed, 0, 400);
    rightSpeed = constrain(rightSpeed, 0, 400);

    // Set the motor speeds
    motors.setM1Speed(leftSpeed);
    motors.setM2Speed(rightSpeed);
}



//rotating functions
void RotateCW_M1() {
  motors.setM1Speed(200);
}
void RotateCW_M2() {
  motors.setM2Speed(200);
}
void RotateCCW_M1() {
  motors.setM1Speed(-200);
}
void RotateCCW_M2() {
  motors.setM2Speed(-200);
}

void turnRight() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  delay(200); // this delay reduces inertia and improves consistency
  encoderCountRight = 0; // Reset right encoder count
  encoderCountLeft = 0;  // Reset left encoder count
  motors.setM1Speed(200); //we want a set speed instead of a ramp up
  motors.setM2Speed(-200);

  while(abs(encoderCountRight) < countsFor90Degrees){
    int error = counts90Degrees - abs(encoderCountRight);
    //proportional control
    int speed = kp * error;
    //constrain the speed
    speed = constrain(speed, 0, 200);
    motors.setM1Speed(speed);
    motors.setM2Speed(-speed); // Opposite direction for turning

  }
  /*
  while (encoderCountLeft < countsFor90Degrees && encoderCountRight > -1 * countsFor90Degrees) {
    // Keep turning until the desired encoder count is reached
    Serial.print("right encoder count (right): ");
    Serial.println(encoderCountRight);
    Serial.print("left encoder count (right): ");
    Serial.println(encoderCountLeft);
  }
  */
  motors.setM1Speed(0); //wait 1 second after turning
  motors.setM2Speed(0);
  delay(1000);
}
void turnLeft() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  delay(200); // this delay reduces inertia and improves consistency
  encoderCountRight = 0; // Reset right encoder count
  encoderCountLeft = 0;  // Reset left encoder count

  motors.setM1Speed(-200); //we want a set speed instead of a ramp up
  motors.setM2Speed(200);

  while(abs(encoderCountLeft) < countsFor90Degrees){
    int error = counts90Degrees - abs(encoderCountLeft);
    //proportional control
    int speed = kp * error;
    //constrain the speed
    speed = constrain(speed, 0, 200);
    motors.setM1Speed(-speed);
    motors.setM2Speed(speed); // Opposite direction for turning

  }
  /*
  while ((encoderCountLeft > -1 * countsFor90Degrees) or (encoderCountRight < countsFor90Degrees)) {
    // Keep turning until the desired encoder count is reached
    Serial.print("right encoder count (left): ");
    Serial.println(encoderCountRight);
    Serial.print("left encoder count (left): ");
    Serial.println(encoderCountLeft);
  }
  */
  motors.setM1Speed(0); //wait 1 second after turning
  motors.setM2Speed(0);
  delay(1000);
}

void turnAround() {
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  delay(200); // this delay reduces inertia and improves consistency
  encoderCountRight = 0; // Reset right encoder count
  encoderCountLeft = 0;  // Reset left encoder count
  //  motors.setM1Speed(0); //we want a set speed instead of a ramp up
  //  motors.setM2Speed(0);
  motors.setM1Speed(-200); //we want a set speed instead of a ramp up
  motors.setM2Speed(200);

  while(abs(encoderCountLeft) < countsFor180Degrees){
    int error = counts180Degrees - abs(encoderCountLeft);
    //proportional control
    int speed = kp * error;
    //constrain the speed
    speed = constrain(speed, 0, 200);
    motors.setM1Speed(-speed);
    motors.setM2Speed(speed); // Opposite direction for turning
  }
  
  /*
  while ((encoderCountLeft > -1 * countsFor180Degrees) or (encoderCountRight < countsFor180Degrees)) {
    // Keep turning until the desired encoder count is reached
    Serial.print("right encoder count (around): ");
    Serial.println(encoderCountRight);
    Serial.print("left encoder count (around): ");
    Serial.println(encoderCountLeft);
  }
  */
  motors.setM1Speed(0); //wait 1 second after turning
  motors.setM2Speed(0);
  delay(1000);
}


//encoder counts
void changeM1A() {
  if (digitalRead(M1CHA) != digitalRead(M1CHB)) {
    encoderCountLeft--;
  }
  else {
    encoderCountLeft++;
  }

}
void changeM1B() {
  if (digitalRead(M1CHA) != digitalRead(M1CHB)) {
    encoderCountLeft++;
  }
  else {
    encoderCountLeft--;
  }

}
void changeM2A() {
  if (digitalRead(M2CHA) != digitalRead(M2CHB)) {
    encoderCountRight++;
  }
  else {
    encoderCountRight--;
  }

}
void changeM2B() {
  if (digitalRead(M2CHA) != digitalRead(M2CHB)) {
    encoderCountRight--;
  }
  else {
    encoderCountRight++;
  }

}

void stopIfFault()
{
  if (motors.getFault())
  {
    while (1)
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
  }
}

void ledOff() {
  digitalWrite(ledForward, LOW);
  digitalWrite(ledBackward, LOW);
  digitalWrite(ledTurnLeft, LOW);
  digitalWrite(ledTurnRight, LOW);
  digitalWrite(ledTurnAround, LOW);
}
