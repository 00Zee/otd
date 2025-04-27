#include <Joystick.h>

// Encoder wiring
const int encPinA = 2;
const int encPinB = 3;

// Motor driver pins
const int motorPWM = 6;
const int motorDir1 = 8;
const int motorDir2 = 9;

// Paddle shift buttons
const int shifterUpPin = 11;
const int shifterDownPin = 12;

// Stuff to track wheel position
volatile long wheelPos = 0;
bool lastAState = HIGH;

// Tuning for force feedback
float kp = 1.0;  // how hard it fights back
float kd = 1.0;  // how quickly it reacts to movement
long lastError = 0;

// Where we want the wheel to be (middle)
int centerPos = 0;

// Set up joystick
Joystick_ joyStick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD,
  2, 0,                  // 2 buttons, no hat switch
  true, false, false,     // X-axis enabled
  false, false, false,    // no rotation stuff
  false, false,           // no rudder/throttle
  false, false, false);   // no extra features

void setup() {
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);

  pinMode(motorPWM, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);

  pinMode(shifterUpPin, INPUT_PULLUP);
  pinMode(shifterDownPin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encPinA), updateWheel, CHANGE);

  joyStick.begin();
}

void loop() {
  long posNow = wheelPos;
  long errorNow = centerPos - posNow;
  long errorSpeed = errorNow - lastError;

  // calculate how much force to apply
  int motorPower = kp * errorNow + kd * errorSpeed;

  // tiny deadzone so motor isn't twitchy at center
  if (abs(errorNow) < 3) motorPower = 0;

  motorPower = constrain(motorPower, -255, 255);

  if (motorPower > 0) {
    digitalWrite(motorDir1, HIGH);
    digitalWrite(motorDir2, LOW);
    analogWrite(motorPWM, motorPower);
  } else if (motorPower < 0) {
    digitalWrite(motorDir1, LOW);
    digitalWrite(motorDir2, HIGH);
    analogWrite(motorPWM, -motorPower);
  } else {
    analogWrite(motorPWM, 0);  // no movement
  }

  lastError = errorNow;

  // send steering data to PC
  int xAxisVal = map(posNow, -512, 512, 0, 1023);
  xAxisVal = constrain(xAxisVal, 0, 1023);
  joyStick.setXAxis(xAxisVal);

  // check paddle shifters
  if (digitalRead(shifterUpPin) == LOW) {
    joyStick.pressButton(0);  // right paddle
  } else {
    joyStick.releaseButton(0);
  }

  if (digitalRead(shifterDownPin) == LOW) {
    joyStick.pressButton(1);  // left paddle
  } else {
    joyStick.releaseButton(1);
  }

  delay(5); // just a little breathing room
}

// runs whenever the wheel moves
void updateWheel() {
  bool currentA = digitalRead(encPinA);
  bool currentB = digitalRead(encPinB);

  // figuring out which way the wheel is spinning
  if (currentA != lastAState && currentA == HIGH) {
    if (currentB == LOW) {
      wheelPos++;
    } else {
      wheelPos--;
    }
  }
  lastAState = currentA;
}
