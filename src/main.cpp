#include <DHT.h>
#include <HUBeeBMDWheel.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>


#define DHTPIN 7
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
Servo myservo;

const int minimumObstacleDist = 50;
const int distToMoveBack = 10;

const int trigEchoPin = 10;
const int pingTime = 100;

const int leftWheelSpeed = 150;
const int rightWheelSpeed = 150;

const int stopTime = 200;
const int backTime = 300;
const int turnTime = 500;
const int cycleTime = 500;

const int buttonPin = 13;
const int leftIn1 = 4;
const int leftIn2 = 5;
const int leftPWM = 3;
const int rightIn1 = 8;
const int rightIn2 = 6;
const int rightPWM = 11;
const int SERVO_PIN = 9;
int pos = 90;

HUBeeBMDWheel leftWheel;
HUBeeBMDWheel rightWheel;

int buttonState;
float distance = 0;

void goForward() {
  leftWheel.setDirectionMode(0);
  rightWheel.setDirectionMode(1);
  leftWheel.setMotorPower(leftWheelSpeed);
  rightWheel.setMotorPower(rightWheelSpeed);
}

void stopRobot() {
  leftWheel.stopMotor(); 
  rightWheel.stopMotor();
}

void moveBack() {
  leftWheel.setDirectionMode(1);
  rightWheel.setDirectionMode(0);
  leftWheel.setMotorPower(leftWheelSpeed); 
  rightWheel.setMotorPower(rightWheelSpeed);
}

void turnRight() {
  leftWheel.setDirectionMode(0);
  rightWheel.setDirectionMode(0);
  leftWheel.setMotorPower(leftWheelSpeed); 
  rightWheel.setMotorPower(rightWheelSpeed);
 }

void turnLeft() {
  leftWheel.setDirectionMode(1);
  rightWheel.setDirectionMode(1);
  leftWheel.setMotorPower(leftWheelSpeed); 
  rightWheel.setMotorPower(rightWheelSpeed); 
}

void waitUntilButtonReleased() {
  while (digitalRead(buttonPin) == LOW);
}

void waitUntilButtonPushed() {
  while (digitalRead(buttonPin) == HIGH);
}


float getDistance() {
  float echoTime, soundSpeed, calculatedDistance;
  
  pinMode(trigEchoPin, OUTPUT);
  digitalWrite(trigEchoPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigEchoPin, LOW);
  pinMode(trigEchoPin, INPUT);
  echoTime = pulseIn(trigEchoPin, HIGH, 30000);  // Timeout to prevent long waits

  dht.begin();
  float temp = dht.readTemperature();
  float humi = dht.readHumidity();
  
  if (isnan(temp) || isnan(humi)) {
    Serial.println("Error: Temperature or Humidity reading failed.");
    return -1; // Indicate invalid reading
  }

  soundSpeed = 331.3 + 0.606 * temp + 0.0124 * humi;
  calculatedDistance = (soundSpeed / 10000) * echoTime / 2;

  if (calculatedDistance <= 0 || calculatedDistance > 300) { // Filter unrealistic distances
    Serial.println("Error: Invalid Distance reading.");
    return -1;
  }

  Serial.print("Temp: "); Serial.print(temp);
  Serial.print("  Humid: "); Serial.print(humi);
  Serial.print("  Distance[Cm]: "); Serial.println(calculatedDistance);

  delay(pingTime);

  return calculatedDistance;
}

void setup() {
  myservo.attach(SERVO_PIN);
  Serial.begin(9600);

  leftWheel.setupPins(leftIn1, leftIn2, leftPWM);
  rightWheel.setupPins(rightIn1, rightIn2, rightPWM);

  pinMode(buttonPin, INPUT_PULLUP);

  waitUntilButtonPushed();
}


// Ensure these variables are declared
float centerDistance, leftDistance, rightDistance, diffLRDistance;


void loop() {
  goForward();
  delay(cycleTime);

  centerDistance = getDistance();

  Serial.print("The centerDistance is: ");
  Serial.print(centerDistance);
  Serial.println(" cm");

  // Check if obstacle is within minimum distance
  if (centerDistance < minimumObstacleDist) {
    stopRobot();
    delay(stopTime);

    centerDistance = getDistance();
    Serial.print("centerDistance when stopped is ");
    Serial.println(centerDistance);

    // Servo rotates to 10 degrees to check the left side
    myservo.write(10);
    delay(300); // Adjust timing if needed
    leftDistance = getDistance();
    delay(100);
    Serial.print("leftDistance: ");
    Serial.println(leftDistance);

    // Servo rotates to 170 degrees to check the right side
    myservo.write(170);
    delay(600); // Adjust timing if needed
    rightDistance = getDistance();
    delay(100);
    Serial.print("rightDistance: ");
    Serial.println(rightDistance);

    // Center servo back to 90 degrees
    myservo.write(90);
    delay(300);

    // Calculate difference between left and right distances
    diffLRDistance = leftDistance - rightDistance;
    Serial.print("The difference between LR Distances: ");
    Serial.println(diffLRDistance);

    // Decide direction based on distance readings
    if (diffLRDistance > 0 && leftDistance > centerDistance) {
      Serial.println("Condition met: diffLRDistance > 0 && leftDistance > centerDistance");
      turnLeft();
      Serial.println("Going left");
      delay(turnTime);
    } else if (diffLRDistance < 0 && rightDistance > centerDistance) {
      Serial.println("Condition met: diffLRDistance < 0 && rightDistance > centerDistance");
      turnRight();
      Serial.println("Going right");
      delay(turnTime);
    } else if (centerDistance > leftDistance && centerDistance > rightDistance) {
      Serial.println("Condition met: centerDistance > leftDistance && centerDistance > rightDistance");
      moveBack();
      delay(backTime);
    } else {
      Serial.println("Condition met: default goForward");
      goForward();
      delay(cycleTime);
    }
  } 
  else if (centerDistance < distToMoveBack && centerDistance < minimumObstacleDist) {
    Serial.println("Condition met: centerDistance < distToMoveBack && centerDistance < minimumObstacleDist");
    moveBack();
    delay(backTime);
    Serial.println("Back");
  } 
  else {
    Serial.println("Condition met: default Fwd");
    goForward();
    delay(cycleTime);
    Serial.println("Fwd");
  }
}

