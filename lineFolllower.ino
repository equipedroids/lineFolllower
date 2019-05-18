#include <FalconRobot.h>
// OLÁ MUNDO
// Motors
FalconRobotMotors motors(5, 7, 6, 8);
// Line sensors
FalconRobotLineSensor leftLightSensor(A2);
FalconRobotLineSensor rightLightSensor(A3);
// Default values for line sensors
#define leftInLine 1003
#define leftOutOfLine 911
#define rightInLine 1004
#define rightOutOfLine 926
// Variables for right motor
int rightLight;
int rightSumError = 0;
int rightLastError = 0;
int rightSpeed = 0;
int rightDirection = 0;
// Variables for left motor
int leftLight;
int leftSumError = 0;
int leftLastError = 0;
int leftSpeed = 0;
int leftDirection = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);
}

int getSpeed(int light, int defaultLight, int sumError, int lastError){
  float Kp = 0.3;
  float Ki = 0.3;
  float Kd = 0.3;
  float proporcional;
  float integral;
  float derivada;
  int error;
  float motorSpeed;
  // int sumError;
  // int lastError;
  error = light - defaultLight;
  proporcional = error * Kp;
  integral = sumError * Ki;
  derivada = (error - lastError) * Kd;
  motorSpeed = proporcional + integral + derivada;
  lastError = error;
  return motorSpeed;
}

void loop() {
  // Line sensor reading
  leftLight = leftLightSensor.read();
  rightLight = rightLightSensor.read();
  
  if(leftLight <= leftInLine-30 && rightLight <= rightInLine-30){
    //Serial.println("Fora da linha");
    Serial.print(rightLight);
    Serial.print("\t");
    Serial.println(leftLight);
    motors.drive(40, FORWARD);
  }else{
    //Serial.println("Na linha");
    Serial.print(rightSpeed);
    Serial.print("\t");
    Serial.println(leftSpeed);
    rightSpeed = getSpeed(rightLight, rightOutOfLine, rightSumError, rightLastError);
    leftSpeed = getSpeed(leftLight, leftOutOfLine, leftSumError, leftLastError);
    
    if(rightSpeed < 0){ rightSpeed *= -1; rightDirection = BACKWARD; }
    else { rightDirection = FORWARD; }
    if(leftSpeed < 0){ leftSpeed *= -1; leftDirection = BACKWARD; }
    else { leftDirection = FORWARD; }
    
    motors.rightDrive(rightSpeed, rightDirection);
    motors.leftDrive(leftSpeed, leftDirection);
  }
}
