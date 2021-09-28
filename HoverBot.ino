#include "config.h"
#include <Wire.h>
#include <Metro.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ODriveArduino.h>

int motorsActive = 0; // motor state
bool tilt_limit_exceeded = false; // motor desired state

Adafruit_BNO055 bno = Adafruit_BNO055(); // instantiate IMU
ODriveArduino odrive(Serial2); // instantiate ODrive

Metro ledMetro = Metro(BLINK_INTERVAL);
Metro controllerMetro = Metro(CONTROLLER_INTERVAL);
Metro activationMetro = Metro(ACTIVATION_INTERVAL);
int xAxis=140;
int yAxis=140;

char command;

int bt_throttle = 0;
int bt_steering = 0;


void setup() {
  pinMode(LEDPIN, OUTPUT);  
  Serial2.begin(BAUDRATE_ODRIVE); // ODrive uses 115200 baud

  Serial.begin(BAUDRATE_PC); // Serial to PC
  Serial1.begin(BAUDRATE_BT);

  // IMU
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1); // halt for safety
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
}

void loop() {
  controlTask();
  activationTask();
  blinkTask();

//  if(Serial.available()>=2 ){
//    xAxis = Serial.readString().toInt();
//    yAxis = Serial.readString().toInt(); 
//    Serial.print("xAxis:"); Serial.print(xAxis);
//    Serial.print("yAxis:"); Serial.print(yAxis);
//    Serial.print(xAxis);Serial.print(",");
//    Serial.print(yAxis);Serial.print(",");
    //Serial.println(xAxis);
//  }
  
  while(Serial1.available() > 0){ // Checks whether data is comming from the serial port
      command = Serial1.read();
      Serial.println(command);
      
    switch(command)
    {
      case 'N':
        {
          Serial.println('N');
          xAxis = 140;
          yAxis = 140;
          break;
         
        }
       case 'F':
       {
          yAxis = 180;
          Serial.println(yAxis);
          break;
       }

       case 'B':
       {
          yAxis = 80;
          Serial.println(yAxis);

          break;
       }
       case 'L':
       {
          xAxis = 110;
          Serial.println(xAxis);
          break;
       }
       case 'R':
       {
          xAxis = 170;
          Serial.println(xAxis);

          break;
       }
    }
}

if (xAxis > 130 && xAxis <150 && yAxis > 130 && yAxis <150){
  Stop();//joystick centered do nothing
  //Serial.print  ("do nothing");
  }

if (yAxis > 130 && yAxis <150){    //y is cetered 

if (xAxis < 130){//joystick left
  Serial.println("left");

  turnLeft(); 
}
Serial.println(xAxis);
if (xAxis > 150) { //joystick right
  Serial.println("right");

  turnRight();
}

}

else{

if (xAxis > 130 && xAxis <150){  //x centered 

if (yAxis < 130){
  Serial.println("forward");

  forward();
}
if (yAxis > 150){
  Serial.println("backword");
  backword();
}

}
else{
  if (yAxis < 130){
  //Serial.print  ("yAxis < 130");
  forward();
  }
  if (yAxis > 150){
  //Serial.print  ("yAxis > 150");
  backword();
  }
  
  if (xAxis < 130){
    //Serial.print  ("xAxis < 130");
    turnLeft(); 
    }
 
  if (xAxis > 150){
    //Serial.print  ("xAxis > 150");
    turnRight();
  }
 } 
}
}



void controlTask() {
  if (controllerMetro.check()) {
    motionController();
  }
}
void activationTask() {
  //Serial.println("activationMetro");
  if (activationMetro.check()) {
    modeSwitch(!tilt_limit_exceeded);
  }
}

void blinkTask() {
  //Serial.println("blinkTask");
  if (ledMetro.check()) {
    digitalWrite(LEDPIN, !digitalRead(LEDPIN));
  }
}
void forward()
{
  bt_throttle= yAxis;
  bt_steering = 140;
  motionController();
}

void backword()
{
  bt_throttle = yAxis;
  bt_steering = 140;
  motionController();
}

void turnLeft()
{
  bt_steering = xAxis;
  bt_throttle = 140;
  motionController();
}

void turnRight()
{
  bt_steering = xAxis;
  bt_throttle = 140;
  motionController();
}

void Stop()
{
  bt_throttle = 140;
  bt_steering = 140;
  motionController();
}

void motionController() {
  // IMU sampling
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  //Serial.print  ("euler:       "); Serial.println(euler);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //Serial.print  ("gyro:       "); Serial.println(gyro);
  
  if (abs(euler.z()) > TILT_LIMIT){
    tilt_limit_exceeded = true;
  }
  else{
    tilt_limit_exceeded = false;
  }

  // balance controller
  float balanceControllerOutput = euler.z() * KP_BALANCE + gyro.x() * KD_BALANCE;
  
  //Serial.print  ("balanceControllerOutput:       "); Serial.println(balanceControllerOutput);
  //Serial.print  ("euler.z:       "); Serial.println(euler.z());
  //Serial.print  ("gyro.x:       "); Serial.println(gyro.x());
  //float steeringControllerOutput = gyro.z() * KD_ORIENTATION; 
  
  float positionControllerOutput =  KP_POSITION * (bt_throttle - PWM_CENTER);
  float steeringControllerOutput = KP_STEERING * (bt_steering - PWM_CENTER) + gyro.z() * KD_ORIENTATION;  
  //Serial.print  ("bt_throttle:       "); Serial.println(bt_throttle);
  //Serial.print  ("bt_steering:       "); Serial.println(bt_steering);
  
  float controllerOutput_right = balanceControllerOutput + positionControllerOutput + steeringControllerOutput;
  float controllerOutput_left  =balanceControllerOutput + positionControllerOutput - steeringControllerOutput;
  
  odrive.SetCurrent(0, MOTORDIR_0 * controllerOutput_right);
  odrive.SetCurrent(1, MOTORDIR_1 * controllerOutput_left);
  //Serial.print  ("controllerOutput_left:       "); Serial.println(controllerOutput_left);
  //Serial.print  ("controllerOutput_right:       "); Serial.println(controllerOutput_right);

}

void modeSwitch(int mode_desired) {
  //Serial.println("modeSwitch2");
//  if (mode_desired == motorsActive) {
 // }
 // else {
    if (mode_desired == 1) {
      int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial.println("Engaging motors");
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, false);
    }
    else if (mode_desired == 0) {
      int requested_state = ODriveArduino::AXIS_STATE_IDLE;
      Serial.println("Disengaging motors");
      odrive.run_state(0, requested_state, false);
      odrive.run_state(1, requested_state, false);
    }
    else {
      Serial.println("Invalid mode selection");
    }
    motorsActive = mode_desired;
 // }
  return;
}

  
