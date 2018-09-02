//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Wire.h>

//COTS libraries located in AVC-Arduino repo
#include <EnableInterrupt.h>
#include <NewPing.h>
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <PID_v1.h>

//Custom libraries located in AVC-Arduino repo
#include <Driving.h>
#include <Odometry.h>
#include <RC.h>


////////////////
////Settings////
////////////////

//Driving (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input
double rightSpeedMeasured, rightSpeedDesired, rightMotorPwm = 0; //PID variables for right-side motors
double leftSpeedMeasured, leftSpeedDesired, leftMotorPwm = 0; //PID variables for left-side motors
float proportionalGain = 100;
float integralGain = 800;
float derivativeGain = 0;
int pidCycleTime = 10; //length of time for one PID cycle in milliseconds
unsigned long lastPidComputeTime;

//Odometry (3200 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;

//Remote Control (RC)
byte linearPin = 9;
byte angularPin = 14;

//Robot Specifications
float wheelBase = .278; //in m
float wheelDiameter = .122; //in m
float maxLinearVelocity = 1.66; //in m/s
float maxAngularVelocity = maxLinearVelocity / (wheelBase / 2.0); //in rad/s
int cpr = 3200; //"cycles per revolution" -- number of encoder increments per one wheel revolution
inline float rightSkidSteerKinematics(float linearVelocity, float angularVelocity) { return linearVelocity + (angularVelocity * (wheelBase / 2.0)); }
inline float leftSkidSteerKinematics(float linearVelocity, float angularVelocity) { return linearVelocity - (angularVelocity * (wheelBase / 2.0)); }

//Serial (USB <--> Intel NUC)
String rxBuffer;
unsigned long watchdogTimer = 200; //fail-safe in case of communication link failure (in ms)
unsigned long lastDriveCommandTime = 0; //time of last communication from NUC (in ms)

//Ultrasound (Ping))))
byte leftSignal = 4;
byte centerSignal = 5;
byte rightSignal = 6;


////////////////////////////
////Class Instantiations////
////////////////////////////

Driving drive(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
L3G gyroscope;
LSM303 magnetometer_accelerometer;
LPS pressure;
NewPing leftUS(leftSignal, leftSignal, 330);
NewPing centerUS(centerSignal, centerSignal, 330);
NewPing rightUS(rightSignal, rightSignal, 330);
Odometry odom(rightEncoderA, rightEncoderB, leftEncoderA, leftEncoderB, wheelBase, wheelDiameter, cpr);
PID rightMotorPid(&rightSpeedMeasured, &rightMotorPwm, &rightSpeedDesired, proportionalGain, integralGain, derivativeGain, DIRECT);
PID leftMotorPid(&leftSpeedMeasured, &leftMotorPwm, &leftSpeedDesired, proportionalGain, integralGain, derivativeGain, DIRECT);
RC rc(linearPin, maxLinearVelocity, angularPin, maxAngularVelocity);


/////////////
////Setup////
/////////////

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  if (imuStatus()) {
    imuInit();
  }

  rxBuffer = "";

  rightMotorPid.SetMode(AUTOMATIC);
  rightMotorPid.SetOutputLimits(-255, 255);
  rightMotorPid.SetSampleTime(pidCycleTime);
  leftMotorPid.SetMode(AUTOMATIC);
  leftMotorPid.SetOutputLimits(-255, 255);
  leftMotorPid.SetSampleTime(pidCycleTime);
  lastPidComputeTime = micros();
}


/////////////////
////Main Loop////
/////////////////

void loop() {
  if ((abs(rc.scaledCommand1()) > 0.2 * maxLinearVelocity) || (abs(rc.scaledCommand2()) > 0.2 * maxAngularVelocity)) {
    //negate angular velocity command to match right-hand rule
    rightSpeedDesired = rightSkidSteerKinematics(rc.scaledCommand1(), -rc.scaledCommand2());
    leftSpeedDesired = leftSkidSteerKinematics(rc.scaledCommand1(), -rc.scaledCommand2());
    lastDriveCommandTime = millis();
    //block drive commands from serial connection while RC is active
    rc.setActive();
    //enable PIDs to control motor speeds
    rightMotorPid.SetMode(AUTOMATIC);
    leftMotorPid.SetMode(AUTOMATIC);
  }
  else if (rc.isActive()) {
    rc.resetActive();
  }

  if ((micros() - lastPidComputeTime) >= (pidCycleTime * 1000)) {
    odom.update();
    rightSpeedMeasured = odom.vr;
    leftSpeedMeasured = odom.vl;
    rightMotorPid.Compute();
    leftMotorPid.Compute();
    drive.drive(leftMotorPwm, rightMotorPwm);
    lastPidComputeTime = micros();
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',' || c == '\n') {
      parse();
      rxBuffer = "";
    }
    else if (c > 0) {
      rxBuffer += c;
    }
  }
  
  if (millis() - lastDriveCommandTime > watchdogTimer) {
    //disable PIDs, manually set motor speeds to 0
    rightMotorPid.SetMode(MANUAL);
    leftMotorPid.SetMode(MANUAL);
    rightMotorPwm = leftMotorPwm = 0;
  }
}


////////////////////////
//Parse receive buffer//
////////////////////////

void parse() {
  if (rxBuffer == "v") {
    float linearVelocity = Serial.parseFloat();
    float angularVelocity = Serial.parseFloat();
    if (!rc.isActive()) {
      //only respond to drive commands from serial connection if not under RC
      rightSpeedDesired = rightSkidSteerKinematics(linearVelocity, angularVelocity);
      leftSpeedDesired = leftSkidSteerKinematics(linearVelocity, angularVelocity);
      lastDriveCommandTime = millis();
      //enable PIDs to control motor speeds
      rightMotorPid.SetMode(AUTOMATIC);
      leftMotorPid.SetMode(AUTOMATIC);
    }
  }
  else if (rxBuffer == "d") {
    Serial.print("IMU,");
    bool imuStatusFlag = imuStatus();
    Serial.print(String(imuStatusFlag) + ",");
    if (imuStatusFlag) {
      imuInit();
      Serial.println(updateIMU());
    }
    else {
      Serial.println(",,,,,,,,");
    }

    Serial.println("ODOM," + String(1) + "," + updateOdom());

    Serial.print("USL,");
    int leftUSValue = leftUS.ping_cm();
    Serial.print(String(leftUSValue > 0 ? 1 : 0) + ",");
    if (leftUSValue > 0) {
      Serial.println(String(leftUSValue));
    }
    else {
      Serial.println();
    }

    Serial.print("USC,");
    int centerUSValue = centerUS.ping_cm();
    Serial.print(String(centerUSValue > 0 ? 1 : 0) + ",");
    if (centerUSValue > 0) {
      Serial.println(String(centerUSValue));
    }
    else {
      Serial.println();
    }

    Serial.print("USR,");
    int rightUSValue = rightUS.ping_cm();
    Serial.print(String(rightUSValue > 0 ? 1 : 0) + ",");
    if (rightUSValue > 0) {
      Serial.println(String(rightUSValue));
    }
    else {
      Serial.println();
    }
  }
}


//////////////////////////
//Update transmit buffer//
//////////////////////////

String updateIMU() {
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  if (!gyroscope.timeoutOccurred() && !magnetometer_accelerometer.timeoutOccurred()) {
    //Collect updated values
    LSM303::vector<int16_t> acc = magnetometer_accelerometer.a;
    L3G::vector<int16_t> gyro = gyroscope.g;
    LSM303::vector<int16_t> mag = magnetometer_accelerometer.m;

    //Convert accelerometer digits to milligravities, then to gravities, and finally to meters per second squared
    LSM303::vector<float> linear_acceleration = {acc.y*0.061/1000*9.81, -acc.x*0.061/1000*9.81, acc.z*0.061/1000*9.81};

    //Convert gyroscope digits to millidegrees per second, then to degrees per second, and finally to radians per second
    L3G::vector<float> angular_velocity = {gyro.y*8.75/1000*(PI/180), -gyro.x*8.75/1000*(PI/180), gyro.z*8.75/1000*(PI/180)};

    //Combine normalized magnetometer and accelerometer digits to produce Euler angles, i.e. pitch, roll, and yaw
    LSM303::vector<float> orientation = {(float)mag.x, (float)mag.y, (float)mag.z};
    orientation.x -= (magnetometer_accelerometer.m_min.x + magnetometer_accelerometer.m_max.x) / 2;
    orientation.y -= (magnetometer_accelerometer.m_min.y + magnetometer_accelerometer.m_max.y) / 2;
    orientation.z -= (magnetometer_accelerometer.m_min.z + magnetometer_accelerometer.m_max.z) / 2;
    LSM303::vector_normalize(&orientation);
    float roll = atan2(linear_acceleration.y, sqrt(pow(linear_acceleration.x,2) + pow(linear_acceleration.z,2)));
    float pitch = -atan2(linear_acceleration.x, sqrt(pow(linear_acceleration.y,2) + pow(linear_acceleration.z,2)));
    float yaw = atan2(-orientation.y*cos(roll) + orientation.z*sin(roll), orientation.x*cos(pitch) + orientation.y*sin(pitch)*sin(roll) + orientation.z*sin(pitch)*cos(roll)) + PI;
    orientation = {roll, pitch, yaw};

    //Append data to buffer
    String txBuffer = String(linear_acceleration.x) + "," +
               String(linear_acceleration.y) + "," +
               String(linear_acceleration.z) + "," +
               String(angular_velocity.x) + "," +
               String(angular_velocity.y) + "," +
               String(angular_velocity.z) + "," +
               String(orientation.x) + "," +
               String(orientation.y) + "," +
               String(orientation.z);

    return txBuffer;
  }

  return "";
}

String updateOdom() {
  String txBuffer;

  txBuffer = String(odom.x) + "," +
             String(odom.y) + "," +
             String(odom.theta) + "," +
             String(odom.vx) + "," +
             String(odom.vy) + "," +
             String(odom.vtheta);

  return txBuffer;
}


////////////////////////////
////Initializer Functions///
////////////////////////////

//Initialize gyroscope, accelerometer, magnetometer, and pressure gauge
void imuInit() {
  gyroscope.init();
  gyroscope.enableDefault();
  gyroscope.setTimeout(1);

  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();
  magnetometer_accelerometer.m_min = (LSM303::vector<int16_t>){ -2247,  -2068,  -1114};
  magnetometer_accelerometer.m_max = (LSM303::vector<int16_t>){+3369,  +2877,  +3634};
  magnetometer_accelerometer.setTimeout(1);

  pressure.init();
  pressure.enableDefault();
}


////////////////////////////
////Diagnostic Functions////
////////////////////////////

//Check for valid I2C connection to verify IMU
bool imuStatus() {
  byte numberOfDevices = 0;

  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (!error) {
      numberOfDevices++;
    }
  }

  if (numberOfDevices > 0) {
    return true;
  }
  else {
    return false;
  }
}
