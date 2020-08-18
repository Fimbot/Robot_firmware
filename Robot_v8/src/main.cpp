#include <Arduino.h>
#include <MadgwickAHRS.h>

// Serial CODE init
String inputString1 = "";
String inputString2 = "";
String inputString3 = "";
String sendStrig = "";

//HardwareSerial Serial1(USART1);
HardwareSerial Serial2(USART2); // or HardWareSerial Serial2 (PA3, PA2);
HardwareSerial Serial3(USART3);

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)

void initSerial()
{

  inputString1.reserve(64);
  inputString2.reserve(64);
  inputString3.reserve(64);

  Serial.begin(9600);
  Serial1.begin(9600);              // on PA9 PA10
  Serial2.begin(HOVER_SERIAL_BAUD); // on PA3 PA2
  Serial3.begin(9600);              // on PB11 PB10

  Serial1.println("Serial1:");
  Serial2.println("Serial2:");
  Serial3.println("Serial3:");
}

#include "MPU9250.h" // IMU CODE
// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
//SPIClass mySPI(1);
MPU9250 IMU(SPI, PA4);
int status;
float pitch;
float roll;
float yaw;
int timerIMU;
bool existIMU;

void initIMU()
{
  // start communication with IMU
  status = IMU.begin();
  if (status < 0)
  {
    Serial.println("IMU ERROR");
    Serial.print("Status: ");
    Serial.println(status);
    //while(1) {}
    existIMU = false;
  }
  else
  {
    existIMU = true;

    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

    //IMU.calibrateAccel();
  }
}

#include <VL53L0X.h> // SENZOR CODE
VL53L0X Sensor1;
VL53L0X Sensor2;
#define SENZOR1_ADDRESS 0x30
#define SENZOR2_ADDRESS 0x34
// set the pins to shutdown
#define SHT_SENZOR1 PB8
#define SHT_SENZOR2 PB9
int senzor1;
int senzor2;
bool existSenzor1;
bool existSenzor2;

void initSenzor()
{
  Wire.begin();

  pinMode(SHT_SENZOR1, OUTPUT);
  pinMode(SHT_SENZOR2, OUTPUT);
  // all reset
  digitalWrite(SHT_SENZOR1, LOW);
  digitalWrite(SHT_SENZOR2, LOW);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_SENZOR1, HIGH);
  digitalWrite(SHT_SENZOR2, LOW);
  delay(10);
  Sensor1.setAddress(SENZOR1_ADDRESS);
  Sensor1.setTimeout(200);
  if (!Sensor1.init())
  {
    Serial.println("ERROR VL53L0X Sensor1 !");
    // while (1) {}
    existSenzor1 = false;
  }
  else
  {
    existSenzor1 = true;
    /* // lower the return signal rate limit (default is 0.25 MCPS)
    Sensor1.setSignalRateLimit(0.1);
    //increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //Serial.print(Sensor1.getAddress());
    //Serial.println("Senzor1 I2C adresa");*/
    Sensor1.startContinuous();
  }

  delay(10);
  // activating LOX2 and reseting LOX1
  digitalWrite(SHT_SENZOR1, HIGH);
  digitalWrite(SHT_SENZOR2, HIGH);
  delay(10);
  Sensor2.setAddress(SENZOR2_ADDRESS);
  Sensor2.setTimeout(200);
  if (!Sensor2.init())
  {
    Serial.println("ERROR VL53L0X Sensor2 !");
    //while (1) {}
    existSenzor2 = false;
  }
  else
  {
    existSenzor2 = true;

    /*// lower the return signal rate limit (default is 0.25 MCPS)
    Sensor2.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // Serial.print(Sensor2.getAddress());
    // Serial.println("Senzor2 I2C adresa");*/

    Sensor2.startContinuous();
  }
}

#define SERIAL_BAUD 115200 // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xAAAA // [-] Start frme definition for reliable serial communication
#define TIME_SEND 100      // [ms] Sending time interval
#define SPEED_MAX_TEST 300 // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

//HardwareSerial HoverSerial(USART2);   // or HardWareSerial Serial2 (PA3, PA2);

// Global variables
long last_send = 0;

// Global variables
uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct
{
  uint16_t start;
  int16_t speedLeft;
  int16_t speedRight;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct
{

  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedL;
  int16_t speedR;
  int16_t speedL_meas;
  int16_t speedR_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  int16_t curL_DC;
  int16_t curR_DC;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

#include "Ticker.h"   // Timer code
void mySerialEvent(); //Def timer funkcion
void imuData();
void senzorData();
void driveMotor();
void sendData();

Ticker timer1(mySerialEvent, 1000, 0, MICROS);
Ticker timer2(imuData, 20, 0, MILLIS);
Ticker timer3(senzorData, 35, 0, MILLIS);
Ticker timer4(driveMotor, 100, 0, MILLIS);
Ticker timer5(sendData, 150, 0, MILLIS);

#include <PID_v1.h>
//PID regulators
double yawPidSet, yawPidInp, yawPidOut;
//Specify the links and initial tuning parameters
double yawPidKp = 5.8, yawPidKi = 5, yawPidKd = 1.1;
PID yawPID(&yawPidInp, &yawPidOut, &yawPidSet, yawPidKp, yawPidKi, yawPidKd, DIRECT);

double pitchPidSet, pitchPidInp, pitchPidOut;
//Specify the links and initial tuning parameters
double pitchPidKp = 10, pitchPidKi = 0.5, pitchPidKd = 10;
PID pitchPID(&pitchPidInp, &pitchPidOut, &pitchPidSet, pitchPidKp, pitchPidKi, pitchPidKd, DIRECT);

void setup()
{ // Start inicializacion Program
  initSerial();
  initIMU();
  initSenzor();

  //tell the PID to range between 0 and the full window size
  yawPID.SetOutputLimits(-200, 200);
  yawPID.SetSampleTime(100);
  //turn the PID on
  yawPID.SetMode(AUTOMATIC);
  yawPidSet = yaw = 0.f;

  pitchPID.SetOutputLimits(-300, 300);
  pitchPID.SetSampleTime(100);
  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  pitchPidSet = yaw = 0.f;

  timer1.start();
  timer2.start();
  timer3.start();
  timer4.start();
  timer5.start();
}

int adStopPin = 0;
bool centralStop = true;
void stopPin()
{
  adStopPin = analogRead(A0);
  if (adStopPin > 250)
  {
    centralStop = false;
  }
  else
  {
    centralStop = true;
  }
}

// ########################## SEND ##########################
void SendSerial2(int16_t uSpeedLeft, int16_t uSpeedRight)
{
  // Create command
  Command.start = (uint16_t)START_FRAME;
  Command.speedLeft = (int16_t)uSpeedLeft;
  Command.speedRight = (int16_t)uSpeedRight;
  Command.checksum = (uint16_t)(Command.start ^ Command.speedLeft ^ Command.speedRight);

  // Write to Serial
  Serial2.write((uint8_t *)&Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void ReceiveSerial2()
{
  // Check for new data availability in the Serial buffer
  if (Serial2.available())
  {
    incomingByte = Serial2.read();                                      // Read the incoming byte
    bufStartFrame = ((uint16_t)(incomingBytePrev) << 8) + incomingByte; // Construct the start frame
  }
  else
  {
    return;
  }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
  Serial3.print(incomingByte);
  return;
#endif

  // Copy received data
  if (bufStartFrame == START_FRAME)
  { // Initialize if new data is detected
    p = (byte *)&NewFeedback;
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  }
  else if (idx >= 2 && idx < sizeof(SerialFeedback))
  { // Save the new received data
    *p++ = incomingByte;
    idx++;
  }

  // Check if we reached the end of the package
  if (idx == sizeof(SerialFeedback))
  {
    uint16_t checksum;
    checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR ^ NewFeedback.speedL ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.curL_DC ^ NewFeedback.curR_DC);

    // Check validity of the new data
    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
    {
      // Copy the new data
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
    }
    else
    {
      //Serial3.println("Non-valid data skipped");
    }
    idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
  }

  // Update previous states
  incomingBytePrev = incomingByte;
}

int motorL = 0;
int motorR = 0;
int robAngle = 0;
int motPoc = 5000; //off

void parametrsMotor(String input)
{
  if (input.length() > 4)
  {

    int pos1 = input.indexOf('L') + 1;
    int pos2 = input.indexOf(',');

    if (pos1 < pos2 && pos1 > 0)
    {
      motorL = (input.substring(pos1, pos2)).toInt();
      pos1 = input.indexOf('R') + 1;
      pos2 = input.length();
      if (pos1 < pos2 && pos1 > 0)
      {
        motorR = (input.substring(pos1, pos2)).toInt();
        motPoc = 0;
        motorR = -(motorR); //!!!
        motorL = -(motorL);
      }
    }
  }
}

bool timerMove(int steps){
  static int count = 0;
  if(steps>0){
    count=steps;
    return false;

  }else{
    if(count>0){
      count--;
      return false;
    }
    return true;
  }

}


void setMove(int motL, int motR, int angle )
{
  motorL = motL;
  motorR = motR;
  robAngle = angle;
  motPoc = 0;
  
}

void motionRobot(int type)
{

  static int step = 0;

  if (type >= 0)
    step = type;

  switch (step)
  {
  case 0:

    break;


case 100:
setMove(0, 0, 0);
timerMove(180);
step=105;
break;

 case 105:
if(senzor1<600 || senzor2<600){
 setMove(0,0,0);
}else{
  if (timerMove(-1)){
    step=110;
    timerMove(10);
    setMove(0,0,0);
    }else{
    setMove(110,110,0);

    }

  }
break;

 case 110:
  if (timerMove(-1)){
    step=120;
      timerMove(100);
      setMove(0,0,-178);    
  }
 break;

 case 120:
 if (timerMove(-1)){
    step=100;  
  }
 break;



  default:
    setMove(0, 0, 0);
    break;
  }
}

float messagePar[4];
void messageBody(String body)
{
  int pos1 = 0, pos2 = 0;
  bool end = false;
  if (body.length() == 0)
    end = true;
  if (body.charAt(0) == ';')
    pos1 = 1;

  for (int i = 0; i < 4; i++)
  {
    if (end)
    {
      messagePar[i] = 0;
    }
    else
    {
      pos2 = body.indexOf(';', pos1);
      if (pos2 > 0)
      {

        messagePar[i] = (body.substring(pos1, pos2)).toFloat();
        pos1 = pos2 + 1;
      }
      else
      {
        messagePar[i] = (body.substring(pos1, body.length())).toFloat();
        end = true;
      }
    }
  }
}

char questionHead = ' ';
String questionBody = "";
char commandHead = ' ';
String commandBody = "";
char hashtagHead = ' ';
String hashtagHeadBody = "";

void setParamters()
{
  messageBody(hashtagHeadBody);
  switch (hashtagHead)
  {

  case 'A':
    robAngle = constrain(messagePar[0], -360.0, 360.0);
    break;

  case 'M':
    motionRobot(constrain(messagePar[0], 0.0, 10000.0));
    break;

  case 'Y':
    yawPidKp = constrain(messagePar[0], 0.0, 50.0);
    yawPidKi = constrain(messagePar[1], 0.0, 50.0);
    yawPidKd = constrain(messagePar[2], 0.0, 50.0);
    yawPID.SetTunings(yawPidKp, yawPidKi, yawPidKd);

    break;
  case ' ':
    // statements
    break;
  default:
    // statements
    break;
  }
  hashtagHead = ' ';
  hashtagHeadBody = "";
}

void message(String input)
{
  input.trim();
  int mLength = input.length();
  if (mLength > 1)
  {

    switch (input.charAt(0))
    {
    case 'L':
      parametrsMotor(input);
      break;

    case '?':
      questionHead = input.charAt(1);
      if (mLength > 2)
      {
        questionBody = input.substring(2);
        questionBody.trim();
        //messageBody(questionBody);
      }
      else
      {
        questionBody = "";
      }
      break;

    case '!':
      commandHead = input.charAt(1);
      if (mLength > 2)
      {
        commandBody = input.substring(2);
        commandBody.trim();
        //messageBody(commandBody);
      }
      else
      {
        commandBody = "";
      }
      break;

    case '#':
      hashtagHead = input.charAt(1);
      if (mLength > 2)
      {
        hashtagHeadBody = input.substring(2);
        hashtagHeadBody.trim();
        setParamters();
      }
      else
      {
        hashtagHeadBody = "";
      }
      break;
    default:

      break;
    }
  }
}

void mySerialEvent1()
{
  while (Serial1.available() > 0)
  {
    char inChar = (char)Serial1.read();
    if (inChar == '\n' || inChar == '\r' || inputString1.length() > 32)
    {
      if (inputString1.length() > 0)
      {
        //Serial2.println(inputString1);
        message(inputString1);
        inputString1 = "";
      }
    }
    else
    {
      inputString1 += inChar;
    }
  }
}

void mySerialEvent2()
{
  while (Serial2.available() > 0)
  {
    char inChar = (char)Serial2.read();
    if (inChar == '\n' || inChar == '\r')
    {
      if (inputString2.length() > 0)
      {
        //Serial1.println(inputString2);
        //Serial3.println(inputString2);
        inputString2 = "";
      }
    }
    else
    {
      inputString2 += inChar;
    }
  }
}

void mySerialEvent3()
{
  while (Serial3.available() > 0)
  {
    char inChar = (char)Serial3.read();
    if (inChar == '\n' || inChar == '\r')
    {
      if (inputString3.length() > 0)
      {
        //Serial2.println(inputString3);
        message(inputString3);
        sendStrig = inputString3;
        inputString3 = "";
      }
    }
    else
    {
      inputString3 += inChar;
    }
  }
}

void mySerialEvent()
{
  mySerialEvent1();

  //mySerialEvent2();
  mySerialEvent3();
}

#define G 9.81;
float covertAccel(float accel_mss)
{
  return accel_mss / G;
}

float covertGyro(float gyro_rad)
{
  return gyro_rad * RAD_TO_DEG;
}
//Madgwick filter;

void imuData()
{

  stopPin(); //analog read central stop pin

  if (existIMU)
  {

    IMU.readSensor();
    if (timerIMU == 0)
    {
      timerIMU = micros();
      yaw = pitch = roll = 0.0f;
    }

    float timeStep = float(micros() - timerIMU) / 1000000.f;
    timerIMU = micros();
    yaw = yaw + (IMU.getGyroY_rads() * timeStep * RAD_TO_DEG);
    pitch = pitch + (IMU.getGyroX_rads() * timeStep * RAD_TO_DEG);
    roll = roll + (IMU.getGyroZ_rads() * timeStep * RAD_TO_DEG);

    /*

    float ax, ay, az;
    float gx, gy, gz;

    ax = covertAccel(IMU.getAccelX_mss());
    ay = covertAccel(IMU.getAccelY_mss());
    az = covertAccel(IMU.getAccelZ_mss());
    
    gx = covertGyro(IMU.getGyroX_rads());
    gy = covertGyro(IMU.getGyroY_rads());
    gz = covertGyro(IMU.getGyroZ_rads());

    gx =pitch;
    gy = yaw;
    gz = roll;

    filter.updateIMU(gx, gy, gz, ax, ay, az);
    */
  }
}

void senzorData()
{
  if (existSenzor1)
  {
    senzor1 = Sensor1.readRangeContinuousMillimeters();
    if (Sensor1.timeoutOccurred())
    {
      Serial.print("ERROR Senzor1");
     // existSenzor1 = false;
    }
  }

  if (existSenzor2)
  {
    senzor2 = Sensor2.readRangeContinuousMillimeters();
    if (Sensor2.timeoutOccurred())
    {
      Serial.print("ERROR Senzor2");
      //existSenzor2 = false;
    }
  }
}

#define AkcelMin 2 //
#define AkcelMAX 25
#define AkcelBrake 50
#define AkcelAngle 4
#define AkcelDiv 10
#define SenzorStop 200
#define AutoBrake 10    //10=1s
#define AutoBrakeYaw 50 //10=5s
void driveMotor()
{
  static int akMotL = 0, akMotR = 0, akcel = 0, akAngle = 0;

  motionRobot(-1);

  bool brakeSenzor = false;

  if (existSenzor1 && senzor1 < SenzorStop)
  {
    brakeSenzor = true;
  }
  if (existSenzor2 && senzor2 < SenzorStop)
  {
    brakeSenzor = true;
  }

  if (motPoc == 0)
  {
    akcel = (abs(motorL) + abs(motorR)) / AkcelDiv;
    if (akcel < AkcelMin)
      akcel = AkcelMin;
    if (akcel > AkcelMAX)
      akcel = AkcelMAX;
  }

  if (motPoc > AutoBrake || brakeSenzor) // auto brake 1s
  {
    motorL = 0;
    motorR = 0;
  }

  if (motPoc < 5000)
  {
    motPoc++;
  }

  if (motorL == 0 && motorR == 0)
    akcel = AkcelBrake;

  if (akMotL < motorL)
  {
    akMotL = akMotL + akcel;
    if (akMotL > motorL)
      akMotL = motorL;
  }

  if (akMotR < motorR)
  {
    akMotR = akMotR + akcel;
    if (akMotR > motorR)
      akMotR = motorR;
  }

  if (akMotL > motorL)
  {
    akMotL = akMotL - akcel;
    if (akMotL < motorL)
      akMotL = motorL;
  }

  if (akMotR > motorR)
  {
    akMotR = akMotR - akcel;
    if (akMotR < motorR)
      akMotR = motorR;
  }

  if (robAngle > 0)
  {
    if (robAngle <= AkcelAngle)
    {
      yawPidSet = yawPidSet + robAngle;
      robAngle = 0;
    }
    else
    {
      yawPidSet = yawPidSet + AkcelAngle;
      robAngle = robAngle - AkcelAngle;
    }
    motPoc = 0;
    akMotL = (akMotL + akMotR) / 2;
    akMotR = akMotL;
  }

  if (robAngle < 0)
  {
    if (robAngle >= -(AkcelAngle))
    {
      yawPidSet = yawPidSet - robAngle;
      robAngle = 0;
    }
    else
    {
      yawPidSet = yawPidSet - AkcelAngle;
      robAngle = robAngle + AkcelAngle;
    }
    motPoc = 0;
    akMotL = (akMotL + akMotR) / 2;
    akMotR = akMotL;
  }

  int regMotL = 0, regMotR = 0;

  yawPidInp = yaw;
  yawPID.Compute();

  yawPidInp = yaw;
  yawPID.Compute();
  if ((abs(akMotL - (akMotR)) < 10) && !brakeSenzor && motPoc < AutoBrakeYaw)
  {
    regMotL = akMotL + int(yawPidOut);
    regMotR = akMotR - int(yawPidOut);
  }
  else
  {
    yawPidSet = yaw;
    regMotL = akMotL;
    regMotR = akMotR;
  }

  /*
  pitchPidInp = pitch;
  pitchPID.Compute();
  if (abs(pitchAvg - (pitch)) > 10.f && pitchStop > 0)
  {
    regMotL = akMotL + pitchPidOut;
    regMotR = akMotR + pitchPidOut;
    pitchStop--;
    if (pitchStop == 0)
      pitchStop = -10;
  }
  else
  {
    if (pitchStop < 10)
      pitchStop++;
    pitchAvgSum += pitch;
    pitchAvgI++;
    if (pitchAvgI > 1000)
    {
      pitchAvg = pitchAvgSum / float(pitchAvgI);
      pitchAvgSum = 0.f;
      pitchAvgI = 0;
      pitchPidSet = pitchAvg;
    }
  }*/

  if (centralStop)
  {
    SendSerial2(0, 0);
  }
  else
  {
    SendSerial2(regMotL, regMotR);
  }
}

void sendData()
{
  switch (questionHead)
  {
  case 'M':
    // Print data to built-in Serial
    Serial3.print("cmd1 ");
    Serial3.print(Feedback.cmd1);
    Serial3.print("cmd2 ");
    Serial3.println(Feedback.cmd2);
    Serial3.print("speedR ");
    Serial3.print(Feedback.speedR);
    Serial3.print("speedL ");
    Serial3.println(Feedback.speedL);
    Serial3.print("speedR_m ");
    Serial3.print(Feedback.speedR_meas);
    Serial3.print("speedL_m");
    Serial3.println(Feedback.speedL_meas);
    Serial3.print("batVoltage ");
    Serial3.print(Feedback.batVoltage);
    Serial3.print("boardTemp ");
    Serial3.println(Feedback.boardTemp);
    break;

  case 'A':
    Serial3.print(" AccelX=");
    Serial3.print(IMU.getAccelX_mss(), 4);
    Serial3.print(" AccelY=");
    Serial3.print(IMU.getAccelY_mss(), 4);
    Serial3.print(" AccelZ=");
    Serial3.println(IMU.getAccelZ_mss(), 4);
    break;

  case 'G':
    Serial3.print(" GyroX=");
    Serial3.print(IMU.getGyroX_rads(), 4);
    Serial3.print(" GyroY=");
    Serial3.print(IMU.getGyroY_rads(), 4);
    Serial3.print(" GyroZ=");
    Serial3.println(IMU.getGyroZ_rads(), 4);
    break;

  case 'C':
    Serial3.print(" MagX=");
    Serial3.print(IMU.getMagX_uT(), 2);
    Serial3.print(" MagY=");
    Serial3.print(IMU.getMagY_uT(), 2);
    Serial3.print(" MagZ=");
    Serial3.println(IMU.getMagZ_uT(), 2);
    break;

  case 'I':
    Serial3.print(" Pitch = ");
    Serial3.print(pitch);
    Serial3.print(" Roll = ");
    Serial3.print(roll);
    Serial3.print(" Yaw = ");
    Serial3.println(yaw);
    break;

  case 'F':
    Serial3.print(" Stop pin= ");
    Serial3.println(adStopPin);
    break;

  case 'S':
    Serial3.print(" Senzor1= ");
    Serial3.print(senzor1);
    Serial3.print(" Senzor2= ");
    Serial3.println(senzor2);
    break;

  case ' ':
    break;

  default:
    Serial3.println(" ERROR ");
    questionHead = ' ';
    break;
  }
  if (questionBody.equals(""))
    questionHead = ' ';
}

void loop()
{ // Repeat loop forever
  timer1.update();
  timer2.update();
  timer1.update();
  timer3.update();
  timer1.update();
  timer4.update();
  timer5.update();
  ReceiveSerial2();
}