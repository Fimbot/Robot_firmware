//robot control projekt c Jiri Suryn 2021
#include <Arduino.h>
#include "MPU9250.h" // IMU CODE
#include <VL53L0X.h> // SENZOR CODE
#include "Ticker.h"
#include <PID_v1.h>
#include <Kalman.h>

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

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
//SPIClass mySPI(1);
MPU9250 IMU(SPI, PA4);
int status;
float pitch2;
float roll2;
float yaw2;
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
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

    //IMU.calibrateAccel();
  }
}

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
    // lower the return signal rate limit (default is 0.25 MCPS)
    Sensor1.setSignalRateLimit(0.1);
    //increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    //Serial.print(Sensor1.getAddress());
    //Serial.println("Senzor1 I2C adresa");
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

    // lower the return signal rate limit (default is 0.25 MCPS)
    Sensor2.setSignalRateLimit(0.1);
    // increase laser pulse periods (defaults are 14 and 10 PCLKs)
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    Sensor2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    // Serial.print(Sensor2.getAddress());
    // Serial.println("Senzor2 I2C adresa");
    Sensor1.startContinuous();
    Sensor2.startContinuous();
  }
}

#define SERIAL_BAUD 115200 // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xAAAA // [-] Start frme definition for reliable serial communication
#define TIME_SEND 50      // [ms] Sending time interval
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

// Timer code
void mySerialEvent(); //Def timer funkcion
void imuData();
void senzorData();
void driveMotor();
void sendData();

Ticker timer1(mySerialEvent, 1000, 0, MICROS);
Ticker timer2(imuData, 5, 0, MILLIS);
Ticker timer3(senzorData, 35, 0, MILLIS);
Ticker timer4(driveMotor, 25, 0, MILLIS);
Ticker timer5(sendData, 500, 0, MILLIS);

bool pitchAvgEnd =false; // Stop AVG set PitchPID
//PID regulators
double yawPidSet, yawPidInp, yawPidOut;
//Specify the links and initial tuning parameters
//double yawPidKp = 6, yawPidKi = 5, yawPidKd = 1.5;
double yawPidKp = 0.0, yawPidKi = 0.0, yawPidKd = 0.0;
PID yawPID(&yawPidInp, &yawPidOut, &yawPidSet, yawPidKp, yawPidKi, yawPidKd, DIRECT);

double pitchPidSet, pitchPidInp, pitchPidOut;
//Specify the links and initial tuning parameters
double pitchPidKp = 3.0, pitchPidKi = 0.0, pitchPidKd = 1.5;
PID pitchPID(&pitchPidInp, &pitchPidOut, &pitchPidSet, pitchPidKp, pitchPidKi, pitchPidKd, DIRECT);

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanZ;

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
float accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle, gyroZangle; // Angle calculate using the gyro only
double compAngleX, compAngleY, compAngleZ; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY, kalAngleZ;    // Calculated angle using a Kalman filter

uint32_t timer;
void setup()
{ // Start inicializacion Program
  initSerial();
  initIMU();
  initSenzor();

  //tell the PID to range between 0 and the full window size
  yawPID.SetOutputLimits(-300, 300);
  yawPID.SetSampleTime(100);
  //turn the PID on
  yawPID.SetMode(AUTOMATIC);
  yawPidSet = yaw2 = 0.f;

  pitchPID.SetOutputLimits(-300, 300);
  pitchPID.SetSampleTime(100);
  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  pitchPidSet = yaw2 = 0.f;

  timer1.start();
  timer2.start();
  timer3.start();
  timer4.start();
  timer5.start();

  IMU.readSensor();

  accX = IMU.getAccelX_mss();
  accY = IMU.getAccelY_mss();
  accZ = IMU.getAccelZ_mss();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = -atan2(accX , accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  pitchPidSet = roll;
  kalmanY.setAngle(pitch);
  kalmanZ.setAngle(yaw);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = yaw;
  compAngleX = roll;
  compAngleY = pitch;
  compAngleZ = yaw;

  timer = micros();
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
int motPoc = 0;

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

  case 'Y':
    yawPidKp = constrain(messagePar[0], 0.0, 50.0);
    yawPidKi = constrain(messagePar[1], 0.0, 50.0);
    yawPidKd = constrain(messagePar[2], 0.0, 50.0);
    yawPID.SetTunings(yawPidKp, yawPidKi, yawPidKd);
    Serial3.println("#Y OK");

    break;

    case 'P':
    pitchPidKp = constrain(messagePar[0], 0.0, 50.0);
    pitchPidKi = constrain(messagePar[1], 0.0, 50.0);
    pitchPidKd = constrain(messagePar[2], 0.0, 50.0);
    pitchPID.SetTunings(pitchPidKp, pitchPidKi, pitchPidKd);
     Serial3.println("#P OK");
    break;

    case 'R':
    pitchAvgEnd =false;

    Serial3.println("#R OK");
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

#define Gravity 9.81;
float covertAccel(float accel_mss)
{
  return accel_mss / Gravity;
}

float covertGyro(float gyro_rad)
{
  return gyro_rad * RAD_TO_DEG;
}

float zeroAccelX(float accel, bool storage)
{
  static float accel_storage = accel;
  if (storage)
    accel_storage = accel;
  return accel - accel_storage;
}

float zeroAccelY(float accel, bool storage)
{
  static float accel_storage = accel;
  if (storage)
    accel_storage = accel;
  return accel - accel_storage;
}

float zeroAccelZ(float accel, bool storage)
{
  static float accel_storage = accel;
  if (storage)
    accel_storage = accel;
  return accel - accel_storage;
}

void filterKalman()
{
  /* Update all the values */
  accX = IMU.getAccelX_mss();
  accY = IMU.getAccelY_mss();
  accZ = IMU.getAccelZ_mss();

  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  double yaw = -atan2(accX , accY) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
  {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  }
  else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  //if (abs(kalAngleX) > 90)
    //gyroZrate = -gyroZrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleZ = kalmanZ.getAngle(yaw, gyroZrate, dt);

#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90))
  {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  }
  else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate;                          // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  compAngleZ = 0.93 * (compAngleZ + gyroZrate * dt) + 0.07 * yaw;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  if (gyroZangle < -180 || gyroZangle > 180)
    gyroZangle = kalAngleZ;

    /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");


  Serial3.print(roll);
  Serial.print("\t");
  Serial3.print(gyroXangle);
  Serial.print("\t");
  Serial3.print(compAngleX);
  Serial.print("\t");
  Serial3.print(kalAngleX);
  Serial.print("\t");

  Serial3.print("\t");

  Serial3.print(pitch);
  Serial.print("\t");
  Serial3.print(gyroYangle);
  Serial.print("\t");
  Serial3.print(compAngleY);
  Serial.print("\t");
  Serial3.print(kalAngleY);
  Serial.print("\t");

 // Set to 1 to print the temperature
  Serial.print("\t");

  double temperature = (double)tempRaw / 340.0 + 36.53;
  Serial.print(temperature); Serial.print("\t");
  Serial3.print("\r\n");
  //delay(2);
#endif

  
}

void imuData()
{
  if (existIMU)
  {

    IMU.readSensor();
    if (timerIMU == 0)
      timerIMU = micros();
    float timeStep = float(micros() - timerIMU) / 1000000.f;
    timerIMU = micros();
    yaw2 = yaw2 + (IMU.getGyroY_rads() * timeStep * RAD_TO_DEG);
    pitch2 = pitch2 + (IMU.getGyroX_rads() * timeStep * RAD_TO_DEG);
    roll2 = roll2 + (IMU.getGyroZ_rads() * timeStep * RAD_TO_DEG);

    filterKalman();
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
    }
  }

  if (existSenzor2)
  {
    senzor2 = Sensor2.readRangeContinuousMillimeters();
    if (Sensor2.timeoutOccurred())
    {
      Serial.print("ERROR Senzor2");
    }
  }
}

#define AkcelMin 2 //
#define AkcelMAX 50
#define AkcelBrake 30
#define AkcelDiv 10
#define AutoBrake 10 //10=1s
void driveMotor()
{
  static int akMotL = 0, akMotR = 0, akcel = 0, pitchAvgI = 0, pitchStop = 0;

  static float pitchAvg = 0.f, pitchAvgSum = 0.f;

  

  if(!pitchAvgEnd){
    if(pitchAvgI<20){
    pitchAvgSum +=kalAngleX;
    pitchAvgI++;
    }else{
      pitchAvg=pitchAvgSum/float(pitchAvgI);
       pitchPidSet = pitchAvg;
      pitchAvgEnd=true;
      pitchAvgSum=0.f;
      pitchAvgI=0;
    }

 }
  

  if (motPoc == 0)
  {
    akcel = (abs(motorL) + abs(motorR)) / AkcelDiv;
    if (akcel < AkcelMin)
      akcel = AkcelMin;
    if (akcel > AkcelMAX)
      akcel = AkcelMAX;
  }

  if (motPoc < AutoBrake) // auto brake 1s
  {
    motPoc++;
  }
  else
  {
    motorL = 0;
    motorR = 0;
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

  int regMotL = 0, regMotR = 0;

  yawPidInp = yaw2;
  yawPID.Compute();
  if (abs(akMotL - (akMotR)) < 10)
  {
    regMotL = akMotL + int(yawPidOut);
    regMotR = akMotR - int(yawPidOut);
  }
  else
  {
    yawPidSet = yaw2;
    regMotL = akMotL;
    regMotR = akMotR;
  }

  pitchPidInp = kalAngleX;
  pitchPID.Compute();
  
    regMotL = regMotL + pitchPidOut;
    regMotR = regMotR  + pitchPidOut;

  SendSerial2(regMotL, regMotR);
}

void sendData()
{
  switch (questionHead)
  {
  case 'M':
    // Print data to built-in Serial
    Serial3.print("cmd1=");
    Serial3.print(Feedback.cmd1);
    Serial3.print(" cmd2=");
    Serial3.println(Feedback.cmd2);
    Serial3.print("speedR=");
    Serial3.print(Feedback.speedR);
    Serial3.print(" speedL=");
    Serial3.println(Feedback.speedL);
    Serial3.print("speedR_m=");
    Serial3.print(Feedback.speedR_meas);
    Serial3.print(" speedL_m=");
    Serial3.println(Feedback.speedL_meas);
    Serial3.print("batVolt=");
    Serial3.print(Feedback.batVoltage);
    Serial3.print(" bTemp=");
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
    Serial3.print(pitch2);
    Serial3.print(" Roll = ");
    Serial3.print(roll2);
    Serial3.print(" Yaw = ");
    Serial3.println(yaw2);
    break;

  case 'F':
    Serial3.print(" PitchX=");
    Serial3.print(kalAngleX, 4);
    Serial3.print(" RollY=");
    Serial3.print(kalAngleY, 4);
    Serial3.print(" YawZ=");
    Serial3.println(kalAngleZ, 4);

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
  //questionHead = ' ';
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