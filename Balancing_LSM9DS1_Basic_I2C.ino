#include <Arduino.h>
/*
The LSM9DS1 is a versatile 9DOF sensor. It has a built-in
SFE_LSM9DS1 library. It'll demo the following:
* How to create a LSM9DS1 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS1 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed,
  magnetic field strength using the calcAccel(), calcGyro()
  and calcMag() functions.
* How to use the data from the LSM9DS1 to calculate
  orientation and heading.
  */

#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M	0x1C // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6A // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////

//#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 16.348
// #define GYROSCOPE_SENSITIVITY 65.536
// 2000*2/2^16

#define M_PI 3.14159265359
#define dt (10.0/1000.0)             // 100hz = 10ms

#define RMotor_offset 30 // The offset of the Motor
#define LMotor_offset 35 // The offset of the Motor


// walkin
// static float kp = 40.00f;
// static float kd = 1000.00f;

// static float kp = 40.00f;
// static float kd = 600.00f;

// static float kp = 23.00f;
// static float kd = 950.00f;

static float kp = 10.00f;
static float kd = 800.00f;

static float ki = 0.0f;

uint32_t _prev = millis();

static uint32_t lastTime = millis();

int TN1 = 4;
int TN2 = 3;

int TN3 = 8;
int TN4 = 7;

int ENA = 11;
int ENB = 12;

static float error = 0;  // Proportion
static float errSum = 0;
static float dErr = 0;

struct Motor {
  float Output;
} motor_t;

static bool system_running = false;


//http://www.pieter-jan.com
static void ComplementaryFilter(float accData[3], float gyrData[3], float *pitch, float *roll)
{
  static float pitchAcc, rollAcc;

  // Integrate the gyroscope data -> int(angularSpeed) = angle
  *pitch += ((float)gyrData[0] / GYROSCOPE_SENSITIVITY) * dt; // Angle around the X-axis
  *roll -= ((float)gyrData[1] / GYROSCOPE_SENSITIVITY) * dt;    // Angle around the Y-axis

  // Compensate for drift with accelerometer data if !bullshit
  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
  int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    // Turning around the X axis results in a vector on the Y-axis
    pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
    *pitch = *pitch * 0.8 + pitchAcc * 0.2;

    // Turning around the Y axis results in a vector on the X-axis
    // rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
    // *roll = *roll * 0.9 + rollAcc * 0.1;
  }
}

void PWMControl(float Output);
void myPID(float filtered_angle, struct Motor *motor);
void readIMUSensor(float*);


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // CurieImu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);
  // CurieImu.setFullScaleGyroRange(BMI160_GYRO_RANGE_2000);
  Serial.begin(115200);

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  imu.settings.mag.enabled = false; // Enable magnetometer

  imu.settings.accel.scale = A_SCALE_2G;
  imu.settings.accel.sampleRate = 1;

    // [enabled] turns the acclerometer on or off.
  imu.settings.accel.enabled = true; // Enable accelerometer
  // [enableX], [enableY], and [enableZ] can turn on or off
  // select axes of the acclerometer.
  imu.settings.accel.enableX = true; // Enable X
  imu.settings.accel.enableY = true; // Enable Y
  imu.settings.accel.enableZ = true; // Enable Z

  imu.settings.accel.bandwidth = 0; // BW = 408Hz
  // [highResEnable] enables or disables high resolution
  // mode for the acclerometer.
  imu.settings.accel.highResEnable = true; // Disable HR
  // [highResBandwidth] sets the LP cutoff frequency of
  // the accelerometer if it's in high-res mode.
  // can be any value between 0-3
  // LP cutoff is set to a factor of sample rate
  // 0 = ODR/50    2 = ODR/9
  // 1 = ODR/100   3 = ODR/400
  imu.settings.accel.highResBandwidth = 0;
  imu.settings.gyro.scale = G_SCALE_2000DPS;
  imu.settings.gyro.sampleRate = 4;

  imu.settings.gyro.bandwidth = 0;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = true; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)
  imu.settings.gyro.HPFCutoff = 1; // HPF cutoff = 4Hz

  imu.settings.gyro.flipX = false; // Don't flip X
  imu.settings.gyro.flipY = false; // Don't flip Y
  imu.settings.gyro.flipZ = false; // Don't flip Z

  Serial.println("START IMU");
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1)
      ;
  }
  Serial.println("FINISHED");

  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  digitalWrite(LED_BUILTIN, HIGH);
  imu.calibrate();
  imu.calibrate(true);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println(" Done");

  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);

  static bool first_run = true;
  static float pitch_value_filtered;
  // while(first_run){
  //   readIMUSensor(&pitch_value_filtered);
  //   if (pitch_value_filtered == 0) {
  //     first_run = false;
  //     break;
  //   }
  // };
  system_running = true;
  digitalWrite(LED_BUILTIN, LOW);
  _prev = millis();

  // while(true) {
  //   forward();
  //   delay(500);
  // //   backward();
  // //   delay(500);
  // }
  // //
}

void loop() {
  static Motor motor;
  static float pitch_value_filtered;
  readIMUSensor(&pitch_value_filtered);
  if (system_running && ((millis() - _prev) > 10)){
    _prev = millis();
    //   If angle > 45 or < -45 then stop the robot
    if (abs(pitch_value_filtered) < 300) {
      myPID(pitch_value_filtered, &motor);
      PWMControl(motor.Output);
    }
    else
    {
      system_running = false;
      //TODO:// set zero when robot down.
          //  Output = error = errSum = dErr = 0;
      // Serial.println("STOP");
      analogWrite(ENA, 0);  // Stop the wheels
      analogWrite(ENB, 0);  // Stop the wheels
    }
  }
}

void myPID(float filtered_angle, struct Motor *motor) {
  static float lastErr = 0;
  static float Output;
  uint32_t timeChange = (millis() - lastTime);
  lastTime = millis();

  error = filtered_angle;  // Proportion
  errSum += error * timeChange;  // Integration
  dErr = (error - lastErr) / timeChange;  // Differentiation
  Output = kp * error + ki * errSum + kd * dErr;
  lastErr = error;

  motor->Output = Output;
}

void backward() {
    digitalWrite(TN1, 1);
    digitalWrite(TN2, 0);

    digitalWrite(TN3, 0);
    digitalWrite(TN4, 1);

    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void forward() {
    digitalWrite(TN1, 0);
    digitalWrite(TN2, 1);

    digitalWrite(TN3, 1);
    digitalWrite(TN4, 0);

    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void PWMControl(float Output)
{
  if (Output > 0) {
    forward();
  }
  else if (Output < 0)
  {
    backward();
  } else {
    digitalWrite(ENA, 0);
    digitalWrite(ENB, 0);
  }

  analogWrite(ENA, min(255, (abs(Output) + LMotor_offset)));
  analogWrite(ENB, min(255, (abs(Output) + RMotor_offset)));
}

void readIMUSensor(float *pitch_value_filtered) {
  static float roll;
  static float pitch;

  static float accData[3];
  static float gyrData[3];

  // CurieImu.getMotion6(&accData[0], &accData[1], &accData[2], &gyrData[0], &gyrData[1], &gyrData[2]);
  while(!imu.accelAvailable());
  imu.readAccel();

  while(!imu.gyroAvailable());
  imu.readGyro();

  accData[0] = (imu.ax);
  accData[1] = (imu.ay);

  gyrData[0] = (imu.gx);
  gyrData[1] = (imu.gy);
  gyrData[2] = (imu.gz);

  ComplementaryFilter(accData, gyrData, &pitch, &roll);
  *pitch_value_filtered = pitch;

  if (system_running) {
    // Serial.println(pitch);
  }

}
