#include <Wire.h>
#include <SparkFunLSM9DS1.h>
//#include <MadgwickAHRS.h>
#include <SensorFusion.h>

SF fusion;

LSM9DS1 imu;

float deltat;
float gx, gy, gz, ax, ay, az, mx, my, mz;

#define LSM9DS1_M   0x1E
#define LSM9DS1_AG  0x6B
#define DECLINATION -19.75

#define PRINT_CALCULATED
#define PRINT_SPEED 250
static unsigned long lastPrint = 0;

#define ALL_LED PIN_B7
#define GRN_LED PIN_C2
#define YEL_LED PIN_C3
#define RED_LED PIN_C4
#define YEL2_LED PIN_C5
#define YEL3_LED PIN_C6

#define CLUTCH PIN_C7  // Seems to be Output with P channel Mosfet controlling
#define HORN PIN_B6    
#define RUD PIN_F0     // Probably Analog Input
#define HDRV PIN_F2    // HCUR on Motor board -- Conencts to Source of NFET (also GND) by 10K resistor, bypassed with large cap, maybe Analog?

#define AUTO_BTN PIN_D7
#define STAR_BTN PIN_E0
#define PORT_BTN PIN_E1

#define PORTDO PIN_D5
#define STARDO PIN_D6

#define RF1 PIN_F6
#define RF2 PIN_F7
#define RF3 PIN_C0
#define RF4 PIN_C1


float mag_offsets[3]            = { -9.4F, 51.95F, 15.65F };
float mag_softiron_matrix[3][3] = { { 1.002, 0.038, 0.026 },
                                    { 0.038, 1.001, -0.001 },                                   
                                    { 0.026, -0.001, 0.999 } }; 

float mag_field_strength        = 48.41F;


void setup() 
{
  Serial1.begin(115200);
  Serial.begin(115200);

  pinMode(ALL_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(YEL_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(YEL2_LED, OUTPUT);
  pinMode(YEL3_LED, OUTPUT);
  pinMode(PORTDO, OUTPUT);
  pinMode(STARDO, OUTPUT);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;

  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    while (1);
  } 
}

void loop() 
{
  bool port; 
  bool starboard; 
  bool auto_btn; 
  bool rf1;
  bool rf2;
  bool rf3;
  bool rf4;
  
  port = !digitalRead(PORT_BTN);
  starboard = !digitalRead(STAR_BTN); 
  auto_btn = !digitalRead(AUTO_BTN); 
  rf1 = digitalRead(RF1);
  rf2 = digitalRead(RF2);
  rf3 = digitalRead(RF3);
  rf4 = digitalRead(RF4);
  
  digitalWrite(GRN_LED, rf2 || rf4);
  digitalWrite(YEL2_LED, rf1 || rf3);
  digitalWrite(YEL3_LED, starboard);
  digitalWrite(YEL_LED, auto_btn);
  digitalWrite(RED_LED, port);
  digitalWrite(ALL_LED, HIGH);

  imu.readGyro();
  imu.readAccel();
  imu.readMag();

  #define MSS_PER_G 9.80665
  #define UT_PER_GAUSS 100
  #define RAD_PER_DEG 0.0174533

  ax = imu.calcAccel(imu.ax) * MSS_PER_G;
  ay = imu.calcAccel(imu.ay) * MSS_PER_G;
  az = imu.calcAccel(imu.az) * MSS_PER_G;

  gx = imu.calcGyro(imu.gx) * RAD_PER_DEG;
  gy = imu.calcGyro(imu.gy) * RAD_PER_DEG;
  gz = imu.calcGyro(imu.gz) * RAD_PER_DEG;

  float x = imu.calcMag(imu.mx) * UT_PER_GAUSS - mag_offsets[0];
  float y = imu.calcMag(imu.my) * UT_PER_GAUSS - mag_offsets[1];
  float z = imu.calcMag(imu.mz) * UT_PER_GAUSS - mag_offsets[2]; 

  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  deltat = fusion.deltatUpdate();
  fusion.MadgwickUpdate(gx, gy, -gz, -ax, -ay, az, -mx, my, -mz, deltat);

  auto roll = fusion.getRoll();
  auto pitch = fusion.getPitch();
  auto heading = fusion.getYaw();

  Serial.println(heading);
}  
