//
// Wiring:
// Motor shield + Arduino Uno at 3.3V
// 240x320 SPI TFT screen
// TFT    Arduino
// --------------
// 
// MPU9250 at I2C
//
#include <MPU9250_asukiaaa.h>
#include  <Adafruit_MotorShield.h>
#include <Adafruit_GFX.h> 
#include <Adafruit_ILI9341.h>
#include  <SoftwareSerial.h>


#define MOTOR_ADDRESS 0x60
#define IMU_ADDRESS   0x68

#define NUM_COLS      20
#define NUM_ROWS      4

#define BT_SPEED      9600
#define BT_RX         4
#define BT_TX         5

#define TFT_CS        10
#define TFT_RST       8
#define TFT_DC        9
#define TFT_MISO      12
#define TFT_MOSI      11
#define TFT_SCK       13

MPU9250_asukiaaa      imu;
Adafruit_ILI9341      tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MotorShield  ams = Adafruit_MotorShield();
Adafruit_DCMotor      *leftMotor = ams.getMotor(1);
Adafruit_DCMotor      *rightMotor = ams.getMotor(2);
SoftwareSerial        bt(BT_RX, BT_TX);

// Global variables
int status;
char cmd = 'S';                   // Command received from BT
float accX, accY, accZ;           // Acceleration from IMU
float girX, girY, girZ;           // Gyro from IMU
float cmpX, cmpY, cmpZ;           // Compass from IMU
float accXcal, accYcal, accZcal;  // Calibratrd acceleration offsets
float girXcal, girYcal, girZcal;  // Calibrated gyro offsets

int yh = 10;
int yl = 210;
int y0 = 110;
float vmin = -1.0;
float vmax = 1.0;
float c = 0.0;
int xmin = 10;
int xmax = 300;
int index = 1;
int maxindex = xmax - xmin -1;
float accXprev, accYprev;
  
float calcCoefficient(int yh, int yl, float vmin, float vmax)
{
  return (float)(yl - yh)/(vmax-vmin);
}

int calcY( int y0, float c, float v)
{
  return y0 - c * v;
}

void drawCoordinates(int yh, int yl, int y0)
{
  tft.drawLine(xmin, yh,  xmax, yh, ILI9341_WHITE);
  tft.drawLine(xmin, yl,  xmax, yl, ILI9341_WHITE);
  tft.drawLine(xmin, yh,  xmin, yl, ILI9341_WHITE);
  tft.drawLine(xmax, yh,  xmax, yl, ILI9341_WHITE);
  tft.drawLine(xmin+1, y0,  xmax-1, y0, ILI9341_BLUE);
}

// Motor control routines

void runForward()
{
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void runBackward()
{
  leftMotor->setSpeed(200);
  rightMotor->setSpeed(200);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
}

void runLeft()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(BACKWARD);
  rightMotor->run(FORWARD);
}

void runRight()
{
  leftMotor->setSpeed(150);
  rightMotor->setSpeed(150);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
}
void runStop()
{
  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

// IMU routines

void  displayIMU()
{
  
  if(index != 1) // Not the leftmost point --> draw the line from the previous point
  {
    tft.drawLine(xmin+index-1, y0-c*accXprev, xmin+index, y0-c*accX, ILI9341_RED);
    tft.drawLine(xmin+index-1, y0-c*accYprev, xmin+index, y0-c*accY, ILI9341_GREEN);
    accXprev = accX;
    accYprev = accY;
  }
  else // Leftmost point --> draw just the point
  {
    tft.startWrite();
    tft.writePixel(xmin+index, y0-c*accX, ILI9341_RED);
    tft.writePixel(xmin+index, y0-c*accY, ILI9341_GREEN);
    tft.endWrite();
  }

  index++;
  if(index >= maxindex)
  {
    index=1;
    tft.fillRect(xmin+1, yh+1, xmax-1, yl-1, ILI9341_BLACK);
    drawCoordinates(yh, yl, y0);
    
  }
    
  tft.fillRect(0, 220, 299, 20, ILI9341_BLACK);
  tft.setTextSize(1);
  
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(0, 220);
  tft.print(accX);
  tft.setCursor(100, 220);
  tft.print(accY);
  tft.setCursor(200, 220);
  tft.print(accZ);

  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(0, 230);
  tft.print(girX);
  tft.setCursor(100, 230);
  tft.print(girY);
  tft.setCursor(200, 230);
  tft.print(girZ);
 }

void sendIMU()
{
  bt.print(cmd);
  bt.print(", ");
  
  bt.print(accX);
  bt.print(", ");
  bt.print(accY);
  bt.print(", ");
  bt.print(accZ);
  bt.print(", ");

  bt.print(girX);
  bt.print(", ");
  bt.print(girY);
  bt.print(", ");
  bt.print(girZ);
  bt.print(", ");

  bt.print(cmpX);
  bt.print(", ");
  bt.print(cmpY);
  bt.print(", ");
  bt.print(cmpZ);
  bt.println(" ");

}

void calibrateIMU()
{
  int sz = 20;
  accXcal = accYcal = accZcal = 0.0;
  girXcal = girYcal = girZcal = 0.0;
  Serial.println("IMU calibration started. Do NOT move the car!");
  for(int i=0; i<sz; i++)
  {
    imu.accelUpdate();
    accX = imu.accelX();
    accY = imu.accelY();
    accZ = imu.accelZ();

    imu.gyroUpdate();
    
    girX = imu.gyroX();
    girY = imu.gyroY();
    girZ = imu.gyroZ();

    Serial.print(i);
    Serial.print(",   ");
    Serial.print(accX);
    Serial.print(", ");
    Serial.print(accY);
    Serial.print(", ");
    Serial.print(accZ);
    Serial.print("   ");
    Serial.print(girX);
    Serial.print(", ");
    Serial.print(girY);
    Serial.print(", ");
    Serial.println(girZ);

    accXcal += accX;
    accYcal += accY;
    accZcal += accZ;

    girXcal += girX;
    girYcal += girY;
    girZcal += girZ;

    delay(100);
  }

  Serial.println("IMU calibration completed");
  
  accXcal = accXcal / sz;
  accYcal = accYcal / sz;
  accZcal = accZcal / sz;

  girXcal = girXcal / sz;
  girYcal = girYcal / sz;
  girZcal = girZcal / sz;
}

void setup() {
  pinMode( 19, INPUT_PULLUP ); // fix Serial1

  Serial.begin(9600);
  Serial.println("IMU_BT_MOTOR_TFT started");
  
  ams.begin();          // Motor shield initialization
  leftMotor->setSpeed(0);
  leftMotor->run(RELEASE);
  rightMotor->setSpeed(0);
  rightMotor->run(RELEASE);
  Serial.println("AMS ready");
  
  bt.begin(BT_SPEED);   // HC-06 Bluetooth module initialization
  bt.print("BT ready");
  Serial.println("BT ready");
  
  
  pinMode(TFT_RST, INPUT_PULLUP);
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  Serial.println("TFT ready");
  c = calcCoefficient(yh, yl, vmin, vmax);
  Serial.print("Coefficient="); Serial.println(c);
  drawCoordinates(yh, yl, y0);
  Serial.println("Coordinates drawn");

  // start communication with IMU 

    Wire.begin();
    imu.setWire(&Wire);
    imu.beginAccel();
    imu.beginGyro();
    imu.beginMag();
    Serial.println("IMU started(Accel+Gyro)");
      
  
  calibrateIMU();
  Serial.println("Setup completed");
}

void loop() {
  
  // Get command from bluetooth
  if( bt.available())
  {
    cmd = bt.read();
  }

  tft.fillRect(300, 220, 16, 16, ILI9341_BLACK);
  tft.setCursor(300, 220);
  tft.setTextColor(ILI9341_MAGENTA);
  tft.setTextSize(2);
   
  switch (cmd)
  {
    case 'F':
      Serial.println("--> Forward");
      tft.println("F");
      runForward();
      break;
    
    case 'B':
      Serial.println("--> Backward");
      tft.println("B");
      runBackward();
    break;

    case 'L':
      Serial.println("--> Left");
      tft.println("L");
      runLeft();
      break;

    case 'R':
      Serial.println("--> Right");
      tft.println("R");
      runRight();
      break;

    case 'S':
      Serial.println("--> Stop");
      tft.println("S");
      runStop();
      break;

    default:
      Serial.println("--> unknown command");
      tft.println("?");
      runStop();
      break;
  }

  
  // read the IMU sensor
  imu.accelUpdate();
  accX = imu.accelX() - accXcal;
  accY = imu.accelY() - accYcal;
  accZ = imu.accelZ() - accZcal;

  imu.gyroUpdate();
  girX = imu.gyroX() - girXcal;
  girY = imu.gyroY() - girYcal;
  girZ = imu.gyroZ() - girZcal;

  imu.magUpdate();
  cmpX = imu.magX();
  cmpY = imu.magY();
  cmpZ = imu.magZ();
  
  displayIMU();
  
  sendIMU();
  
  delay(50);
}
