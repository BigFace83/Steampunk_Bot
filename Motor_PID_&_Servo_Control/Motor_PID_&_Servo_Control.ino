//*********************************************************************
//    FILE: PID_Closed_Loop_Speed.ino
//  AUTHOR: Peter Neal
// PURPOSE: Test closed loop motor control
//     URL: 
//
//  DESCRIPTION:
//  Test code to run two DC motors using the DRV8833 driver module
//  Position feedback from an AS5600 encoder mounted on each motor is 
//  used to contol the motor speed using a closed loop PID controller.
//  Motor speed can be increased by touching a touch input pin.
//  Data is displayed on TFT screen if connected.
//*********************************************************************

#include "AS5600.h"
#include <PID_v1.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>


#define MOT1_PIN1    33
#define MOT1_PIN2    32
#define MOT2_PIN1    26
#define MOT2_PIN2    27

//define pins for second I2C bus for AS5600
#define I2C1_SCL       17
#define I2C1_SDA       16


TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite img = TFT_eSprite(&tft);


AS5600 as5600Left(&Wire1);   //  use default Wire
AS5600 as5600Right(&Wire); //second I2C interface




ESP32PWM mot1PWM1;
ESP32PWM mot1PWM2;
ESP32PWM mot2PWM1;
ESP32PWM mot2PWM2;

int threshold = 0; //variable for touch input sensitivity
bool touchdetected = false;
void gotTouch() {
  touchdetected = true;
}


//PID control for Left motor speed with encoder feedback
//Define PID variables
double LeftSP, LeftIn, LeftOut;
//Specify the links and initial tuning parameters
double Kp=0.1, Ki=0.8, Kd=0;
PID LeftPID(&LeftIn, &LeftOut, &LeftSP, Kp, Ki, Kd, DIRECT);


//PID control for Right motor speed with encoder feedback
//Define PID variables
double RightSP, RightIn, RightOut;
//Share gain settings between motors, can be added to have different gains if required
//double Kp=0.1, Ki=0.3, Kd=0;
PID RightPID(&RightIn, &RightOut, &RightSP, Kp, Ki, Kd, DIRECT);


//update interval for AS5600 encoder and TFT - 50ms
unsigned long ENCMillis = 0;
int ENCinterval = 50;

bool up = true;

unsigned long previousMillis = 0;

int LeftPrevPulse = 0;
int RightPrevPulse = 0;


void setup() {
  Serial.begin(115200);                               // init serial port for debugging
  delay(2000);

  //init tft screen
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);


  Wire.begin();
  as5600Left.begin();  //  set direction pin.
  as5600Left.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600Left.resetCumulativePosition();
  Serial.print("Left AS5600 -  Connect: ");
  Serial.print(as5600Left.isConnected());
  Serial.print("  I2C Address: ");
  Serial.println(as5600Left.getAddress());


  Wire1.begin(I2C1_SDA, I2C1_SCL, 400000);
  as5600Right.begin();  //  set direction pin.
  as5600Right.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600Right.resetCumulativePosition();
  Serial.print("Right AS5600 -  Connect: ");
  Serial.print(as5600Right.isConnected());
  Serial.print("  I2C Address: ");
  Serial.println(as5600Right.getAddress());

  

  pinMode(MOT1_PIN1, OUTPUT);
  pinMode(MOT1_PIN2, OUTPUT);
  pinMode(MOT2_PIN1, OUTPUT);
  pinMode(MOT2_PIN2, OUTPUT);
 

  LeftSP = 0;
  LeftPID.SetOutputLimits(-255,255);
  //turn the PID on
  LeftPID.SetMode(AUTOMATIC);
  LeftPID.SetSampleTime(50);

  RightSP = 0;
  RightPID.SetOutputLimits(-255,255);
  //turn the PID on
  RightPID.SetMode(AUTOMATIC);
  RightPID.SetSampleTime(50);

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);


  mot1PWM1.attachPin(MOT1_PIN1, 25000, 8); // Use 25khz to reduce motor whine
  mot1PWM2.attachPin(MOT1_PIN2, 25000, 8); 
  mot2PWM1.attachPin(MOT2_PIN1, 25000, 8); 
  mot2PWM2.attachPin(MOT2_PIN2, 25000, 8); 

  

  //Optional: Set the threshold to 5% of the benchmark value. Only effective if threshold = 0.
  touchSetDefaultThreshold(5);
  touchAttachInterrupt(T5, gotTouch, threshold);


  delay(1000);

 
}


void loop() {

  unsigned long now = millis();

  //update loop for reading from AS5600 encoder and updating PID loop for motor speed
  if ((unsigned long)(now - ENCMillis) > ENCinterval) { // check if "interval" ms has passed since last time the clients were updated
    ENCMillis = now;                            // reset previousMillis
    int Leftpulses = as5600Left.getCumulativePosition();
    LeftIn = Leftpulses - LeftPrevPulse;
    LeftPrevPulse = Leftpulses;

    int Rightpulses = as5600Right.getCumulativePosition();
    RightIn = Rightpulses - RightPrevPulse;
    RightPrevPulse = Rightpulses;

    

    char data[20];

    tft.setTextColor(TFT_YELLOW, TFT_TRANSPARENT);
    tft.drawString("LEFT", 20, 20, 4);
    tft.drawString("SP", 20, 40, 2);
    tft.drawString("In", 20, 60, 2);
    tft.drawString("Out", 20, 80, 2);

    sprintf(data, "%.1f", LeftSP);
    drawData(data, 160, 40);
    sprintf(data, "%.1f", LeftIn);
    drawData(data, 160, 60);
    sprintf(data, "%.1f", LeftOut);
    drawData(data, 160, 80);


    tft.drawString("RIGHT", 20, 120, 4);
    tft.drawString("SP", 20, 140, 2);
    tft.drawString("In", 20, 160, 2);
    tft.drawString("Out", 20, 180, 2);
    sprintf(data, "%.1f", RightSP);
    drawData(data, 160, 140);
    sprintf(data, "%.1f", RightIn);
    drawData(data, 160, 160);
    sprintf(data, "%.1f", RightOut);
    drawData(data, 160, 180);

    
  }


  if(LeftPID.Compute()){
    if(LeftOut<0){
        mot1PWM1.write(0);
        mot1PWM2.write(-LeftOut);
      }
      else{
        mot1PWM1.write(LeftOut);
        mot1PWM2.write(0);
      } 
  }

  if(RightPID.Compute()){
    if(RightOut<0){
        mot2PWM1.write(0);
        mot2PWM2.write(-RightOut);
      }
      else{
        mot2PWM1.write(RightOut);
        mot2PWM2.write(0);
      }

     Serial.print(RightSP);
     Serial.print(",");
     Serial.print(RightIn);
     Serial.print(",");
     Serial.println(RightOut);
  }
  


  //handle touch input to reset encoder counts
  if (touchdetected) {
    touchdetected = false;
    if (touchInterruptGetLastStatus(T5)) {
      if (up){
        LeftSP += 50;
        if(LeftSP>1000){
        up = false;
        }
      }
      else{
        LeftSP -= 50;
        if(LeftSP<-1000){
        up = true;
        }
      }
      RightSP = LeftSP;
    }
  }
 }


// #########################################################################
// Create sprite, write text in it, plot to screen, then delete sprite
// #########################################################################
void drawData(char* data, int x, int y)
{
  // Create an 8-bit sprite 70x 80 pixels (uses 5600 bytes of RAM)
  img.setColorDepth(8);
  img.createSprite(40, 100);

  // Fill Sprite with a "transparent" colour
  // TFT_TRANSPARENT is already defined for convenience
  // We could also fill with any colour as "transparent" and later specify that
  // same colour when we push the Sprite onto the screen.
  img.fillSprite(TFT_TRANSPARENT);

  img.fillRect(0,0,40,100,TFT_BLACK);

  img.setTextColor(TFT_YELLOW, TFT_BLACK);
  //img.setTextDatum(TC_DATUM);
  img.drawString(data, 0, 0, 2);


  // Push sprite to TFT screen CGRAM at coordinate x,y (top left corner)
  // Specify what colour is to be treated as transparent.
  img.pushSprite(x, y, TFT_TRANSPARENT);


  // Delete it to free memory
  img.deleteSprite();
 
}








