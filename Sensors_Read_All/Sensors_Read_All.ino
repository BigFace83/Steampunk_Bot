//*********************************************************************
//    FILE: All_Sensors.ino
//  AUTHOR: Peter Neal
// PURPOSE: Read from all sensors
//     URL: 
//
//  DESCRIPTION:
//  Test code to rread from all sensors on the Steampunk robot
//  Sensors:
//  I2C Bus 0 - VL53L0X Right, AS5600 Right, Compass
//  I2C Bus 1 - VL53L0x Left,  AS5600 Left
//  Battery Voltage
//  Touch Input
//  Write sensor values to the connected TFT screen
//*********************************************************************

#include "AS5600.h"
#include <Wire.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>
#include <QMC5883LCompass.h>
#include <VL53L0X.h>


//define pins for second I2C bus for AS5600
#define I2C1_SCL       17
#define I2C1_SDA       16
#define BATTPIN        34


TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
TFT_eSprite img = TFT_eSprite(&tft);


AS5600 as5600Left(&Wire1);   //  use default Wire
AS5600 as5600Right(&Wire); //second I2C interface

VL53L0X Left_Dist;
VL53L0X Right_Dist;

QMC5883LCompass compass;
int16_t compassDir = 0;



int threshold = 0; //variable for touch input sensitivity
bool touchdetected = false;
void gotTouch() {
  touchdetected = true;
}


int interval = 300; 
unsigned long previousMillis = 0;



void setup() {
  Serial.begin(115200);                               // init serial port for debugging
  delay(2000);

   //init tft screen
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(TL_DATUM); // Centre text on x,y position
  tft.setTextColor(TFT_WHITE, TFT_TRANSPARENT);


  Serial.println("Initialising QMS5338L Compass");
  compass.init();

  
  Wire.begin();
  Wire1.begin(I2C1_SDA, I2C1_SCL, 400000);


  as5600Left.begin();  //  set direction pin.
  as5600Left.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600Left.resetCumulativePosition();
  Serial.print("Left AS5600 -  Connect: ");
  Serial.print(as5600Left.isConnected());
  Serial.print("  I2C Address: ");
  Serial.println(as5600Left.getAddress());


  as5600Right.begin();  //  set direction pin.
  as5600Right.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  as5600Right.resetCumulativePosition();
  Serial.print("Right AS5600 -  Connect: ");
  Serial.print(as5600Right.isConnected());
  Serial.print("  I2C Address: ");
  Serial.println(as5600Right.getAddress());

  Left_Dist.setBus(&Wire1);
  if (!Left_Dist.init())
  {
    Serial.println("Failed to detect and initialize Left Encoder");
  }
  else{
    Left_Dist.init();
    Left_Dist.setTimeout(1000);
    Serial.print("Left VL53L0X - Connected | ");
    Serial.print("I2C Address: ");
    Serial.println(Left_Dist.getAddress());
  }
  
  
  if (!Right_Dist.init())
  {
    Serial.println("Failed to detect and initialize Left Encoder");
  }
  else{
    Right_Dist.setTimeout(1000);
    Serial.print("Right VL53L0X - Connected | ");
    Serial.print("I2C Address: ");
    Serial.println(Right_Dist.getAddress());
  }
  

  Left_Dist.startContinuous();
  Right_Dist.startContinuous();

  

  //Optional: Set the threshold to 5% of the benchmark value. Only effective if threshold = 0.
  touchSetDefaultThreshold(5);
  touchAttachInterrupt(T5, gotTouch, threshold);


  delay(1000);

 
}


void loop() {

  unsigned long now = millis();

  //update loop for reading from AS5600 encoder and updating PID loop for motor speed
  if ((unsigned long)(now - previousMillis) > interval) { // 100ms loop for usefu
    previousMillis = now;                            // reset previousMillis
    
    compass.read();
    Serial.print("Compass: ");
    Serial.print(compass.getAzimuth());

    Serial.print(" | Left VL53L0X: ");
    Serial.print(Left_Dist.readRangeContinuousMillimeters());

    Serial.print(" | Left AS5600: ");
    Serial.print(as5600Left.getCumulativePosition());

    Serial.print(" | Right VL53L0X: ");
    Serial.print(Right_Dist.readRangeContinuousMillimeters());

    Serial.print(" | Right AS5600: ");
    Serial.print(as5600Right.getCumulativePosition());

    Serial.print(" | Battery V: ");
    Serial.println(readBattery(BATTPIN));


    int xpos =  0;
    int ypos = 20;
    tft.setCursor(xpos, ypos);    // Set cursor near top left corner of screen

    tft.drawString("Compass", xpos, ypos, 4);  
    ypos += tft.fontHeight(4);   

    tft.drawString("Left VL53L0X", xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString("Left AS5600", xpos, ypos, 4);  
    ypos += tft.fontHeight(4); 

    tft.drawString("Right VL53L0X", xpos, ypos, 4);  
    ypos += tft.fontHeight(4); 

    tft.drawString("Right AS5600", xpos, ypos, 4);  
    ypos += tft.fontHeight(4);  

    tft.drawString("Battery V", xpos, ypos, 4); 


    tft.setTextPadding(80);
    xpos =  tft.width()/2 + 40;
    ypos = 20;
    tft.setCursor(xpos, ypos);    // Set cursor near top left corner of screen
    tft.drawString(String(compass.getAzimuth()), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString(String(Left_Dist.readRangeContinuousMillimeters()), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString(String(as5600Left.getCumulativePosition()), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString(String(Right_Dist.readRangeContinuousMillimeters()), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString(String(as5600Right.getCumulativePosition()), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);

    tft.drawString(String(readBattery(BATTPIN)), xpos, ypos, 4);  
    ypos += tft.fontHeight(4);
                 


    
  }

  


  


  //handle touch input to reset encoder counts
  if (touchdetected) {
    touchdetected = false;
    if (touchInterruptGetLastStatus(T5)) {
      Serial.println("Touched");
    }
  }
 }


float readBattery(int pin){

  analogReadResolution(12);
  analogSetAttenuation(ADC_2_5db);
  int analogValue = analogRead(pin);
  float voltage = analogValue * ((9*1.25) / 4095.0);
  return voltage;
}








