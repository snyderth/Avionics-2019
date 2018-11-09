#include "Arduino.h"

//#include <i2c_t3.h>
//#include <i2c_t3.cpp>

#include <ADXL377.h>

#include <Wire.h>
//#include <Adafruit_MPL3115A2.h>
#include <MPL3115A2.h>

#include <Adafruit_Sensor.h> //Dep for Adafruit_BNO055
#include <Adafruit_BNO055.h>


// #define __ACCEL_TEST__
// #define __PRESSURE_TEST__
#define __HEARTBEAT__
#define __IMU_TEST__
#define __MULTI_SENSOR__

//TWI Addresses
int iBaro_Addr = 0xC0;
//int iIMU_Addr = 0x;
//int iPres_Addr = 0x;

//Hardware pins:
int iSDA_Pin = 18;
int iSCL_Pin = 19;
int pLED = 13;
bool bLED = false;

//accel HW pins
int xpin = 20;
int ypin = 22;
int zpin = 23;

//time variables
unsigned long timeCurMS;
unsigned long timePrintMS = 0;
unsigned long timeBlinkMS = 0;
unsigned long timeReadMS = 0;
const unsigned long PRINT_TIME = 100; // ms
const unsigned long BLINK_TIME = 1000; //ms

//sensor read variables
float PressureRead = 0;
float AltRead = 0;
float x = 0, y = 0, z = 0; //accel_sensor


//instance of MPL3115 (low pres sense) class
//Adafruit_MPL3115A2 Alt_Sensor = Adafruit_MPL3115A2();
MPL3115A2 Alt_Sensor = MPL3115A2();

ADXL377 accel_sensor = ADXL377();

Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A);//sensor ID, i2c addr

void setup() {
  //Serial Comms for debugging
  Serial.begin(115200);
  while(!Serial){}

  Serial.println("Start");
  #ifdef __MULTI_SENSOR__
  Wire.setSDA(iSDA_Pin);
  Wire.setSCL(iSCL_Pin);

  #endif
  //TWI setup
  //Wire.setSDA(iSDA_Pin);  //set SDA Pin
  //Wire.setSCL(iSCL_Pin);  //set SCL Pin
  //Wire.begin();           //start TWI comm



#ifdef __PRESSURE_TEST__
  Alt_Sensor.begin(iSDA_Pin, iSCL_Pin);       //initialize altimeter
#endif



#ifdef __ACCEL_TEST__ //init accel
  accel_sensor.begin(xpin, ypin, zpin);
#endif



#ifdef __IMU_TEST__ //init IMU
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

#endif


  pinMode(pLED, OUTPUT);
}

void loop() {
  timeCurMS = millis();
#ifdef __PRESSURE_TEST__
  PressureRead = Alt_Sensor.getPressure();
  AltRead = Alt_Sensor.getAltitude();
  //serial output for debug

  if((timeCurMS - timePrintMS) >= PRINT_TIME){
    Serial.print("Time (s): ");
    Serial.print((float)timeCurMS/1000);
    //Serial.print("   Altitude (ft): ");
    //Serial.print(AltRead*0.3048);
    Serial.print("   Pressure (kPa) ");
    Serial.print(PressureRead/1000.0);
    Serial.println();
    timePrintMS = timeCurMS;
  }
#endif


#ifdef __ACCEL_TEST__
  x = accel_sensor.readX();
  y = accel_sensor.readY();
  z = accel_sensor.readZ();
  if((timeCurMS - timePrintMS) >= PRINT_TIME){
    Serial.print("Time (s): ");
    Serial.print((float)timeCurMS/1000);
    //Serial.print("   Altitude (ft): ");
    //Serial.print(AltRead*0.3048);
    Serial.printf("   x: %fg, y: %fg, z: %fg\n",x,y,z);
    timePrintMS = timeCurMS;
  }
#endif


#ifdef __IMU_TEST__
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
#endif

  //Serial.println("Time: ");
  //delay(1000);
  //heartbeat blink:
#ifdef __HEARTBEAT__
  if((timeCurMS - timeBlinkMS) >= BLINK_TIME){
    if(bLED){
      bLED = false;
      digitalWrite(pLED,LOW); }
    else{
      bLED = true;
      digitalWrite(pLED,HIGH);
      Serial.printf("Hearbeat\n");
     }
    timeBlinkMS = timeCurMS;
  }
  #endif


}
