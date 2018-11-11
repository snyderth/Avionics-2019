#include "Arduino.h"

#define __ACCEL_TEST__
#define __PRESSURE_TEST__
#define __HEARTBEAT__
#define __IMU_TEST__
#define __MULTI_SENSOR__
#define LINE_LIMIT 100

#include <Wire.h>

//SD Libs
#include <SD.h>
#include <SPI.h>

//high-g accel
#include <ADXL377.h>

//Pressure sensor
#include <MPL3115A2.h>

//IMU
#include <Adafruit_Sensor.h> //Dep for Adafruit_BNO055
#include <Adafruit_BNO055.h>




//TWI Addresses
int iBaro_Addr = 0xC0;
//see Adafruit_BNO055.h look for BNO055_ADDRESS_A



//Hardware pins:
int iSDA_Pin = 18;
int iSCL_Pin = 19;

//LED pins
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
float PressureRead = 0, AltRead = 0; //Pressure and altitude
float x = 0, y = 0, z = 0; //accel_sensor


//instance of MPL3115 (low pres sense) class
//Adafruit_MPL3115A2 Alt_Sensor = Adafruit_MPL3115A2();
MPL3115A2 Alt_Sensor = MPL3115A2();

//instance of high-g accelerometer
ADXL377 accel_sensor = ADXL377();


//instance of IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDRESS_A);//sensor ID, i2c addr

//Vars associated with SD
const int chipSelect = BUILTIN_SDCARD;
File dataFile;
int lineCount;


/*Initialize all sensors here*/
bool init_sensors(void){

  //initialize altimeter
  #ifdef __PRESSURE_TEST__
    //begins sensor, args only for if using single sensor on i2c
    Alt_Sensor.begin(iSDA_Pin, iSCL_Pin);
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
      return false;
      while(1);
    }
    bno.setExtCrystalUse(true);
  #endif

  return true;
}

/*
  Initialize wire library here. NOTE: If not using __MULTI_SENSOR__
  the wire SCL and SDA may not be set correctly within libs.
*/
bool init_i2c(){
  #ifdef __MULTI_SENSOR__
    Wire.setSDA(iSDA_Pin); //Set SDA
    Wire.setSCL(iSCL_Pin); //Set SCL
    Wire.begin(); //Start i2c
  #endif

  return true;
}


/*Run once*/
void setup() {
  //Serial Comms for debugging
  Serial.begin(115200);
  while(!Serial){}//

  lineCount = 0;//Counter for the SD card writer to save the file

  init_i2c();
  init_sensors();

  SD.begin(chipSelect); //init SD card
  pinMode(pLED, OUTPUT);
}



void loop() {

  if(lineCount < LINE_LIMIT){
    dataFile = SD.open("datalog.txt", FILE_WRITE);
  }else{
    dataFile.close();
    lineCount = 0;
  }

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
    Serial.println("");
  }
  if(dataFile){
    dataFile.print("Time (s): ");
    dataFile.print((float)timeCurMS/1000);
    //Serial.print("   Altitude (ft): ");
    //Serial.print(AltRead*0.3048);
    dataFile.print("   Pressure (kPa) ");
    dataFile.print(PressureRead/1000.0);
    dataFile.println("");
    lineCount++;
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
  }
  if(dataFile){
    dataFile.print("Time (s): ");
    dataFile.print((float)timeCurMS/1000);
    //Serial.print("   Altitude (ft): ");
    //Serial.print(AltRead*0.3048);
    dataFile.printf("   x: %fg, y: %fg, z: %fg\n",x,y,z);
    lineCount++;
  }
#endif


#ifdef __IMU_TEST__
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  if((timeCurMS - timePrintMS) >= PRINT_TIME){
    /* Display the floating point data */
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    Serial.println("");
  }
  if(dataFile){
      dataFile.print("X: ");
      dataFile.print(event.orientation.x, 4);
      dataFile.print("\tY: ");
      dataFile.print(event.orientation.y, 4);
      dataFile.print("\tZ: ");
      dataFile.print(event.orientation.z, 4);
      dataFile.println("");
      lineCount++;
  }

#endif
if((timeCurMS - timePrintMS) >= PRINT_TIME){
  timePrintMS = timeCurMS;
}




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
