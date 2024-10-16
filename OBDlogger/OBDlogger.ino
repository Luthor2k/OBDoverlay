/*
  Get OBD data from an ISO 9141 based car and log it to the sd card with a timestamp

 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10
*/

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

#include "Arduino.h"
// Be sure that the AltSoftSerial library is available, download it from http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html"
#include "AltSoftSerial.h"
#include "OBD9141.h"

const int chipSelect = 10;

#define RX_PIN 8  // connect to transceiver Rx
#define TX_PIN 9  // connect to transceiver Tx
#define EN_PIN 7  //  pin will be set high (connect to EN pin of SN65HVDA100)

AltSoftSerial altSerial;

OBD9141 obd;

#define REQDELAY 50 //ms between requests, 50 seems to be about the minimum for a '97 Geo Metro

String logFileName;

bool prependzero(int number) {
  if (number >= 0 && number < 10) {
    return (true);
  }
  return (false);
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Init SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed");
    // don't do anything more:
    while (1);
  }
  Serial.println("card good.");

  delay(2000);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // enable the transceiver IC.

  obd.begin(altSerial, RX_PIN, TX_PIN);

  tmElements_t tm;
  if (RTC.read(tm)) {

  } else {
    if (RTC.chipPresent()) {
      Serial.println("The DS1307 is stopped.  Please run the SetTime");
      Serial.println("example to initialize the time and begin running.");
      Serial.println();
    } else {
      Serial.println("DS1307 read error!  Please check the circuitry.");
      Serial.println();
    }
  }
  // make a string for the log file name
  //log file name can only be dos 8.3
  //ddmmhhmm
  String dataString = "";

  //add date and time to string
  if (prependzero(tm.Day)) dataString += "0";
  dataString += String(tm.Day);
  if (prependzero(tm.Month)) dataString += "0";
  dataString += String(tm.Month);

  if (prependzero(tm.Hour)) dataString += "0";
  dataString += String(tm.Hour);
  if (prependzero(tm.Minute)) dataString += "0";
  dataString += String(tm.Minute);

  logFileName += dataString;
  logFileName += ".csv";
  Serial.print("log file name title: ");
  Serial.println(logFileName);

  File dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.println("date,time,LOAD,ECT,MAP,RPM,speed,advance,IAT,TPS");
    dataFile.close(); 
  }
  else {
    Serial.println("log file error");
  }
}

void loop() {
  //Serial.print('|');
  //Serial.print(0x01);
  //Serial.print(0x80);
  Serial.println("OBD init loop");

  bool init_success =  obd.init();
  Serial.print("init_success:");
  Serial.println(init_success);

  //read OBD and append to string or wait on init
  if (init_success) {
    bool res;
    while (1)
    {
      tmElements_t tm;

      if (RTC.read(tm)) {

      } else {
        if (RTC.chipPresent()) {
          Serial.println("The DS1307 is stopped");
        } else {
          Serial.println("DS1307 read error");
        }
      }

      // make a string for assembling the data to log:
      String dataString = "";

      //add date and time to string
      if (prependzero(tm.Day)) dataString += "0";
      dataString += String(tm.Day);
      dataString += String("/");
      if (prependzero(tm.Month)) dataString += "0";
      dataString += String(tm.Month);
      dataString += String("/");
      dataString += String(tmYearToCalendar(tm.Year));
      dataString += String(",");

      if (prependzero(tm.Hour)) dataString += "0";
      dataString += String(tm.Hour);
      dataString += String(":");
      if (prependzero(tm.Minute)) dataString += "0";
      dataString += String(tm.Minute);
      dataString += String(":");
      if (prependzero(tm.Second)) dataString += "0";
      dataString += String(tm.Second);
      dataString += String(",");

      /* PIDs to read:
      Engine Load
      Engine coolant temperature 
      Intake manifold absolute pressure 
      Engine speed 
      Vehicle speed 
      Timing advance 
      Intake air temperature
      MAF
      Throttle position 
      O2 bank 1 sensor 1 voltage
      O2 bank 1 sensor 2 voltage
       */
      float reading = 0;
      res = obd.getCurrentPID(0x04, 1); //LOAD
      if (res) {
        reading = obd.readUint8() / 2.55;
        dataString += String(reading);
        dataString += String(",");
      }
      delay(REQDELAY);

      res = obd.getCurrentPID(0x05, 1); //ECT
      if (res) {
        dataString += String(obd.readUint8() - 40);
        dataString += String(",");
      }
      delay(REQDELAY);
      
      
      res = obd.getCurrentPID(0x0B, 1); //MAP
      if (res) {
        //no conversion needed
        dataString += String(obd.readUint8());
        dataString += String(",");
      }
      delay(REQDELAY);
      
      res = obd.getCurrentPID(0x0C, 2); //RPM
      if (res) {
        dataString += String(obd.readUint16() / 4);
        dataString += String(",");
      }
      delay(REQDELAY);

      res = obd.getCurrentPID(0x0D, 1); //speed
      if (res) {
        //no conversion needed
        dataString += String(obd.readUint8());
        dataString += String(",");
      }
      delay(REQDELAY);

      res = obd.getCurrentPID(0x0E, 1); //advance
      if (res) {
        reading = obd.readUint8() / 2 - 64;
        dataString += String(reading);
        dataString += String(",");
      }
      delay(REQDELAY);

      res = obd.getCurrentPID(0x0F, 1); //IAT
      if (res) {
        dataString += String(obd.readUint8() - 40);
        dataString += String(",");
      }
      delay(REQDELAY);

      res = obd.getCurrentPID(0x11, 1); //TPS
      if (res) {
        reading = obd.readUint8() / 2.55;
        dataString += String(reading);
        dataString += String(",");
      }
      delay(REQDELAY);
      
      Serial.println(dataString);
      
      File dataFile = SD.open(logFileName, FILE_WRITE);
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();        
      }
      else {
        Serial.println("log file error");
      }
    }
  }

  delay(2000);
}
