/*
 
// DPS3005 MODBUS Example By Luke (www.ls-homeprojects.co.uk)
// Free to use this code how you like but please link my site
 
// Credit to Doc Walker of ModbusMaster for making a great Arduino Library https://github.com/4-20ma/ModbusMaster
*/
 
 
#include <ESP8266WiFi.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
 
 
#define SERIAL_RX     D5  // PIN Mapping for ESP8266 NODE MCU Only! pin for SoftwareSerial RX
#define SERIAL_TX     D6  // PIN Mapping for ESP8266 NODE MCU Only! pin for SoftwareSerial TX
SoftwareSerial mySerial(SERIAL_RX, SERIAL_TX);
 
 
ModbusMaster node;        // instantiate ModbusMaster object
float voltage = 0.0;
float current = 0.0;
 
void setup()
{
 
    Serial.begin(9600);
 
    mySerial.begin(9600);     //start software serial connected to dps3005
    pinMode(SERIAL_RX, INPUT);
    pinMode(SERIAL_TX, OUTPUT);
 
 
  // communicate with Modbus slave ID 1 over Serial (port 0)
    node.begin(1,mySerial);
}
 
 
void loop()
{
 
  uint8_t j, result;
 
    //********Example of sending cmds***********
    node.writeSingleRegister(0, 0x96); //set 1.5v
    node.writeSingleRegister(1, 0x1F4); //set 500ma
    node.writeSingleRegister(9, 1); //set power on ! 
 
    delay(2000);
 
  //*******Example of reading data*****************
 
 result = node.readHoldingRegisters(2, 2);  // slave: read a range of 16-bit registers starting at register 2 to 3 (measured voltage and current) 
 
 
 if (result == node.ku8MBSuccess)        // only do something with data if read is successful
  {
 
    voltage =  ((float)node.getResponseBuffer(0) / 100 ); // get voltage from response buffer and convert to float
    current =  ((float)node.getResponseBuffer(1) / 100 ); // get current from response buffer and convert to float
 
    Serial.println("");
 
    Serial.print ("Voltage: ");
    Serial.print (voltage);
    Serial.println ("V  ");
   
    Serial.println("");
 
    Serial.print ("Current: : ");
    Serial.print (current);
    Serial.println ("A  ");
   
    delay(2000);
 
  }
 
}
