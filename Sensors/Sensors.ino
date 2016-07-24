#include <Adafruit_BMP280.h>
#include <dht.h>

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include "LowPower.h"
#include "printf.h"


//
// BMP280 (Pressure, Temperature, Altitude Estimate)
//
const byte BMP_CSB = 10; //BMP_CS
const byte BMP_SDA = 11; //BMP_MOSI
const byte BMP_SDO = 12; //BMP_MISO
const byte BMP_SCL = 13; //BMP_SCK

//SPI (Software)
//Adafruit_BMP280 bme(BMP_CSB, BMP_SDA, BMP_SDO, BMP_SCL);

//SPI (Hardware)
Adafruit_BMP280 bme(BMP_CSB);

//I2C
//Adafruit_BMP280 bme;

//
//  DHT22 (Temperature, Humidity)
//
const byte DHT_PIN = 5;
dht DHT;


//
//  NRF24
//
const byte NRF24_CE_PIN  = 7;
const byte NRF24_CSN_PIN = 8;
RF24 radio (NRF24_CE_PIN, NRF24_CSN_PIN);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL,0xF0F0F0F0D2LL};


struct Sensor_Data_Type
{
   double BMP_Temperature;
   double BMP_Pressure;
   double BMP_Altitude;
   double DHT_Temperature;
   double DHT_Humidity;
};

Sensor_Data_Type Sensor_Data = {0.0, 0.0, 0.0, 0.0, 0.0};


bool Debug_Print = true;

//=========================================================
// 
//  Purpose:
//    Convert Celsius temperature to Fahrenheit
//
//=========================================================
double To_Fahrenheit(double Celsius)
{
  return (1.8 * Celsius + 32);
}

//=========================================================
// 
//  Purpose:
//    Convert Celsius temperature to Fahrenheit
//
//=========================================================
double To_Inches_Hg(double Pascals)
{
  return (Pascals / 3386.39);
}

void Initialize_BMP() 
{
  if (!bme.begin()) 
  {  
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
}

void Initialize_NRF24()
{
  radio.begin();
  radio.setRetries(15,15);
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  radio.stopListening();
  //radio.powerDown();

  if (Debug_Print)
  {
    radio.printDetails();
  }
}


void setup() 
{
  Serial.begin(115200);
  printf_begin();
    
  Initialize_BMP();
  //Initialize_DHT();
  Initialize_NRF24();
}

void Print_Sensor_Data()
{
   Serial.print ("DHT Temperature:\t");
   Serial.println(To_Fahrenheit(Sensor_Data.DHT_Temperature));
  
   Serial.print ("DHT Humidity:\t");
   Serial.println(Sensor_Data.DHT_Humidity);
  
   Serial.print ("BMP Temperature:\t");
   Serial.println(To_Fahrenheit(Sensor_Data.BMP_Temperature));
 
   Serial.print ("BMP Pressure:\t");
   Serial.println(Sensor_Data.BMP_Pressure);

   Serial.print ("BMP Pressure IN:\t");
   Serial.println(To_Inches_Hg(Sensor_Data.BMP_Pressure));

   Serial.print ("BMP Altitude:\t");
   Serial.println(Sensor_Data.BMP_Altitude);
  
   Serial.println();
}


void Collect_Sensor_Data()
{
   DHT.read22(DHT_PIN);
   Sensor_Data.DHT_Temperature = DHT.temperature;
   Sensor_Data.DHT_Humidity    = DHT.humidity;

   Sensor_Data.BMP_Temperature = bme.readTemperature();
   Sensor_Data.BMP_Pressure    = bme.readPressure();

   /*Must pass the pressure @ sea level for the current day
   as a parameter to compute Altitude*/
  //Sensor_Data.BMP_Altitude    = bme.readAltitude(1013.25); 
  
}

void Send_Sensor_Data()
{
  // First, stop listening so we can talk.
  radio.stopListening();

  bool ok = radio.write(&Sensor_Data, sizeof(Sensor_Data) );

  if (Debug_Print)
  {
    if (ok)
      Serial.println("Sending Successful");
    else
      Serial.println("Send Failed");
  }

  // Listen for Response
  radio.startListening();

  // Wait here until we get a response, or timeout (250ms)
  unsigned long started_waiting_at = millis();
  bool timeout = false;
  while ( ! radio.available() && ! timeout )
    if (millis() - started_waiting_at > 200 )
      timeout = true;

    // Describe the results
    if ( timeout )
    {
      if (Debug_Print)
        Serial.println("Failed, response timed out.");
    }
    else
    {
      // Grab the response
      unsigned long response;
      radio.read( &response, sizeof(unsigned long) );

      if (Debug_Print)
      {
        Serial.print("Got response: ");
        Serial.println(response);
      }
    }
}

void loop() 
{
    delay (2000);
    Collect_Sensor_Data();
    delay (2000);

    if (Debug_Print)
    {
      Print_Sensor_Data();
    }

    Send_Sensor_Data();

    delay (200);
    //Sleep in low power state
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}



//Acknologements:
/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

