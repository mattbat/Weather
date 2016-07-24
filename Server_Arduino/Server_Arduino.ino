#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"


//
//  NRF24
//
const byte NRF24_CE_PIN  = 7;
const byte NRF24_CSN_PIN = 8;
RF24 radio (NRF24_CE_PIN, NRF24_CSN_PIN);

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = {0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL};


struct Sensor_Data_Type
{
  double BMP_Temperature;
  double BMP_Pressure;
  double BMP_Altitude;
  double DHT_Temperature;
  double DHT_Humidity;
};

Sensor_Data_Type Sensor_Data = {0.0, 0.0, 0.0, 0.0, 0.0};
unsigned long counter = 0;


bool Debug_Print = true;


void Initialize_NRF24()
{ 
  radio.begin();
  radio.setRetries(15, 15);
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1, pipes[0]);
  radio.startListening();

  if (Debug_Print)
  {
    radio.printDetails();
  }
}


void setup()
{
  Serial.begin(115200);
  printf_begin();
  Initialize_NRF24();
}

//=========================================================
// 
//  Purpose:
//    Convert Celsius temperature to Fahrenheit
//
//=========================================================
double To_Fahrenheit(double Celsius)
{
  return 1.8 * Celsius + 32;
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

  Serial.print ("BMP Altitude:\t");
  Serial.println(Sensor_Data.BMP_Altitude);

  Serial.println();
}




void loop()
{
  //Receive Sensor Data

  if (radio.available())
  {

    bool done = false;

    while (!done)
    {
      done = radio.read( &Sensor_Data, sizeof(Sensor_Data) );

      if (Debug_Print)
      {
        Serial.println("Received Data...");
        Print_Sensor_Data();
      }

      // Delay just a little bit to let the other unit
      // make the transition to receiver
      delay(20);

    }

    // First, stop listening so we can talk
    radio.stopListening();

    counter = counter + 1;
    radio.write( &counter, sizeof(unsigned long) );
    
    if (Debug_Print)
      Serial.println("Sent response.\n\r");

    // Now, resume listening so we catch the next packets.
    radio.startListening();

  }
}
