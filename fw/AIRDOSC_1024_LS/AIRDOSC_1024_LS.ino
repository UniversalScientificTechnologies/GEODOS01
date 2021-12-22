// R = 12 k OHM
// 0.015 * CH + 0.268 [MeV]
// CH0 = 14
// 1024 ch. from 0.27 to cca 15 MeV

#define DEBUG // Please comment it if you are not debugging
String gittag = "v1";
String FWversion = "C_LS_1024"; // Output data format

#define RANGE 25  // histogram range
#define EVENTS 500 // maximal number of recorded events
#define CHANNELS 512    // number of channels in buffer for histogram, including negative numbers
#define GPSerror 700000 // number of cycles for waitig for GPS in case of GPS error 
//#define GPSdelay  3   // number of measurements between obtaining GPS position
//#define GPSdelay  60   // number of measurements between obtaining GPS position cca 10 minutes
#define GPSdelay 2700   // number of measurements between obtaining GPS position and time
                        // 2700 = cca 12.5 h
#define GPSWAIT 600 // more than 50 s waiting for GPS fix
//#define GPSWAIT 60 // less waiting for GPS fix

// Compiled with: Arduino 1.8.13

/*
  AORDOS_C for GeoDos

ISP
---
PD0     RX
PD1     TX
RESET#  through 50M capacitor to RST#

SDcard
------
DAT3   SS   4 B4
CMD    MOSI 5 B5
DAT0   MISO 6 B6
CLK    SCK  7 B7

ANALOG
------
+      A0  PA0
-      A1  PA1
RESET  0   PB0

LED
---
LED_red  23  PC7         // LED for Dasa

LoRa Modem
----------
SX1262_RESET, 21, PC5
SX1262_BUSY, 18, PC2
SX1262_DIO1, 19, PC3
CS, 20, PC4
MOSI, 5, PB5
MISO, 6, PB6
SCK, 7, PB7

SD Card Power
-------------
SDpower1  1    // PB1
SDpower2  2    // PB2
SDpower3  3    // PB3

GPS Power
---------
GPSpower  26   // PA2



                     Mighty 1284p    
                      +---\/---+
           (D 0) PB0 1|        |40 PA0 (AI 0 / D24)
           (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
      INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
       PWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
    PWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)
      MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)
  PWM/MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)
   PWM/SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)
                 RST 9|        |32 AREF
                VCC 10|        |31 GND
                GND 11|        |30 AVCC
              XTAL2 12|        |29 PC7 (D 23)
              XTAL1 13|        |28 PC6 (D 22)
      RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI
      TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO
RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS
TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK
     PWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
     PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
     PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM
                      +--------+
*/

#include <SD.h>             // Revised version from MightyCore
#include "wiring_private.h"
#include <Wire.h>           
#include <Adafruit_MPL3115A2.h>
//#include <avr/wdt.h>

//#include <stdio.h>
//#include <stdint.h>

#include <SPI.h>

#define LED_red   23   // PC7
#define RESET     0    // PB0
#define GPSpower  26   // PA2
#define SDpower1  1    // PB1
#define SDpower2  2    // PB2
#define SDpower3  3    // PB3
#define SS        4    // PB4
#define MOSI      5    // PB5
#define MISO      6    // PB6
#define SCK       7    // PB7
#define INT       20   // PC4
#define IOT_RESET 21   // PC5
#define IOT_BUSY  18   // PC2
#define IOT_DIO1  19   // PC3
#define IOT_CS    20   // PC4

uint16_t count = 0;
uint32_t serialhash = 0;
uint8_t lo, hi;
uint16_t u_sensor, maximum;
Adafruit_MPL3115A2 sensor = Adafruit_MPL3115A2();
uint32_t last_packet = 0;
uint16_t hits;

// Read Analog Differential without gain (read datashet of ATMega1280 and ATMega2560 for refference)
// Use analogReadDiff(NUM)
//   NUM  | POS PIN             | NEG PIN           |   GAIN
//  0 | A0      | A1      | 1x
//  1 | A1      | A1      | 1x
//  2 | A2      | A1      | 1x
//  3 | A3      | A1      | 1x
//  4 | A4      | A1      | 1x
//  5 | A5      | A1      | 1x
//  6 | A6      | A1      | 1x
//  7 | A7      | A1      | 1x
//  8 | A8      | A9      | 1x
//  9 | A9      | A9      | 1x
//  10  | A10     | A9      | 1x
//  11  | A11     | A9      | 1x
//  12  | A12     | A9      | 1x
//  13  | A13     | A9      | 1x
//  14  | A14     | A9      | 1x
//  15  | A15     | A9      | 1x
#define PIN 0
uint8_t analog_reference = INTERNAL2V56; // DEFAULT, INTERNAL, INTERNAL1V1, INTERNAL2V56, or EXTERNAL

uint8_t bcdToDec(uint8_t b)
{
  return ( ((b >> 4)*10) + (b%16) );
}


uint32_t tm;
uint8_t tm_s100;

void readRTC()
{
  Wire.beginTransmission(0x51);
  Wire.write(0);
  Wire.endTransmission();
  
  Wire.requestFrom(0x51, 6);
  tm_s100 = bcdToDec(Wire.read());
  uint8_t tm_sec = bcdToDec(Wire.read() & 0x7f);
  uint8_t tm_min = bcdToDec(Wire.read() & 0x7f);
  tm = bcdToDec(Wire.read());
  tm += bcdToDec(Wire.read()) * 100;
  tm += bcdToDec(Wire.read()) * 10000;
  tm = tm * 60 * 60 + tm_min * 60 + tm_sec;
}

#define BQ34Z100 0x55

int16_t readBat(int8_t regaddr)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(regaddr);
  Wire.endTransmission();
  
  Wire.requestFrom(BQ34Z100,1);
  
  unsigned int low = Wire.read();
  
  Wire.beginTransmission(BQ34Z100);
  Wire.write(regaddr+1);
  Wire.endTransmission();
  
  Wire.requestFrom(BQ34Z100,1);
  
  unsigned int high = Wire.read();
  
  unsigned int high1 = high<<8;
  
  return (high1 + low);
}

#define SD_ON     1
#define SD_OFF    2
#define GPS_ON    3
#define GPS_OFF   4
#define LORA_ON   5
#define LORA_OFF  6


void set_power(uint8_t state)
{
  switch(state)
  {
    case SD_ON:
      pinMode(MISO, OUTPUT);     
      pinMode(MOSI, OUTPUT);     
      PORTB |= 0b00001110;
      digitalWrite(SS, LOW);  
      break;
    case SD_OFF:
      digitalWrite(SS, HIGH);  
      PORTB &= 0b11110001;
      break;
    case GPS_ON:
      digitalWrite(GPSpower, HIGH);  
      break;      
    case GPS_OFF:
      digitalWrite(GPSpower, LOW);  
      break;
    case LORA_ON:
      digitalWrite(IOT_CS, LOW);  
      break;
    case LORA_OFF:
      digitalWrite(IOT_CS, HIGH);  
      break;
    default:
      delay(100);
  }
}
  
void setup()
{
  pinMode(SDpower1, OUTPUT);  // SDcard interface
  pinMode(SDpower2, OUTPUT);     
  pinMode(SDpower3, OUTPUT);     
  pinMode(GPSpower, OUTPUT);  // GPS Power    
  pinMode(IOT_CS, OUTPUT);     
  pinMode(SS, OUTPUT);     
  pinMode(MOSI, OUTPUT);     
  pinMode(MISO, OUTPUT);     
  pinMode(SCK, OUTPUT);  
  pinMode(RESET, OUTPUT);   // reset signal for peak detetor

  set_power(SD_OFF);
  set_power(GPS_OFF);
  set_power(LORA_OFF);
  
  //watchdog enable
  //!!!wdt_enable(WDTO_8S);

  Wire.setClock(100000);

  // Open serial communications
  Serial.begin(38400);
  Serial1.begin(38400);

  Serial.println("#Cvak...");
 
  ADMUX = (analog_reference << 6) | ((PIN | 0x10) & 0x1F);
  
  //ADCSRB = 0;               // Switching ADC to Free Running mode
  //sbi(ADCSRA, ADATE);       // ADC autotrigger enable (mandatory for free running mode)
  //sbi(ADCSRA, ADSC);        // ADC start the first conversions
  sbi(ADCSRA, 2);           // 0x111 = clock divided by 128, 125 kHz, 104 us for 13 cycles of one AD conversion, 12 us fo 1.5 cycle for sample-hold
  sbi(ADCSRA, 1);        
  sbi(ADCSRA, 0);        

  pinMode(LED_red, OUTPUT);
  digitalWrite(LED_red, LOW);  
  digitalWrite(RESET, LOW);  
  
  for(int i=0; i<5; i++)  
  {
    delay(50);
    digitalWrite(LED_red, HIGH);  // Blink for Dasa 
    delay(50);
    digitalWrite(LED_red, LOW);  
  }

  // Initiation of RTC
  Wire.beginTransmission(0x51); // init clock
  Wire.write((uint8_t)0x23); // Start register
  Wire.write((uint8_t)0x00); // 0x23
  Wire.write((uint8_t)0x00); // 0x24 Two's complement offset value
  Wire.write((uint8_t)0b00000101); // 0x25 Normal offset correction, disable low-jitter mode, set load caps to 6 pF
  Wire.write((uint8_t)0x00); // 0x26 Battery switch reg, same as after a reset
  Wire.write((uint8_t)0x00); // 0x27 Enable CLK pin, using bits set in reg 0x28
  Wire.write((uint8_t)0x97); // 0x28 stop watch mode, no periodic interrupts, CLK pin off
  Wire.write((uint8_t)0x00); // 0x29
  Wire.write((uint8_t)0x00); // 0x2a
  Wire.endTransmission();
  Wire.beginTransmission(0x51); // reset clock
  Wire.write(0x2f); 
  Wire.write(0x2c);
  Wire.endTransmission();
  Wire.beginTransmission(0x51); // start stop-watch
  Wire.write(0x28); 
  Wire.write(0x97);
  Wire.endTransmission();
  Wire.beginTransmission(0x51); // reset stop-watch
  Wire.write((uint8_t)0x00); // Start register
  Wire.write((uint8_t)0x00); // 0x00
  Wire.write((uint8_t)0x00); // 0x01 
  Wire.write((uint8_t)0x00); // 0x02 
  Wire.write((uint8_t)0x00); // 0x03
  Wire.write((uint8_t)0x00); // 0x04
  Wire.write((uint8_t)0x00); // 0x05
  Wire.endTransmission();
  
  //wdt_reset(); //Reset WDT

  // make a string for device identification output
  String dataString = "$AIRDOS," + FWversion + "," + gittag + ","; // FW version and Git hash

  if (digitalRead(17)) // Protection against sensor mallfunction 
  {
    Wire.beginTransmission(0x58);                   // request SN from EEPROM
    Wire.write((int)0x08); // MSB
    Wire.write((int)0x00); // LSB
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)0x58, (uint8_t)16);    
    for (int8_t reg=0; reg<16; reg++)
    { 
      uint8_t serialbyte = Wire.read(); // receive a byte
      if (serialbyte<0x10) dataString += "0";
      dataString += String(serialbyte,HEX);    
      serialhash += serialbyte;
    }
  }
  else
  {
    dataString += "NaN";    
  }

  Serial.println("#Hmmm...");
  
  {
    set_power(SD_ON);

    delay(100);
    
    // make sure that the default chip select pin is set to output
    // see if the card is present and can be initialized:
    if (!SD.begin(SS)) 
    {
      Serial.println("#Card failed, or not present");
      // don't do anything more:
      return;
    }
  
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
  
    // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.println(dataString);  // write to SDcard (800 ms)     
      dataFile.close();
      digitalWrite(LED_red, HIGH);  // Blink for Dasa
      Serial.println(dataString);  // print SN to terminal 
      digitalWrite(LED_red, LOW);          
    }  
    // if the file isn't open, pop up an error:
    else 
    {
      Serial.println("#error opening datalog.txt");
    }
    
    set_power(SD_OFF);
  }    
  
  hits = 0;
  //!!! wdt_reset(); //Reset WDT
}

void loop()
{
  uint16_t buffer[RANGE];       // buffer for histogram
  uint32_t hit_time[EVENTS];    // time of events
  uint8_t hit_channel[EVENTS];  // energy of events

  //!!!wdt_reset(); //Reset WDT

  {
    // make a string for assembling the data to log:
    String dataString = "";
    int8_t temp = -63;
    
    if (digitalRead(17)) // Protection against sensor mallfunction 
    {
  
      readRTC();
      
      // make a string for assembling the data to log:
      dataString += "$HEALTH,";
  
      dataString += String(count); 
      dataString += ",";
    
      dataString += String(tm); 
      dataString += ".";
      dataString += String(tm_s100); 
      dataString += ",";

      if (! sensor.begin()) 
      {
        dataString += "NaN,NaN";
      }
      else
      {
        float pressure = sensor.getPressure();
        dataString += String(pressure); 
        dataString += ",";
    
        float temperature = sensor.getTemperature();
        dataString += String(temperature); 
        temp = round(temperature);
      }  
    }
    else
    {
      dataString += "$Error";
    }

    int16_t voltage = readBat(8);
    int16_t current = readBat(10);
    
    dataString += ",";
    dataString += String(float(voltage)/1000);   // V - U
    dataString += ",";
    dataString += String(current);  // mA - I
    dataString += ",";
    dataString += String(readBat(4));   // mAh - remaining capacity
    dataString += ",";
    dataString += String(readBat(6));   // mAh - full charge
    dataString += ",";
    dataString += String(readBat(0xc) * 0.1 - 273.15);   // temperature
    dataString += ",";    
    dataString += String(hits);   // number of hits per measurement cycle
    hits = 0;

    Serial.println(dataString);
  }


  // GPS **********************
  set_power(GPS_ON);
//!!! if (false)
  {
     delay(100);

     // make a string for assembling the data to log:
      String dataString = "";

#define MSG_NO 20    // number of logged NMEA messages

    {
      // switch to UTC time; UBX-CFG-RATE (6)+6+(2)=14 configuration bytes
      const char cmd[14]={0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0xE8 ,0x03 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x37};
      for (int n=0;n<(14);n++) Serial1.write(cmd[n]); 
    }
    // flush serial buffer
    while (Serial1.available()) Serial1.read();

    boolean flag = false;
    char incomingByte; 
    int messages = 0;
    uint32_t nomessages = 0;

    while(true)
    {
      if (Serial1.available()) 
      {
        // read the incoming byte:
        incomingByte = Serial1.read();
        nomessages = 0;
        
        if (incomingByte == '$') 
        {
          messages++;   
          //wdt_reset();
        };   // Prevent endless waiting
        if (messages > GPSWAIT) break; // maximal waiting for fix

        if (flag && (incomingByte == '*')) break;
        flag = false;

        if (incomingByte == 'A') flag = true;   // Waiting for FIX
      }
      else
      {
        nomessages++;  
        if (nomessages > GPSerror) break; // preventing of forever waiting
      }
    }
    
    // make a string for assembling the NMEA to log:
    dataString = "";

    flag = false;
    messages = 0;
    nomessages = 0;
    while(true)
    {
      if (Serial1.available()) 
      {
        // read the incoming byte:
        incomingByte = Serial1.read();
        nomessages = 0;
        
        if (incomingByte == '$') {flag = true; messages++;};
        if (messages > MSG_NO)
        {
          readRTC();
                
          dataString += "$TIME,";
          dataString += String(tm); 
          dataString += ".";
          dataString += String(tm_s100); 
          
          break;
        }
        
        // say what you got:
        if (flag && (messages<=MSG_NO)) dataString+=incomingByte;
      }
      else
      {
        nomessages++;  
        if (nomessages > GPSerror) break; // preventing of forever waiting
      }
    }  
    //set_power(GPS_OFF);

    {
        set_power(SD_ON);
        
        // make sure that the default chip select pin is set to output
        // see if the card is present and can be initialized:
        if (!SD.begin(SS)) 
        {
          Serial.println("#Card failed, or not present");
          // don't do anything more:
          return;
        }
        
        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
        
        // if the file is available, write to it:
        if (dataFile) 
        {
          digitalWrite(LED_red, HIGH);  // Blink for Dasa
          dataFile.println(dataString);  // write to SDcard (800 ms)     
          digitalWrite(LED_red, LOW);          
          dataFile.close();
        }  
        // if the file isn't open, pop up an error:
        else 
        {
          Serial.println("#error opening datalog.txt");
        }
        
        set_power(SD_OFF);
    }  
#ifdef DEBUG
    Serial.println(dataString);  // print to terminal (additional 700 ms)
    //!!!wdt_reset(); //Reset WDT
#endif
  }

  
  for(uint16_t i=0; i<(GPSdelay); i++)  // measurements between GPS aquisition
  {
    PORTB = 1;                          // Set reset output for peak detector to H
    ADMUX = (analog_reference << 6) | 0b00000; // Select A0 single ended
    DDRB = 0b10011111;                  // Reset peak detector
    delayMicroseconds(2);              
    DDRB = 0b10011110;
    delayMicroseconds(8);   
               
    for(int n=0; n<RANGE; n++) // clear histogram
    {
      buffer[n]=0;
    }
    uint16_t hit_count = 0;    // clear events
      
    // dosimeter integration
    //for (uint32_t i=0; i<65535*4; i++)    // 
    for (uint32_t i=0; i<100000; i++)    // cca 10.4 s
    //for (uint32_t i=0; i<10000; i++)    // faster for testing
    {
      //wdt_reset(); //Reset WDT

      // start the conversion
      sbi(ADCSRA, ADSC);   
      delayMicroseconds(20); // wait more than 1.5 cycle of ADC clock for sample/hold
      DDRB = 0b10011111;                  // Reset peak detector
      delayMicroseconds(2);              
      DDRB = 0b10011110;
      while (bit_is_clear(ADCSRA, ADIF)); // wait for end of conversion 
  
      // we have to read ADCL first; doing so locks both ADCL
      // and ADCH until ADCH is read.  reading ADCL second would
      // cause the results of each conversion to be discarded,
      // as ADCL and ADCH would be locked when it completed.
      lo = ADCL;
      hi = ADCH;
      sbi(ADCSRA, ADIF);                  // reset interrupt flag from ADC
  
      // combine the two bytes
      u_sensor = (hi << 8) | lo;          // 1024
      //u_sensor = (hi << 7) | (lo >> 1); // 512
      //u_sensor = (hi << 6) | (lo>>2);   // 256
      // manage negative values
      //if (u_sensor <= (CHANNELS/2)-1 ) {u_sensor += (CHANNELS/2);} else {u_sensor -= (CHANNELS/2);}
      //if (u_sensor < ZERO) {u_sensor = 0;} else {u_sensor -= ZERO;}
    
      if (u_sensor <  RANGE)
      {
        buffer[u_sensor]++;
      }
      else
      {
        if (hit_count < EVENTS)
        {
          hit_time[hit_count] = i;
          hit_channel[hit_count] = u_sensor;         
        }
        hit_count++;
      }
      //if (u_sensor > NOISE) hits++;
    }  

    
    // Data out
    // Histogram out
    {
      // make a string for assembling the data to log:
      String dataString = "";
      
      if (digitalRead(17)) // Protection against sensor mallfunction 
      {
    
        readRTC();
        
        // make a string for assembling the data to log:
        dataString += "$HIST,";
    
        dataString += String(count); 
        dataString += ",";
      
        dataString += String(tm); 
        dataString += ".";
        dataString += String(tm_s100); 
        dataString += ",";
  
        if (! sensor.begin()) 
        {
          dataString += "NaN,NaN";
        }
        else
        {
          float pressure = sensor.getPressure();
          dataString += String(pressure); 
          dataString += ",";
      
          float temperature = sensor.getTemperature();
          dataString += String(temperature); 
        }  
      }
      else
      {
        dataString += "$Error";
      }

      dataString += ",";
      dataString += String(float(readBat(8))/1000);   // V - U
      dataString += ",";
      dataString += String(readBat(10));  // mA - I
      dataString += ",";
      dataString += String(readBat(4));   // mAh - remaining capacity
      dataString += ",";
      dataString += String(readBat(6));   // mAh - full charge
      dataString += ",";
      dataString += String(readBat(0xc) * 0.1 - 273.15);   // temperature
            
      for(int n=0; n<RANGE; n++)  
      {
        dataString += ",";
        dataString += String(buffer[n]); 
        //dataString += "\t";
        //if (n==NOISE) dataString += "*,";
      }
       
      count++;

      {        

        set_power(SD_ON);

        // make sure that the default chip select pin is set to output
        // see if the card is present and can be initialized:
        if (!SD.begin(SS)) 
        {
          Serial.println("#Card failed, or not present");
          // don't do anything more:
          return;
        }

        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
      
        // if the file is available, write to it:
        if (dataFile) 
        {
          //digitalWrite(LED_red, HIGH);  // Blink for Dasa
          dataFile.println(dataString);  // write to SDcard (800 ms)     
          //digitalWrite(LED_red, LOW);          
          dataFile.close();
        }  
        // if the file isn't open, pop up an error:
        else 
        {
          Serial.println("#error opening datalog.txt");
        }
  
        set_power(SD_OFF);
      }          
      digitalWrite(LED_red, HIGH);  // Blink for Dasa
      Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
      digitalWrite(LED_red, LOW);                
    }    

    // Hits out
    {
      // make a string for assembling the data to log:
      String dataString = "$HITS,";

      dataString += String(hit_count); 

      if (hit_count > EVENTS) hit_count = EVENTS;
      
      for(uint16_t n=0; n<hit_count; n++)  
      {
        dataString += ",";
        dataString += String(hit_time[n]); 
        dataString += ",";
        dataString += String(hit_channel[n]); 
      }
       
      {
        set_power(SD_ON);

        // make sure that the default chip select pin is set to output
        // see if the card is present and can be initialized:
        if (!SD.begin(SS)) 
        {
          Serial.println("#Card failed, or not present");
          // don't do anything more:
          return;
        }

        // open the file. note that only one file can be open at a time,
        // so you have to close this one before opening another.
        File dataFile = SD.open("datalog.txt", FILE_WRITE);
      
        // if the file is available, write to it:
        if (dataFile) 
        {
          //digitalWrite(LED_red, HIGH);  // Blink for Dasa
          dataFile.println(dataString);  // write to SDcard (800 ms)     
          //digitalWrite(LED_red, LOW);          
          dataFile.close();
        }  
        // if the file isn't open, pop up an error:
        else 
        {
          Serial.println("#error opening datalog.txt");
        }
  
        set_power(SD_OFF);
      }          
      digitalWrite(LED_red, HIGH);  // Blink for Dasa
      Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
      digitalWrite(LED_red, LOW);                
    } 
  }
}
