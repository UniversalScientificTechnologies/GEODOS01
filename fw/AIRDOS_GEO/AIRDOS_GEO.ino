#define DEBUG // Please comment it if you are not debugging
String FWversion = "GEO_1024_v3"; // Output data format (multiple files)

#define RANGE 25   // histogram range
#define NOISE  6  // first channel for reporting flux to IoT
#define EVENTS 500 // maximal number of recorded events
#define GPSerror 700000 // number of cycles for waitig for GPS in case of GPS error 
#define MSG_NO 20 // number of recorded NMEA messages
//#define GPSdelay  3   // number of measurements between obtaining GPS position
#define GPSdelay 346   // number of measurements between obtaining GPS position and sending telemetry
                       // 346 = cca 1 h
#define MAXFILESIZE 28000000 // in bytes, 4 MB per day, 28 MB per week, 122 MB per month
#define MAXCOUNT 53000 // in measurement cycles, 7 479 per day, 52353 per week, 224 369 per month
#define MAXFILES 100 // maximal number of files on SD card


// Compiled with: Arduino 1.8.13

/*
  AIRDOS_C for GeoDos

ISP
---
PD0     RX
PD1     TX
RESET#  through 50M capacitor to DTR#

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
#include <avr/wdt.h>
#include "src/TinyGPS++/TinyGPS++.h"

#include <stdio.h>
#include <stdint.h>

#include "src/basicmac/basicmac.h"  // LoRa IoT
#include "src/basicmac/hal/hal.h"
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

String filename = "";
uint16_t fn;
uint16_t count = 0;
uint32_t serialhash = 0;
uint8_t lo, hi;
uint16_t u_sensor, maximum;
Adafruit_MPL3115A2 sensor = Adafruit_MPL3115A2();
uint32_t last_packet = 0;
uint16_t hits;
uint16_t lat_old;
uint16_t lon_old;

// 1290c00806a20090c013a000a0000086
String crystal = "D16x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xD8,0xF7,0x35,0x50,0x0C,0x85,0x10,0xEA,0x5E,0xFA,0xCD,0xEF,0x89,0xBC,0x11,0xAC};
static const u1_t PROGMEM APPSKEY[16] = {0x93,0x67,0xC1,0x39,0xA6,0x1E,0xF3,0x9D,0xDB,0x3A,0xB2,0xA7,0xFA,0x5C,0x52,0x48};
static const u4_t DEVADDR = 0x260BB492; 

/*
// 1290c00806a200908013a000a00000dd
String crystal = "NaI(Tl)-D18x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xA0,0x32,0xE7,0xEC,0x37,0x14,0x83,0x38,0xD5,0xBC,0x33,0x63,0xDE,0x07,0xE8,0x07};
static const u1_t PROGMEM APPSKEY[16] = {0x05,0x23,0x95,0x5B,0x55,0x60,0x26,0x13,0xB7,0xC6,0xAB,0xCD,0x3E,0xE9,0xF4,0xBC};
static const u4_t DEVADDR = 0x260B08E8; 

// 1290c00806a20090b413a000a0000070
String crystal = "NaI(Tl)-D18x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xA5,0x4F,0x8E,0xA4,0x48,0x4E,0xD3,0xAC,0x65,0xCD,0x7D,0x47,0xE5,0xFC,0x1B,0x0D};
static const u1_t PROGMEM APPSKEY[16] = {0x3C,0x1E,0x87,0xA7,0xB1,0x17,0xB0,0x2E,0x06,0xE7,0xF6,0x9F,0x48,0x5E,0x1E,0x02};
static const u4_t DEVADDR = 0x260B0655; 

// 1290c00806a20090c013a000a0000086
String crystal = "D16x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xD8,0xF7,0x35,0x50,0x0C,0x85,0x10,0xEA,0x5E,0xFA,0xCD,0xEF,0x89,0xBC,0x11,0xAC};
static const u1_t PROGMEM APPSKEY[16] = {0x93,0x67,0xC1,0x39,0xA6,0x1E,0xF3,0x9D,0xDB,0x3A,0xB2,0xA7,0xFA,0x5C,0x52,0x48};
static const u4_t DEVADDR = 0x260BB492; 

// 1290c00806a200924413a000a00000b2
String crystal = "NaI(Tl)-D16x30";
static const PROGMEM u1_t NWKSKEY[16] = {0x3C,0x23,0x32,0x61,0x55,0x5A,0x3E,0xA8,0x85,0x38,0xE7,0x3C,0x1C,0xB0,0x00,0xB9};
static const u1_t PROGMEM APPSKEY[16] = {0xA6,0xBB,0x6A,0x5D,0xF5,0x48,0x8C,0x2A,0xA8,0xE1,0xEF,0x20,0xCE,0xFD,0x88,0xFC};
static const u4_t DEVADDR = 0x260B7FBD; 

// 1290c00806a20090e413a000a000007c
String crystal = "NaI(Tl)-D17x40";
static const PROGMEM u1_t NWKSKEY[16] = {0x5E,0x20,0x13,0x24,0xE7,0x43,0x2E,0xDE,0x69,0x9E,0xA5,0xB4,0x4C,0x52,0xA3,0xD9};
static const u1_t PROGMEM APPSKEY[16] = {0x08,0x51,0x2D,0x61,0x85,0x6E,0x4B,0x1E,0xE7,0x87,0x0E,0x13,0x9A,0x6B,0x1F,0xF7};
static const u4_t DEVADDR = 0x260BB491; 

// 1290c00806a200921812a000a0000045
String crystal = "NaI(Tl)-D18x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xCA,0x89,0x44,0x10,0xDA,0x01,0xA0,0xD5,0x5E,0xBA,0x01,0xBE,0xB0,0x33,0x91,0x85};
static const u1_t PROGMEM APPSKEY[16] = {0x4F,0x5F,0xB3,0x12,0xE1,0x16,0x0B,0x91,0xAC,0x78,0x11,0xA1,0x6A,0x00,0x72,0x0A};
static const u4_t DEVADDR = 0x260B06DE; 

// 1290c00806a200905c4aa000a0000013
// Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = {0xBC,0x7C,0x13,0xA1,0xBB,0xE1,0xF0,0x77,0x14,0x9D,0x79,0xB8,0x50,0x2D,0xCC,0xD7};
// App Session Key
static const u1_t PROGMEM APPSKEY[16] = {0x4C,0xB7,0x66,0x21,0xB6,0xAA,0xF1,0xF7,0xEE,0x2E,0xAA,0x7E,0xCC,0x5C,0xFC,0x16};
// Device Address
static const u4_t DEVADDR = 0x260BE73E;

// 1290c00806a200923813a000a00000ec
String crystal = "D16x30";
static const PROGMEM u1_t NWKSKEY[16] = {0x4C,0x4D,0x0E,0x77,0x7B,0x3F,0xAD,0xAC,0x93,0xF2,0x62,0x33,0xE5,0x81,0x6C,0x1B};
static const u1_t PROGMEM APPSKEY[16] = {0x27,0x2C,0x7A,0x13,0xA9,0xC4,0xB6,0x9D,0x9B,0xAE,0x33,0x5F,0xFA,0xA6,0x8B,0x58};
static const u4_t DEVADDR = 0x260BD7C1;

// 1290c00806a20091c057a000a0000036
// Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = {0x3F,0x76,0xD6,0xEB,0xFC,0x9A,0x42,0x2A,0xD9,0x06,0x81,0x59,0x8C,0xAE,0x3E,0x60};
// App Session Key
static const u1_t PROGMEM APPSKEY[16] = {0xD8,0x5F,0x50,0x3E,0x28,0xC2,0xF6,0x61,0xE2,0x81,0x10,0xF5,0xFF,0x90,0x02,0x2B};
// Device Address
static const u4_t DEVADDR = 0x260B4150; 

// 1290c00806a20091e412a000a0000010
String crystal = "NaI(Tl)-D16x30";
static const PROGMEM u1_t NWKSKEY[16] = {0xF2,0x28,0xA1,0xB1,0xF4,0xD9,0xB9,0x89,0xAD,0x30,0x91,0x2B,0xC1,0x87,0x45,0x49};
static const u1_t PROGMEM APPSKEY[16] = {0x9C,0x55,0xE5,0x6C,0x0E,0xBE,0xCD,0x3A,0x29,0xE3,0x79,0x85,0x2C,0xDC,0x33,0x40};
static const u4_t DEVADDR = 0x260B1BEA; 

// 1290c00806a200923c12a000a00000bf
// Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = {0x30,0x6E,0x13,0xCA,0x44,0x89,0xFF,0x31,0x5C,0x0A,0x08,0xED,0xE1,0x67,0x5D,0x4B};
// App Session Key
static const u1_t PROGMEM APPSKEY[16] = {0x4D,0x2F,0xBF,0xA2,0xD3,0x57,0xC6,0x52,0xB0,0xC4,0x74,0xE2,0xE3,0xAE,0xE1,0x7C};
// Device Address
static const u4_t DEVADDR = 0x260BD310;

// geodos01test
// Network Session Key
static const PROGMEM u1_t NWKSKEY[16] = {0xC0,0x1D,0x53,0x11,0xCB,0xDB,0x2C,0x2F,0xA9,0x2B,0xFE,0xA8,0x86,0x37,0x5B,0x8A};
// App Session Key
static const u1_t PROGMEM APPSKEY[16] = {0x6C,0x74,0x49,0x34,0x06,0x7A,0xE2,0xFA,0x2F,0xC6,0x6E,0xD7,0xC7,0xC7,0x39,0x49};
// Device Address
static const u4_t DEVADDR = 0x26013714; 
*/

void os_getJoinEui (u1_t* /* buf */) { }
void os_getDevEui (u1_t* /* buf */) { }
void os_getNwkKey (u1_t* /* buf */) { }

// The region to use, this just uses the first one (can be changed if
// multiple regions are enabled).
u1_t os_getRegion (void) { return LMIC_regionCode(0); }

// Schedule TX every this many milliseconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 20000;

// All pin assignments use Arduino pin numbers (e.g. what you would pass
// to digitalWrite), or LMIC_UNUSED_PIN when a pin is not connected.
const lmic_pinmap lmic_pins = {
    // NSS input pin for SPI communication (required)
    .nss = 20,
    .tx = LMIC_CONTROLLED_BY_DIO2,
    .rx = LMIC_UNUSED_PIN,
    // Radio reset output pin (active high for SX1276, active low for
    // others). When omitted, reset is skipped which might cause problems.
    .rst = 21,
    // DIO input pins.
    //   For SX127x, LoRa needs DIO0 and DIO1, FSK needs DIO0, DIO1 and DIO2
    //   For SX126x, Only DIO1 is needed (so leave DIO0 and DIO2 as LMIC_UNUSED_PIN)
    .dio = {/* DIO0 */ LMIC_UNUSED_PIN, /* DIO1 */ 19, /* DIO2 */ LMIC_UNUSED_PIN},
    // Busy input pin (SX126x only). When omitted, a delay is used which might
    // cause problems.
    .busy = 18,
    .tcxo = LMIC_UNUSED_PIN,
};


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

void(* resetFunc) (void) = 0;//declare reset function at address 0

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

void onLmicEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_SCAN_FOUND:
            Serial.println(F("EV_SCAN_FOUND"));
            break;
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXDONE:
            Serial.println(F("EV_TXDONE"));
            break;
        case EV_DATARATE:
            Serial.println(F("EV_DATARATE"));
            break;
        case EV_START_SCAN:
            Serial.println(F("EV_START_SCAN"));
            break;
        case EV_ADR_BACKOFF:
            Serial.println(F("EV_ADR_BACKOFF"));
            break;
         default:
            Serial.print(F("Unknown event: "));
            Serial.println(ev);
            break;
    }
}

uint8_t iot_message[5];

void send_packet()
{
    LMIC_setTxData2(1, iot_message, sizeof(iot_message), 0);
    Serial.println(F("# Packet queued"));

    last_packet = millis();
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
      Serial1.begin(38400);
      digitalWrite(GPSpower, HIGH);  
      break;      
    case GPS_OFF:
      digitalWrite(GPSpower, LOW);
      Serial1.end();  
      pinMode(10, OUTPUT);  // RX1
      digitalWrite(10, LOW);  
      pinMode(11, OUTPUT);  // TX1   
      digitalWrite(10, LOW);  
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

uint8_t *pack_latlon(uint8_t *p, double vd) {
    uint32_t v = vd * 4194304.0;
    *p++ = v >> 24;
    *p++ = v >> 16;
    *p++ = v >> 8;
    *p++ = v;
    return p;
}

void send_GPS_packet()
{
    uint8_t m[17];
    uint8_t *p = m+1;
    m[0] = 0;

    //p = pack_latlon(p, gps.location.lat());
    //p = pack_latlon(p, gps.location.lng());
    LMIC_setTxData2(1, iot_message, sizeof(iot_message), 0);
    Serial.println(F("# Packet queued"));

    last_packet = millis();
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
  
  /*
  DDRB = 0b10011110;   // SDcard Power OFF
  PORTB = 0b00000001;  
  DDRA = 0b11111100;
  PORTA = 0b00000000;  // Initiating ports
  DDRC = 0b11101100;
  PORTC = 0b00000000;  
  DDRD = 0b11111100;
  PORTD = 0b00000000;  
  */
  //watchdog enable
  //!!!wdt_enable(WDTO_8S);

  Wire.setClock(100000);

  // Open serial communications
  Serial.begin(115200);

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
  String dataString = "$AIRDOS," + FWversion + "," + crystal + ","; // FW version and Git hash

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

  {
    set_power(LORA_ON);
  
    // LMIC init
    os_init(nullptr);

    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
  
    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif
    
    #if defined(CFG_eu868)
    // These are defined by the LoRaWAN specification
    enum {
        EU_DR_SF12 = 0,
        EU_DR_SF11 = 1,
        EU_DR_SF10 = 2,
        EU_DR_SF9 = 3,
        EU_DR_SF8 = 4,
        EU_DR_SF7 = 5,
        EU_DR_SF7_BW250 = 6,
        EU_DR_FSK = 7,
    };
  
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7_BW250)); // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(EU_DR_SF12, EU_DR_SF7));      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(EU_DR_FSK,  EU_DR_FSK));      // g2-band
  
    // TTN uses SF9 at 869.525Mhz for its RX2 window (frequency is
    // default LoRaWAN, SF is different, but set them both to be
    // explicit).
    LMIC.dn2Freq = 869525000;
    LMIC.dn2Dr = EU_DR_SF9;
  
    // Set data rate for uplink
    LMIC_setDrTxpow(EU_DR_SF10, KEEP_TXPOWADJ);
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    // TODO: How to configure these channels? LMIC had LMIC_selectSubBand,
    // but it seems BasicMac only has LMIC_disableChannel.
    #endif
  
    // Disable link check validation
    LMIC_setLinkCheckMode(0);
  
    // Enable this to increase the receive window size, to compensate
    // for an inaccurate clock.  // This compensate for +/- 10% clock
    // error, a lower value will likely be more appropriate.
    //LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
  
    // Queue first packet
    //send_packet();
    set_power(LORA_OFF);
  }
  
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
  
    for (fn = 1; fn<MAXFILES; fn++) // find last file
    {
       filename = String(fn) + ".txt";
       if (SD.exists(filename) == 0) break;
    }
    fn--;
    filename = String(fn) + ".txt";
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open(filename, FILE_WRITE);

    if (dataFile.size() > MAXFILESIZE)
    {
      dataFile.close();
      fn++;
      filename = String(fn) + ".txt";      
      dataFile = SD.open(filename, FILE_WRITE);
    }
  
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
  
  Serial.println("#Hmmm...");

  hits = 0;
  lat_old = 0;
  lon_old = 0;
  //!!! wdt_reset(); //Reset WDT
}

uint8_t TelemetryCounts = 0;

void loop()
{
  uint16_t buffer[RANGE];       // buffer for histogram
  uint32_t hit_time[EVENTS];    // time of events
  uint16_t hit_channel[EVENTS];  // energy of events

  //!!!wdt_reset(); //Reset WDT

  if (TelemetryCounts++ < 24)
  {
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
      iot_message[0] = round(voltage / 10) - 175;
      int16_t current = readBat(10);
      iot_message[1] = current;
      iot_message[2] = (current >> 8) & 0x01;
      iot_message[2] |= temp << 1;
      iot_message[3] = hits & 0xff;
      iot_message[4] = hits >> 8 ;
  
      /* DEBUG
      Serial.println();
      for(int i=0; i<5; i++)
      {
        Serial.println(iot_message[i],HEX);
      }
      */
      
      dataString += ",";
      dataString += String(float(voltage)/1000);   // V - U
      dataString += ",";
      dataString += String(current);  // mA - I
      dataString += ",";
      dataString += String(readBat(4));   // mAh - remaining capacity
      dataString += ",";
      dataString += String(readBat(6));   // mAh - full charge
      dataString += ",";
      dataString += String(hits);   // number of hits per measurement cycle
  
      // send_packet over LoRa IoT
      set_power(LORA_ON);
      // Let LMIC handle background tasks for LoRa IoT
      os_runstep();
      send_packet();
      Serial.println(dataString);
      os_runstep();
  
      for(int i=0; i<50; i++)  
      {
        delay(50);
        digitalWrite(LED_red, HIGH);  // Blink for Dasa 
        delay(50);
        digitalWrite(LED_red, LOW);  
        os_runstep();
      }
  
      set_power(LORA_OFF);
    }
    hits = 0;

  }
  else
  {
    TelemetryCounts = 0;
    
    // GPS **********************
    set_power(GPS_ON);
    delay(30);  
    {
      // switch to UTC time; UBX-CFG-RATE (6)+6+(2)=14 configuration bytes
      const char cmd[14]={0xB5 ,0x62 ,0x06 ,0x08 ,0x06 ,0x00 ,0xE8 ,0x03 ,0x01 ,0x00 ,0x00 ,0x00 ,0x00 ,0x37};
      for (int n=0;n<(14);n++) Serial1.write(cmd[n]); 
    }
  
    {
      TinyGPSPlus gps;
      int16_t gpschars = 0;
      while (true)
      {
        if (Serial1.available() > 0) 
        {
          gps.encode(Serial1.read());
          if (gpschars++ > 20000) break;
          if (gps.location.isUpdated()) break;
        }
      } 
  
      Serial.print("#");
      Serial.print(gps.charsProcessed());
      Serial.print(" Lat ");
      Serial.print(gps.location.rawLat().deg);
      Serial.print(" ");
      Serial.print(gps.location.rawLat().billionths);
      Serial.print(", Lon ");
      Serial.print(gps.location.rawLng().deg);
      Serial.print(" ");
      Serial.println(gps.location.rawLng().billionths);
  
      uint32_t lat = round(gps.location.lat()*10000);
      uint32_t lon = round(gps.location.lng()*10000);
      uint16_t lat_short = lat % 65535;
      uint16_t lon_short = lon % 65535;
      int lat_diff = lat_short - lat_old;
      int lon_diff = lon_short - lon_old;
  
      Serial.print("#latdiff ");
      Serial.print(lat_diff);
      Serial.print(" londiff ");
      Serial.println(lon_diff);
   
      if ((abs(lat_diff)>2) || (abs(lon_diff)>4)) // movement detection
      {
        lat_old = lat_short;
        lon_old = lon_short;
              
        iot_message[0] = 0xff;
        iot_message[1] = lat_short & 0xff;
        iot_message[2] = lat_short >> 8;
        iot_message[3] = lon_short & 0xff;
        iot_message[4] = lon_short >> 8 ;
          
        // send_packet over LoRa IoT
        set_power(LORA_ON);
  
        // Let LMIC handle background tasks for LoRa IoT
        for(int i=0; i<50; i++)  
        {
          delay(50);
          digitalWrite(LED_red, HIGH);  // Blink for Dasa 
          delay(50);
          digitalWrite(LED_red, LOW);  
          os_runstep();
        }  
  
        os_runstep();
        send_packet();
        os_runstep();
    
        for(int i=0; i<100; i++)  
        {
          delay(50);
          digitalWrite(LED_red, HIGH);  // Blink for Dasa 
          delay(50);
          digitalWrite(LED_red, LOW);  
          os_runstep();
        }  
        set_power(LORA_OFF);
      }
    }
  
    {    
      // make a string for assembling the NMEA to log:
      String dataString = "";
  
      char incomingByte;
      bool flag = false;
      uint8_t messages = 0;
      uint32_t nomessages = 0;
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
      set_power(GPS_OFF);
  
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
          File dataFile = SD.open(filename, FILE_WRITE);
          
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
      Serial.println(dataString);  // print to terminal 
      //!!!wdt_reset(); //Reset WDT
  #endif
    }
  }
   
  for(uint16_t i=0; i<(GPSdelay); i++)  // measurements between telemetry sendings
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
      
      if (u_sensor > NOISE) hits++;
      
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
      dataString += String(hits);   
            
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
        File dataFile = SD.open(filename, FILE_WRITE);
      
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
      Serial.println(dataString);   // print to terminal
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
        File dataFile = SD.open(filename, FILE_WRITE);
      
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
      Serial.println(dataString);   // print to terminal 
      digitalWrite(LED_red, LOW);                

      if (count > MAXCOUNT) resetFunc(); //call reset 

    } 
  }
}
