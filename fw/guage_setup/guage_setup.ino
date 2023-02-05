// Compiled with: Arduino 1.8.13

/*
  Battery Guage setup for BQ34Z100-G1

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


#include "wiring_private.h"
#include <Wire.h>

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

#define BQ34Z100 0x55

String dataString;

// read words by standard commands
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

// Read one byte from battery guage flash
uint8_t ReadFlashByte(uint8_t fclass, uint8_t foffset)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x61);                   // start access to flash memory
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);   // memory Subclass
  Wire.write(0x3E);
  Wire.write(fclass);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);   // memory 32 bytes page
  Wire.write(0x3F);
  Wire.write(foffset / 32);
  Wire.endTransmission();

  uint16_t fsum = 0;                  // compute checksum of 32 bytes RAM block
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,32);
  for (uint8_t addr=0; addr<32; addr++)
  {
    uint8_t tmp = Wire.read();
    fsum += tmp;
  }
  fsum = (0xFF^fsum) & 0xFF;          // invert bits

  Wire.beginTransmission(BQ34Z100);   // read specific byte
  Wire.write(0x40+(foffset % 32));
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,1);
  uint8_t value = Wire.read();

  Wire.beginTransmission(BQ34Z100);   // read of chcecksum of RAM block
  Wire.write(0x60);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,1);
  uint8_t v = Wire.read();

  return (value);
}

// Write one byte to battery guage flash
void WriteFlashByte(uint8_t fclass, uint8_t foffset, uint8_t fbyte)
{
  for(uint8_t xx=0; xx<4; xx++) // I do not now why
  {
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x61);                   // start access to flash memory
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3E);                   // memory Subclass
    Wire.write(fclass);
    Wire.endTransmission();
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3F);                   // memory 32 bytes page
    Wire.write(foffset / 32);
    Wire.endTransmission();


    uint16_t fsum = 0;                  // Compute chcecksum of readed block
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40);
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100,32);
    for (uint8_t addr=0; addr<32; addr++)
    {
      uint8_t tmp = Wire.read();
      fsum += tmp;
    }
    fsum = (0xFF^fsum) & 0xFF;

    Wire.beginTransmission(BQ34Z100);   // Rewrite specific byte in readed block
    Wire.write(0x40+(foffset % 32));
    Wire.write(fbyte);
    Wire.endTransmission();

    fsum = 0;                               // Compute new chcecksum
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40);
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100,32);
    for (uint8_t addr=0; addr<32; addr++)
    {
      uint8_t tmp = Wire.read();
      fsum += tmp;
    }
    fsum = (0xFF^fsum) & 0xFF;

    delay(100);
    Wire.beginTransmission(BQ34Z100);   // Write new checksum and those rewrite flash
    Wire.write(0x60);
    Wire.write(fsum);
    Wire.endTransmission();
    delay(100);
  }
}

// Reset battery guage (0x0041)
void ResetGuage()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x01);
  Wire.write(0x41);
  Wire.endTransmission();
}

int8_t readb(int8_t command)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(command);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,1);
  uint8_t low = Wire.read();

}

void PrintBatteryStatus()
{
  dataString = "$GUAGE, ";

  dataString += String(readBat(0x8));   // mV - U
  dataString += " mV, ";
  dataString += String(readBat(0xa));  // mA - I
  dataString += " mA, ";
  dataString += String(readBat(0x4));   // mAh - remaining capacity
  dataString += " mAh, ";
  dataString += String(readBat(0x6));   // mAh - full charge
  dataString += " mAh full, ";
  dataString += String(readBat(0xc) * 0.1 - 273.15);   // temperature
  dataString += " C";

  digitalWrite(LED_red, HIGH);  // Blink for Dasa
  Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
  digitalWrite(LED_red, LOW);
}


void setup()
{
  Wire.setClock(100000);

  // Open serial communications
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.println("#Cvak...");

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

  Serial.println("#Hmmm...");

}

void loop()
{
  PrintBatteryStatus();
  Serial.println();

  /* old version BQ34Z100
  dataString = "$FLASH,";
  dataString += char(ReadFlashByte(48, 68));
  dataString += char(ReadFlashByte(48, 69));
  dataString += char(ReadFlashByte(48, 70));
  dataString += char(ReadFlashByte(48, 71));
  dataString += ",";
  dataString += char(readb(0x63));
  dataString += char(readb(0x64));
  dataString += char(readb(0x65));
  dataString += char(readb(0x66));
  dataString += char(readb(0x67));
  dataString += char(readb(0x68));
  dataString += char(readb(0x69));
  dataString += char(readb(0x6a));
  */

  dataString = "$FLASH,";
  for (uint8_t n=32; n<43; n++) dataString += char(ReadFlashByte(48, n));   // Part type
  dataString += ",";
  for (uint8_t n=44; n<55; n++) dataString += char(ReadFlashByte(48, n));   // Manufacturer
  dataString += ",";
  for (uint8_t n=56; n<60; n++) dataString += char(ReadFlashByte(48, n));   // Chemistry

  digitalWrite(LED_red, HIGH);  // Blink for Dasa
  Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
  digitalWrite(LED_red, LOW);

  Serial.print("LED CONF: ");
  Serial.println(ReadFlashByte(64,4), HEX);
  Serial.print("Design Capacity: ");
  Serial.println(ReadFlashByte(48,11)*256 + ReadFlashByte(48,12));
  Serial.print("Design Energy: ");
  Serial.println(ReadFlashByte(48,13)*256 + ReadFlashByte(48,14));
  Serial.print("Bat. low alert: ");
  Serial.print("Cell BL Set Volt Threshold: ");
  Serial.println(ReadFlashByte(49,8)*256 + ReadFlashByte(49,9));
  Serial.print("Cell BL Clear Volt Threshold: ");
  Serial.println(ReadFlashByte(49,11)*256 + ReadFlashByte(49,12));
  Serial.print("Cell BL Set Volt Time: ");
  Serial.println(ReadFlashByte(49,10));
  Serial.print("Cell BH Set Volt Threshold: ");
  Serial.println(ReadFlashByte(49,13)*256 + ReadFlashByte(49,14));
  Serial.print("Cell BH Volt Time: ");
  Serial.println(ReadFlashByte(49,15));
  Serial.print("Cell BH Clear Volt Threshold: ");
  Serial.println(ReadFlashByte(49,16)*256 + ReadFlashByte(49,17));
  Serial.print("Cycle Delta: ");
  Serial.println(ReadFlashByte(49,21));
  WriteFlashByte(64,4,0x63);    // 7+1 LEDs and I2C interface
  WriteFlashByte(48,11,0x0d);   // Design Capacity 3500 mAh = 0xbb8 (default 1000 mAh = 0x3e8)
  WriteFlashByte(48,12,0xac);   // Must be written in this order, I do not know why
  WriteFlashByte(48,13,0x31);   // Design Energy 12600 mWh = 0x3138 (default 5400 mWh = 0x1518)
  WriteFlashByte(48,14,0x38);
  //WriteFlashByte(83,0,0x20);   // Set Chem ID  TODO, check register adresses. 
  //WriteFlashByte(83,1,0x59);   //

  WriteFlashByte(64,5,0x10);    // Battery Low voltage ALERT
  WriteFlashByte(64,6,0x00);   
  //WriteFlashByte(49,8,0x0a);    // Battery set low voltage threshold 2750 mV = 0x0abe (default 0 mV = 0x0000)
  //WriteFlashByte(49,9,0xbe);   
  WriteFlashByte(49,8,0x0c);    // Battery set low voltage threshold 3300 mV = 0x0ce4 (default 0 mV = 0x0000)
  WriteFlashByte(49,9,0xe4);   
  WriteFlashByte(49,11,0x0e);   // Battery clear low voltage threshold 3635 mV = 0x0e33 (default 5 mV = 0x0005)
  WriteFlashByte(49,12,0x33);
  WriteFlashByte(49,10,0x32);   // Battery set low voltage time hreshold 50 s = 0x32 (default 0 s = 0x00, max. 60 s)
  //ResetGuage();
  delay(100);
  Serial.print("LED CONF: ");
  Serial.println(ReadFlashByte(64,4), HEX);
  Serial.print("Design Capacity: ");
  Serial.println(ReadFlashByte(48,11)*256 + ReadFlashByte(48,12));
  Serial.print("Design Energy: ");
  Serial.println(ReadFlashByte(48,13)*256 + ReadFlashByte(48,14));
  Serial.print("Bat. low alert: ");
  Serial.print("Cell BL Set Volt Threshold: ");
  Serial.println(ReadFlashByte(49,8)*256 + ReadFlashByte(49,9));
  Serial.print("Cell BL Clear Volt Threshold: ");
  Serial.println(ReadFlashByte(49,11)*256 + ReadFlashByte(49,12));
  Serial.print("Cell BL Set Volt Time: ");
  Serial.println(ReadFlashByte(49,10));
  Serial.print("Cell BH Set Volt Threshold: ");
  Serial.println(ReadFlashByte(49,13)*256 + ReadFlashByte(49,14));
  Serial.print("Cell BH Volt Time: ");
  Serial.println(ReadFlashByte(49,15));
  Serial.print("Cell BH Clear Volt Threshold: ");
  Serial.println(ReadFlashByte(49,16)*256 + ReadFlashByte(49,17));
  Serial.print("Cycle Delta: ");
  Serial.println(ReadFlashByte(49,21));

  ResetGuage();

  while(true)
  {
    PrintBatteryStatus();
    delay(1000);
  }
}
