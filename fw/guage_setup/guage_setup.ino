// Compiled with: Arduino 1.8.13

/*
  Battery Guage setup

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

Time Synchronisation
--------------------
SYNC0   18  PC2
SYNC1   19  PC3


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
#define SYNC0     18   // PC2
#define SYNC1     19   // PC3



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

void ReadFlash()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3F);
  Wire.write(55 / 32 );
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3E);
  Wire.write(48);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x61);
  Wire.write(0);
  Wire.endTransmission();
  
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
 
  Wire.requestFrom(BQ34Z100,32);
  Serial.println( );
  for(int n=0;n<32;n++)
  {
    Serial.print( char(Wire.read()));
    //Serial.print( Wire.read(),HEX);
    Serial.print(",");
  }
  Serial.println( );
  Wire.requestFrom(BQ34Z100,32);
  Serial.println( );
  for(int n=0;n<32;n++)
  {
    Serial.print( char(Wire.read()));
    Serial.print(",");
  }
  Serial.println( );
 }  

uint8_t ReadFlashByte(uint8_t fclass, uint8_t foffset)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x61);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3E);
  Wire.write(fclass);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3F);
  Wire.write(foffset / 32);
  Wire.endTransmission();
    
  /*
  Serial.print(fclass);
  Serial.print("#");
  Serial.print(foffset / 32);
  Serial.print("#");
  Serial.print(foffset % 32);
  Serial.print("#");
  */
  
  uint16_t fsum = 0;
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,32);
  for (uint8_t addr=0; addr<32; addr++)
  {      
    uint8_t tmp = Wire.read();      
    fsum += tmp;
    //Serial.print(tmp, HEX);
    //Serial.print(",");
  }
  //Serial.println (fsum, HEX);
  fsum = (0xFF^fsum) & 0xFF;
  //Serial.println (fsum, HEX);

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40+(foffset % 32));
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,1);
  uint8_t value = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x60);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,1);
  uint8_t v = Wire.read();
    
  /*
  Serial.print (v, HEX);
  Serial.println (" sum ");
  Serial.print (value, HEX);
  Serial.println (" value ");
  */
  
  return (value);
}

void WriteFlashByte(uint8_t fclass, uint8_t foffset, uint8_t fbyte)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x61);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3E);
  Wire.write(fclass);
  Wire.endTransmission();
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x3F);
  Wire.write(foffset / 32);
  Wire.endTransmission();
    
  
  Serial.print(fclass);
  Serial.print("#");
  Serial.print(foffset / 32);
  Serial.print("#");
  Serial.print(foffset % 32);
  Serial.print("#");

  uint16_t fsum = 0;
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,32);
  for (uint8_t addr=0; addr<32; addr++)
  {      
    uint8_t tmp = Wire.read();      
    fsum += tmp;
    Serial.print(tmp, HEX);
    Serial.print(",");
  }
  //Serial.println (fsum, HEX);
  fsum = (0xFF^fsum) & 0xFF;
  Serial.println (fsum, HEX);

  
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40+(foffset % 32));
  Wire.write(fbyte);
  Wire.endTransmission();

  fsum = 0;
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x40);
  Wire.endTransmission();
  Wire.requestFrom(BQ34Z100,32);
  for (uint8_t addr=0; addr<32; addr++)
  {      
    uint8_t tmp = Wire.read();      
    fsum += tmp;
    Serial.print(tmp, HEX);
    Serial.print(",");
  }
  //Serial.println (fsum, HEX);
  fsum = (0xFF^fsum) & 0xFF;
  Serial.println (fsum, HEX);
  
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x60);
  Wire.write(fsum);
  Wire.endTransmission();
}

void ResetGuage()
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(0x00);
  Wire.write(0x00);
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
  String dataString = "$FLASH,";
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

  digitalWrite(LED_red, HIGH);  // Blink for Dasa
  Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
  digitalWrite(LED_red, LOW);                

  Serial.print("CHEM ID: ");
  Serial.println(ReadFlashByte(83,0)+ReadFlashByte(83,1)*256, HEX); 
  Serial.print("LED CONF: ");
  Serial.println(ReadFlashByte(64,4), HEX); 
  WriteFlashByte(64,4,0x64);
  delay(100);
  ResetGuage();
  delay(100);
  Serial.print("LED CONF: ");
  Serial.println(ReadFlashByte(64,4), HEX); 

  while(true)
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
    dataString += " C, ";
    dataString += String(readBat(0x7a),HEX);   // Chemistry
    dataString += String(readBat(0x7c),HEX);   
    dataString += " CHEM";
  
    digitalWrite(LED_red, HIGH);  // Blink for Dasa
    Serial.println(dataString);   // print to terminal (additional 700 ms in DEBUG mode)
    digitalWrite(LED_red, LOW);  

    delay(1000);
  }
}
