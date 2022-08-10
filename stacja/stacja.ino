#include <SPI.h>
#include <OneWire.h>

#include "LCD.h"
#include "DEV_Config.h"
#include "DFRobot_BMP280.h"
#include "Wire.h"

typedef DFRobot_BMP280_IIC    BMP;    // ******** use abbreviations instead of full names ********

BMP   bmp(&Wire, BMP::eSdoLow);

// show last sensor operate status
void printLastOperateStatus(BMP::eStatus_t eStatus)
{
  switch(eStatus) {
  case BMP::eStatusOK:    Serial.println("everything ok"); break;
  case BMP::eStatusErr:   Serial.println("unknow error"); break;
  case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
  case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_DC, OUTPUT);

  Serial.begin(115200);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  LCD_SCAN_DIR Lcd_ScanDir = SCAN_DIR_DFT;
  Serial.println("LCD Init ");
  LCD.LCD_Init(Lcd_ScanDir);

  Serial.println("LCD_Show ");
  //LCD.LCD_Show();
  LCD.LCD_Clear(BLACK);
  // barometr
  bmp.reset();
  Serial.println("bmp read data test");
  while(bmp.begin() != BMP::eStatusOK) {
    Serial.println("bmp begin faild");
    printLastOperateStatus(bmp.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bmp begin success");
  delay(100);
}

void loop()
{
  Wyswietl();
  delay(2000);
  
}
void Wyswietl(){
  float tmp1 =  temperatura(3);
  float tmp2 =  temperatura(5);
  
  Serial.println(tmp1);Serial.print(" tmp1");
  Serial.println(tmp2);Serial.print(" tmp2");
  
  String wew = "Wew: " + String(tmp1) + String("*C");
  String zew = "Zew: " + String(tmp2) + String("*C");
  
  char com1[wew.length()];
  char com2[zew.length()];
  
  wew.toCharArray(com1,wew.length());
  zew.toCharArray(com2,zew.length());
  
  uint32_t    press = bmp.getPressure();
  Serial.print("pressure (unit hpa): "); Serial.println(press/100.0);

  String hpa = "Hpa: " + String(press/100.0);
  char hpa_w[hpa.length()];
  hpa.toCharArray(hpa_w,hpa.length());
  
  LCD.LCD_DisplayString(5,5,com1,&Font16,BLACK,WHITE);
  LCD.LCD_DisplayString(5,25,com2,&Font16,BLACK,WHITE);
  LCD.LCD_DisplayString(5,45,hpa_w,&Font16,BLACK,WHITE);
}

float temperatura(int x)
{
  OneWire  ds(x);
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    ds.reset_search();
    return;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {return;}
  
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      type_s = 1;
      break;
    case 0x28:
      type_s = 0;
      break;
    case 0x22:
      type_s = 0;
      break;
    default:
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
    
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for ( i = 0; i < 9; i++) {data[i] = ds.read();}

  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  }
  celsius = (float)raw / 16.0;
  return celsius;
}
