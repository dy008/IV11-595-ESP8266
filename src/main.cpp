#include <Arduino.h>
#include <SPI.h>
#include <ShiftRegister74HC595.h>

// SN74HC595的引脚连接
// ESP8266 (WemosD1)--->  74HC595
// SCK 12 (D6) --> SRCLK 11
// MOSI 13 (D7) --> SER 14
// SS 15 (D8) --> RCLK 12
// create a global shift register object
// parameters: (number of shift registers, data pin, clock pin, latch pin)
ShiftRegister74HC595 sr (4, 13, 12, 15);
#define  SecondContrl  14   // 秒显控制脚 HiGH = 显示
//uint8_t m_ssdArray[16] = {252, 96, 218, 242, 102, 182, 190, 224, 254, 246, 238, 62, 156, 122, 158, 142};
uint8_t m_ssdArray[16] = {B10110111, B10100000, B00111011, B10111001, B10101100, B10011101, B10011111, B10110000, B10111111, B10111101, 238, 62, 156, 122, 158, 142};

uint8_t BCD_HMS[6] = {0, 0, 0, 0, 0, 0};
uint8_t BCD_YMD[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t BCD_S[4] = {0, 0, 0, 0};
uint8_t DispMode = 0;

#include <NTPClient.h>
//#include <WiFi.h>
#include "ESP8266WiFi.h"
#include <WiFiUdp.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

// NTP
WiFiUDP ntpUDP;
// You can specify the time server pool and the offset, (in seconds)
// additionaly you can specify the update interval (in milliseconds).
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);
NTPClient timeClient(ntpUDP, "pool.ntp.org", +8*3600);
// CONNECTIONS:
// DS3231 SDA --> SDA
// DS3231 SCL --> SCL
// DS3231 VCC --> 3.3v or 5v
// DS3231 GND --> GND
// RTC Libraries
#include <Wire.h>
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);

RtcDateTime currTime;

// ************************************
// ** RTC Setup
// ************************************
void RTC_Update(){
  // Do udp NTP lookup, epoch time is unix time - subtract the 30 extra yrs (946684800UL) library expects 2000
  if (timeClient.update()) {
    unsigned long epochTime = timeClient.getEpochTime()-946684800UL;
    Rtc.SetDateTime(epochTime);
    Serial.println("RTC was Update DateTime from NTP...");
    Serial.println(timeClient.getFormattedTime());
  }else {
    Serial.println("RTC was Update failed...");
  }

}

void RTC_Valid(){
  if (!Rtc.IsDateTimeValid()){
    Serial.println("RTC lost confidence in the DateTime!  Updating DateTime");
  }
  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now.  Updating Date Time");
    Rtc.SetIsRunning(true);
  }
  RTC_Update();
}

// Utility print function
#define countof(a) (sizeof(a) / sizeof(a[0]))
void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];
    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.println(datestring);
}

void setup() {
  pinMode(SecondContrl,OUTPUT_OPEN_DRAIN);
  digitalWrite(SecondContrl,HIGH);  // 显示秒点

  // setting all pins at the same time to either HIGH or LOW
  sr.setAllHigh(); // set all pins HIGH
  delay(1500);

  Serial.begin(115200);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeClock);
  Rtc.SetSquareWavePinClockFrequency(DS3231SquareWaveClock_1Hz);
  Rtc.Begin();
  currTime = Rtc.GetDateTime();
  // 下面将当前时间转换：BIN->7段显示格式数组准备传输
  BCD_HMS[0] = m_ssdArray[currTime.Hour() /10];   // 十位数字
  BCD_HMS[1] = m_ssdArray[currTime.Hour() %10];   // 个位数字
  BCD_HMS[2] = m_ssdArray[currTime.Minute() /10];
  BCD_HMS[3] = m_ssdArray[currTime.Minute() %10];
  BCD_HMS[4] = m_ssdArray[currTime.Second() /10];
  BCD_HMS[5] = m_ssdArray[currTime.Second() %10];
  sr.setAll(&BCD_HMS[0]);

  Serial.println("");
  Serial.println("My DS3231 Memory DateTime is : ");
  printDateTime(currTime);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(wifiManager.autoConnect("dy008IV11CLOCK")) {
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    timeClient.begin();
    delay(3000);
    RTC_Update();
    timeClient.end();
  } else{
    Serial.println("failed to connect and hit timeout");
  }
}

void loop() {

  currTime = Rtc.GetDateTime();
  // 下面将当前时间转换：BIN->7段显示格式数组准备传输
  BCD_HMS[0] = m_ssdArray[currTime.Hour() /10];   // 十位数字
  if ((currTime.Hour() /10) == 0) {
    BCD_HMS[0] = 0;
  }
  BCD_HMS[1] = m_ssdArray[currTime.Hour() %10];   // 个位数字
  BCD_HMS[2] = m_ssdArray[currTime.Minute() /10];
  BCD_HMS[3] = m_ssdArray[currTime.Minute() %10];
  BCD_HMS[4] = BCD_S[2] = m_ssdArray[currTime.Second() /10];
  BCD_HMS[5] = BCD_S[3] = m_ssdArray[currTime.Second() %10];
  BCD_YMD[0] = m_ssdArray[currTime.Year() /1000];  // 千位数字
  BCD_YMD[1] = m_ssdArray[currTime.Year() /100%10];
  BCD_YMD[2] = m_ssdArray[currTime.Year() %100/10];
  BCD_YMD[3] = m_ssdArray[currTime.Year() %100%10];
  BCD_YMD[4] = m_ssdArray[currTime.Month() /10];
  if ((currTime.Month() /10) == 0) {
    BCD_YMD[4] = 0;
  }
  BCD_YMD[5] = m_ssdArray[currTime.Month() %10];
  BCD_YMD[6] = m_ssdArray[currTime.Day() /10];
  BCD_YMD[7] = m_ssdArray[currTime.Day() %10];

  switch (DispMode) {
    case 0:;case 1:;case 2:;case 3:;case 4:;case 5:;case 6:;case 7:;case 8:;case 9:;
    case 10:;case 11:;case 12:;case 13:;case 14:;case 15:;case 16:;case 17:;case 18:;
    case 19:;case 20:;case 21:;case 22:;case 23:;case 24:
      sr.setAll(&BCD_HMS[0]);
      DispMode++;
      break;
    case 25:;case 26:;case 27:
      sr.setAll(&BCD_S[0]);
      DispMode++;
      break;
    case 28:
      digitalWrite(SecondContrl,HIGH);  // 显示秒点
      sr.setAll(&BCD_YMD[0]);
      DispMode++;
      break;
    case 29:
      digitalWrite(SecondContrl,HIGH);  // 显示秒点
      sr.setAll(&BCD_YMD[4]);
      DispMode++;
      break;

    default: sr.setAll(&BCD_HMS[0]);
      DispMode = 0;
  }
  // Reset WDT in case we blocked without yield'ing for a while, but shouldn't be necessary
  ESP.wdtFeed();
  delay(390);
  digitalWrite(SecondContrl,HIGH);  // 显示秒点
  delay(600);
  digitalWrite(SecondContrl,LOW);  // 关闭秒点
  // printDateTime(currTime);
}
