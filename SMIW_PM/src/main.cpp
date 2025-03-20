#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLECharacteristic.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

#include "Wire.h"
#include "SoftwareSerial.h"
#include "PMS.h"

#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include "SparkFun_APDS9960.h"

#include <Fonts/FreeSansBold24pt7b.h>
#include "pictures.h"

#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>  

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;  

//TFT ILI9341 PINS
#define TFT_CS    D0  // Chip Select (CS)
#define TFT_RST   D1  // Reset Pin (RST)
#define TFT_DC    D2  // Data/Command Pin (DC)
#define PINK    0xf81f

//APDS PINS
#define APDS9960_INT    D3 

//Face defs
#define NONE_FACE                            -1
#define GOOD_FACE                             0
#define MODERATE_FACE                         1
#define UNHEALTHY_FOR_SENSITIVE_GROUPS_FACE   2
#define UNHEALTHY_FACE                        3
#define VERY_UNHEALTHY_FACE                   4

#define SCREEN_CHANGED_NEED_TO_DISPLAY_PMS   -1

#define RTC_NOT_CONFIGURED                    -1
#define RTC_BEGIN                              0
#define RTC_BLUETOOTH_INIT                     1
#define RTC_BLUETOOTH_INITED                   2
#define RTC_BLUETOOTH_CONNECTED                3
#define RTC_BLUETOOTH_PASSWORD_SENT_INIT_WIFI  4
#define RTC_WIFI_CONNECTED                     5
#define RTC_CONFIGURED                         6

#define SERVICE_UUID        "50961397-e038-477e-9511-efdd7c6e4caf"
#define CHARACTERISTIC_UUID "0e2f45f7-9a9e-41f4-be6c-168db8ce4eae"

#define HAPPY_FACE                            1
#define NOT_HAPPY_NOR_SAD                     0
#define SAD_FACE                             -1

//Gesture Flag
volatile int isr_flag = 0;

//Last hour
uint8_t last_hour = 25;

//If RTC not configured, don't save measurements
volatile int RTC_Configuration_Status;
int BLE_Enable; //first up check, confirm

//Time variiables
unsigned long previousMillis = 0;  
const long interval = 1000;  

// Gestture sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();

//Display
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
HardwareSerial pmsSerialHard(1);

//PMS module
PMS pms(pmsSerialHard);
PMS::DATA data;

uint8_t verticalScreen;
uint8_t currentScreen;
bool screenChanged;

int8_t previousFace;
int16_t previousPM;

uint16_t photoBuffer[272 * 152];
uint16_t PM1Buffer[108 * 32];
uint16_t PM2_5Buffer[108 * 32];
uint16_t PM10Buffer[108 * 32];

//Wifi 
std::string ssid;
std::string password;

BLEServer* pServer;

//NTP
const char* ntpServer = "pool.ntp.org";      
const long gmtOffset_sec = 3600;             
const int daylightOffset_sec = 3600; 

//AHT, BMP
float pressure = 0;
sensors_event_t humidity, temp;

//Variables for time
unsigned long startMillis = 0;
unsigned long endMillis = 0;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if(value.length() > 3)
    {
      if('i' == value[0] && 'd' == value[1] && value[2] == ':') ssid = value.substring(3, value.length()).c_str();
      if('p' == value[0] && 'd' == value[1] && value[2] == ':') password = value.substring(3, value.length()).c_str();

      Serial.println(ssid.length());
      Serial.print(password.length());
    }
    if (value.length() > 0) {
      Serial.println("*********");
      Serial.print("New value: ");
      for (int i = 0; i < value.length(); i++) {
        Serial.print(value[i]);
      }

      Serial.println();
      Serial.println("*********");
    }
  }
};

uint16_t PM1_Hour_Buffer[60 * 60];
uint16_t PM2_5_Hour_Buffer[60 * 60];
uint16_t PM10_Hour_Buffer[60 * 60];

uint16_t  PM1_Average[24];
uint16_t  PM2_5_Average[24];
uint16_t  PM10_Average[24];



void drawColorCircle(uint16_t x, uint16_t y, uint8_t r, uint16_t color, uint8_t face)
{
tft.fillCircle(x,y,r,color);
}

void displayPM10Circle()
{
  uint16_t currentPM10 = data.PM_AE_UG_10_0;

  if(currentPM10 <=50)
  {
    if(previousFace != GOOD_FACE) drawColorCircle(220,140,40,ILI9341_GREEN, HAPPY_FACE);
  }
  else if(currentPM10 <=80)
  {
    if(previousFace != MODERATE_FACE) drawColorCircle(220,140,40,ILI9341_YELLOW, NOT_HAPPY_NOR_SAD);
  }
  else if(currentPM10 <=110)
  {
    if(previousFace != UNHEALTHY_FOR_SENSITIVE_GROUPS_FACE) drawColorCircle(220,140,40,ILI9341_ORANGE, SAD_FACE);
  }
    else if(currentPM10 <=150)
  {
    if(previousFace != UNHEALTHY_FACE)  drawColorCircle(220,140,40,ILI9341_RED, SAD_FACE);
  }
  else
  {
    if(previousFace != VERY_UNHEALTHY_FACE) drawColorCircle(220,140,40,ILI9341_PURPLE, SAD_FACE);
  }
}

void displayPM2_5Circle()
{
  uint16_t currentPM2_5 = data.PM_AE_UG_2_5;

  if(currentPM2_5 <=35)
  {
    if(previousFace != GOOD_FACE) drawColorCircle(220,140,40,ILI9341_GREEN, HAPPY_FACE);
  }
  else if(currentPM2_5 <=55)
  {
    if(previousFace != MODERATE_FACE) drawColorCircle(220,140,40,ILI9341_YELLOW, NOT_HAPPY_NOR_SAD);
  }
  else if(currentPM2_5 <=75)
  {
    if(previousFace != UNHEALTHY_FOR_SENSITIVE_GROUPS_FACE) drawColorCircle(220,140,40,ILI9341_ORANGE, SAD_FACE);
  }
    else if(currentPM2_5 <=110)
  {
    if(previousFace != UNHEALTHY_FACE)  drawColorCircle(220,140,40,ILI9341_RED, SAD_FACE);
  }
  else
  {
    if(previousFace != VERY_UNHEALTHY_FACE) drawColorCircle(220,140,40,ILI9341_PURPLE, SAD_FACE);
  }
}

void init_BLE()
{
  Serial.println("Before BLE init");
  BLEDevice::init("MyESP32");
  Serial.println("After BLE init");

  pServer = BLEDevice::createServer();
	
  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic =
  pService->createCharacteristic(CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  Serial.println("After start advertising");
}

void deinit_BLE() {
  Serial.println("Before deinit BLE");
  if (pServer->getConnectedCount()) {
    pServer->disconnect(0); 
    delay(100);            
  }

  if (pServer) 
  {
    BLEAdvertising *pAdvertising = pServer->getAdvertising();

    if (pAdvertising) 
    {
        pAdvertising->stop();
    }

    pServer->removeService(pServer->getServiceByUUID(SERVICE_UUID));
    delete pServer;
    pServer = nullptr;
  }

  BLEDevice::deinit(false);
	delay(1000);
  Serial.println("BLE deinitialized.");
}

void display_RTC_Configuration()
{
  screenChanged = false;
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.setCursor(30, 40);
  tft.print("RTC was not configured");
  tft.setCursor(30, 60);
  tft.print("Swipe up 2 times to ");
  tft.setCursor(30, 80);
  tft.print("connect to BLE ");
  tft.setCursor(30, 100);
  tft.print("And send wifi");
  tft.setCursor(30, 120);
  tft.print("ssid and password");
}

void enable_BLE_display()
{
  screenChanged = false;
  tft.setFont(NULL);
  tft.setTextSize(2);
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.setCursor(30, 40);
  tft.print("BLE initialized.");
  tft.setCursor(30, 60);
  tft.print("Waiting for connection");
  init_BLE();
  RTC_Configuration_Status = RTC_BLUETOOTH_INITED;
}

void display_history()
{
  
  tft.setFont(NULL);
  tft.setTextSize(1);
  screenChanged = false;
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.setCursor(30, 30);
  tft.print("History");

  tft.setCursor(80, 40);
  tft.print("PM1");
	tft.setCursor(140, 40);
	tft.print("PM2.5");
	tft.setCursor(200, 40);
  tft.print("PM10");
  tft.setTextSize(1);

  if(verticalScreen == 0)
  {
    for(int i = 0; i < 12; i++)
    {
      tft.setCursor(30,60 + i * 12);
      tft.print(i);
      tft.print(":00");

      tft.setCursor(80, 60 + i * 12);
      tft.print(PM1_Average[i]);

      tft.setCursor(140, 60 + i * 12);
      tft.print(PM2_5_Average[i]);

      tft.setCursor(200, 60 + i * 12);
      tft.print(PM10_Average[i]);
    }
  }
  else if(verticalScreen == 1)
  {
    for(int i = 12; i < 24; i++)
    {
      tft.setCursor(30,60 + (i-12) * 12);
      tft.print(i);
      tft.print(":00");

      tft.setCursor(80, 60 + (i-12) * 12);
      tft.print(PM1_Average[i]);

      tft.setCursor(140, 60 + (i-12) * 12);
      tft.print(PM2_5_Average[i]);

      tft.setCursor(200, 60 + (i-12) * 12);
      tft.print(PM10_Average[i]);
    }
  }
}

void display_PM1_0_label()
{
  screenChanged = false;
  previousFace = NONE_FACE;
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.drawRGBBitmap(40,40, PM1Buffer, 108, 32);
  tft.fillRect(155, 40, 60, 40, ILI9341_BLACK);
  previousPM = SCREEN_CHANGED_NEED_TO_DISPLAY_PMS; 
}

void display_current_PM1_0()
{
  if(previousPM != data.PM_AE_UG_1_0)
  {
    tft.fillRect(40, 116, 104, 36, ILI9341_BLACK);
    tft.setCursor(40, 150);
    tft.println(data.PM_AE_UG_1_0);
    previousPM = data.PM_AE_UG_1_0;
  }
}

void display_PM2_5_label()
{
  screenChanged = false;
  previousFace = NONE_FACE;
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.drawRGBBitmap(40,40, PM2_5Buffer, 108, 32);
  previousPM = SCREEN_CHANGED_NEED_TO_DISPLAY_PMS; 
}

void display_current_PM2_5()
{
  if(previousPM != data.PM_AE_UG_2_5)
  {
    displayPM2_5Circle();
    tft.fillRect(40, 116, 104, 36, ILI9341_BLACK);
    tft.setCursor(40, 150);
    tft.println(data.PM_AE_UG_2_5);
    previousPM = data.PM_AE_UG_2_5;
  }
}

void display_PM10_label()
{
  screenChanged = false;
  previousFace = NONE_FACE;
  tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
  tft.drawRGBBitmap(40,40, PM10Buffer, 108, 32);
  previousPM = SCREEN_CHANGED_NEED_TO_DISPLAY_PMS; 
}

void display_current_PM10()
{
  if(previousPM != data.PM_AE_UG_10_0)
  {
    displayPM10Circle();
    tft.fillRect(40, 116, 104, 36, ILI9341_BLACK);
    tft.setCursor(40, 150);
    tft.println(data.PM_AE_UG_10_0);
    previousPM = data.PM_AE_UG_10_0;
  }
}

void display_AQI_Values()
{
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextSize(1);
  tft.fillRect(24,24, 272, 50, ILI9341_BLACK);
  tft.fillRect(24,150, 272, 60, ILI9341_BLACK);
  tft.drawRGBBitmap(24,40, photoBuffer , 272, 152);
  screenChanged =false;
  previousPM = SCREEN_CHANGED_NEED_TO_DISPLAY_PMS; 
}

void gestureInterupt()
{
  isr_flag=1; 
}

void readPMSData()
{
  pms.readUntil((data), 100);
}

void getGestureAndApplyChange()
{

  uint8_t gesture;
  Serial.println("Before Gesture available");
  if ( apds.isGestureAvailable() )
  {
    Serial.println("After Gesture available");
    gesture = apds.readGesture();
    Serial.println("After Gesture read");

    if(gesture == DIR_RIGHT) 
    {
      if(currentScreen != 0) 
      {
        currentScreen--;
        screenChanged = true;
      }
    }
    else if(gesture == DIR_LEFT)
    {
      if(currentScreen != 5) 
      {
        screenChanged = true;
        currentScreen++;
      }
    }
    else if(gesture == DIR_UP)
    {
      if(currentScreen == 0)
      {
        if(RTC_Configuration_Status == RTC_NOT_CONFIGURED || RTC_Configuration_Status == RTC_BEGIN) RTC_Configuration_Status++;
        if(RTC_Configuration_Status == RTC_CONFIGURED) 
        {
          if(verticalScreen != 1)
          {
            screenChanged = true;
            verticalScreen++;
          }
        } 
      }
    }
    else if(gesture == DIR_DOWN)
    {
      if(currentScreen == 0 && RTC_Configuration_Status == RTC_CONFIGURED)
      {
        if(verticalScreen != 0) 
        {
          verticalScreen--;
          screenChanged = true;	
        }
      }
    }

}
}

void measure()
{
  aht.getEvent(&humidity, &temp);
  pressure = bmp.readPressure() / 100.0F;
}

void cofigure_RTC()
{
  if(RTC_Configuration_Status == RTC_BLUETOOTH_INITED) 
  {
  for(int i = 0; i < 30; i++)
  {
    if(pServer->getConnectedCount()) break;
    delay(1000);
    RTC_Configuration_Status = RTC_NOT_CONFIGURED;
  }
  
  if(pServer->getConnectedCount())
  {
    tft.setFont(NULL);
    tft.setTextSize(2);
    tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
    tft.setCursor(30, 40);
    tft.print("BLE Connected.");
    tft.setCursor(30, 60);
    tft.print("Send id and password.");
    Serial.println("Before deinit");
    RTC_Configuration_Status = RTC_BLUETOOTH_CONNECTED;
  }

  if(RTC_Configuration_Status == RTC_NOT_CONFIGURED) 
  {
    tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
    tft.setCursor(30, 40);
    tft.print("RTC init failed");
    tft.setCursor(30, 60);
    tft.print("No device connected");
    deinit_BLE();
    delay(2000);
  }

  if(RTC_Configuration_Status == RTC_BLUETOOTH_CONNECTED)
  {
    ssid = "";
    password = "";

    uint8_t _delay = 60;
    while(true) 
    {
      if(_delay == 1) break;
      RTC_Configuration_Status = RTC_NOT_CONFIGURED;	
      delay(1000);
      if(ssid.length() != 0 && password.length() != 0) 
      {
        RTC_Configuration_Status = RTC_BLUETOOTH_PASSWORD_SENT_INIT_WIFI;
        break;
      }
      _delay--;
    }
    Serial.println("");

    delay(1000);
    deinit_BLE();
    delay(3000);

    if(RTC_Configuration_Status == RTC_NOT_CONFIGURED)
    {
      tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
      tft.setCursor(30, 40);
      tft.print("RTC init failed");
      tft.setCursor(30, 60);
      tft.print("No data sent via BLE");
      delay(3000);
    }
  }

  if(RTC_Configuration_Status == RTC_BLUETOOTH_PASSWORD_SENT_INIT_WIFI)
  {
    Serial.printf("Connecting with Wi-Fi: %s\n", ssid.c_str());
    tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
    tft.setCursor(30, 40);
    tft.printf("Connecting with Wi-Fi:");
    tft.setCursor(30, 60);
    tft.printf(ssid.c_str());
    delay(2000);
    WiFi.begin(ssid.c_str(), password.c_str());
    
    uint8_t _delay = 10;
    while (true) 
    {
      _delay--;
      if(WiFi.status() == WL_CONNECTED)
      {
        RTC_Configuration_Status = RTC_WIFI_CONNECTED;
        break;
      }
      if(_delay == 1) 
      {
        RTC_Configuration_Status = RTC_NOT_CONFIGURED;
        WiFi.disconnect(true, false); delay(1);
        delay(3000);
        break;
      }
      delay(1000);
      Serial.print(".");
    }

    

    if(RTC_Configuration_Status == RTC_NOT_CONFIGURED)
    {
      tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
      tft.setCursor(30, 40);
      tft.print("RTC init failed");
      tft.setCursor(30, 60);
      tft.print("Cannot connect to WiFi");
      delay(3000);	
    }

    if(RTC_Configuration_Status == RTC_WIFI_CONNECTED)
    {
      RTC_Configuration_Status = RTC_CONFIGURED;
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      delay(100);
      struct tm timeinfo;
      if (!getLocalTime(&timeinfo)) 
      {
        RTC_Configuration_Status = RTC_NOT_CONFIGURED;
      }
      WiFi.disconnect(true, false);
      delay(4000);
    }
  }

	//Attach Interupt
	if(apds.isGestureAvailable()) apds.readGesture();
	attachInterrupt(D3, gestureInterupt, FALLING);
	isr_flag = 0;
	
	screenChanged = true;	
	}
}

void displayDependsCurrentScreen()
{

  if(currentScreen == 0)
  {
    if(screenChanged == true && RTC_Configuration_Status == RTC_NOT_CONFIGURED) display_RTC_Configuration();
    else if(screenChanged == true && RTC_Configuration_Status == RTC_CONFIGURED) display_history();
    else if(RTC_Configuration_Status == RTC_BLUETOOTH_INIT) 
    {
      detachInterrupt(D3);
      enable_BLE_display();
      Serial.println("After BLE init");
      cofigure_RTC();
    }
  }
  if(currentScreen == 1)
  {
    if(screenChanged == true)
    {
      if(RTC_Configuration_Status!= RTC_CONFIGURED) RTC_Configuration_Status = RTC_NOT_CONFIGURED;
      display_AQI_Values();
    }
  }
  else if(currentScreen == 2)
  {
    if(screenChanged == true)
    {
      screenChanged = false;
      tft.setFont(NULL);
      tft.setTextSize(2);

      tft.fillRect(24, 24, 272, 192, ILI9341_BLACK);
      tft.setCursor(30, 40);
      tft.print("Temperature");
      tft.setCursor(30, 70);
      tft.print("Humidity");
      tft.setCursor(30, 100);
      tft.print("Pressure");
    }

    tft.fillRect(160, 24, 130, 130, ILI9341_BLACK);
    tft.setCursor(180, 40);
    tft.print(temp.temperature);
    tft.drawCircle(248, 38, 2, ILI9341_WHITE);
    tft.print(" C");
    tft.setCursor(180, 70);
    tft.print(humidity.relative_humidity);
    tft.print("%");
    tft.setCursor(180, 100);
    tft.print(pressure);
    tft.print("hPa");
    }
    //display PM1
    else if(currentScreen == 3)
    {
      if(screenChanged == true) display_PM1_0_label();
      tft.setFont(&FreeSansBold24pt7b);
      tft.setTextSize(1);
      display_current_PM1_0();
    }
    //display PM2.5
    else if(currentScreen == 4)
    {
      if(screenChanged == true) display_PM2_5_label();
      display_current_PM2_5();
    }
    //display PM10
    else if(currentScreen == 5)
    {
      if(screenChanged == true) display_PM10_label();
      display_current_PM10();
    }
}

void setup() {

	memset(PM1_Hour_Buffer, 0xFFFF, sizeof(PM1_Hour_Buffer));
	memset(PM10_Hour_Buffer, 0xFFFF, sizeof(PM10_Hour_Buffer));
	memset(PM2_5_Hour_Buffer, 0xFFFF, sizeof(PM2_5_Hour_Buffer));

  Serial.begin(9600);
  pmsSerialHard.begin(9600, SERIAL_8N1, D6, D7);

  //Initialize LCD
  tft.begin();
  tft.setTextSize(1);
  tft.setRotation(3);
  tft.setFont(&FreeSansBold24pt7b);
  tft.setTextColor(ILI9341_WHITE); 
  tft.setCursor(10, 10);
  tft.fillScreen(ILI9341_BLACK);

  //Initialize GUI registers
  verticalScreen = 0;
  currentScreen = 1;
  screenChanged = true;
  previousFace = NONE_FACE;

  //Initialize BLE, WIFI RTC
  BLE_Enable = 0;
  RTC_Configuration_Status = RTC_NOT_CONFIGURED;

  //Copy Images from 
  memcpy(photoBuffer, Photo, sizeof(Photo));
  memcpy(PM1Buffer, PM1, sizeof(PM1));
  memcpy(PM2_5Buffer, PM2_5, sizeof(PM2_5));
  memcpy(PM10Buffer, PM10, sizeof(PM10));

  //Printing frame
  tft.drawRGBBitmap(0,0,horizontalStripe, 320, 24);
  tft.drawRGBBitmap(0,216,horizontalStripe, 320, 24);
  tft.drawRGBBitmap(0,0,verticalStripeLeft, 24, 240);
  tft.drawRGBBitmap(296,0,verticalStripeRight, 24, 240);

	if (!aht.begin()) 
  {
    Serial.println("AHT20 failed");
    while (1) delay(10);
  }

  Serial.println("AHT20 initialized!");

  //Iniit BMP280 
  if (!bmp.begin(0x77)) 
  {
    Serial.println("BMP280 failed");
    while (1) delay(10);
  }

  Serial.println("BMP280 initialized!");

  //PMS should run for a while
  delay(3000);

  //Initalizing Gesture Module
  if ( apds.init() ) 
  {
    Serial.println(F("APDS-9960 initialization complete"));
  }
  else 
  {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }
  
  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) 
  {
    Serial.println(F("Gesture sensor is now running"));
  } else 
  {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }

  //Use gesture module with Interupt
	attachInterrupt(D3, gestureInterupt, FALLING);
}

void getTimeComponents(int &hour, int &minute, int &second) {

	struct tm timeinfo;
	if (!getLocalTime(&timeinfo)) 
  {
		Serial.println("Cannot load RTC Time");
	}

	hour = timeinfo.tm_hour;   // hour
	minute = timeinfo.tm_min;  // minutes
	second = timeinfo.tm_sec;  // seconds
}

void makeInstructionsRelatedToRTC()
{
	int hour, minute, second;
	int sumPM1 = 0, sumPM2_5 = 0, sumPM10 = 0;
	int numberOfElementsPM1 = 0,numberOfElementsPM2_5 = 0,numberOfElementsPM10 = 0;
	getTimeComponents(hour, minute, second);
	if(hour == 25) last_hour = hour;
	if(hour != last_hour)
	{
		for(int i = 0; i < 3600;i++)
		{
			if(PM1_Hour_Buffer[i] != 0xFFFF) 
      {
				sumPM1 += PM1_Hour_Buffer[i];
				numberOfElementsPM1++;
			}
			if(PM10_Hour_Buffer[i] != 0xFFFF)
      {
				sumPM10 += PM10_Hour_Buffer[i];
				numberOfElementsPM10++;
			}
			if(PM2_5_Hour_Buffer[i] != 0xFFFF)
      {
				sumPM2_5 += PM2_5_Hour_Buffer[i];
				numberOfElementsPM2_5++;
			}
		}

    if(numberOfElementsPM10 > 0 && numberOfElementsPM2_5 > 0 && numberOfElementsPM1 > 0)
    {
      PM1_Average[last_hour] = sumPM1 / numberOfElementsPM1;
      PM2_5_Average[last_hour] =	sumPM2_5 / numberOfElementsPM2_5;
      PM10_Average[last_hour] = sumPM10 / numberOfElementsPM10;
    }
	
    memset(PM1_Hour_Buffer, 0xFFFF, sizeof(PM1_Hour_Buffer));
    memset(PM10_Hour_Buffer, 0xFFFF, sizeof(PM10_Hour_Buffer));
    memset(PM2_5_Hour_Buffer, 0xFFFF, sizeof(PM2_5_Hour_Buffer));

    last_hour = hour;
    screenChanged = true;
  }

	PM10_Hour_Buffer[second + minute * 60] = data.PM_AE_UG_10_0;
	PM2_5_Hour_Buffer[second + minute * 60] = data.PM_AE_UG_2_5;
	PM1_Hour_Buffer[second + minute * 60] = data.PM_AE_UG_1_0;
}


void loop() {
  unsigned long currentMillis = millis();

  //1 loop = 1s
  if (currentMillis - previousMillis >= interval) {
	
  startMillis = 0;
	startMillis = millis();
 
  previousMillis = currentMillis;

	if(RTC_Configuration_Status==RTC_CONFIGURED)
	{
		makeInstructionsRelatedToRTC();
	}

  //Make measures
  readPMSData();
  measure();

  displayDependsCurrentScreen();

  if (isr_flag == 1) 
  {
    detachInterrupt(D3);  
    getGestureAndApplyChange();  
    isr_flag = 0; 
    attachInterrupt(D3, gestureInterupt, FALLING);
	  if(apds.isGestureAvailable()) apds.readGesture();
  }

	displayDependsCurrentScreen();

	endMillis = 0;
	endMillis = millis();

	unsigned long result = endMillis - startMillis;
	printf("Time of current loop: %lu\n", result);
  }
}