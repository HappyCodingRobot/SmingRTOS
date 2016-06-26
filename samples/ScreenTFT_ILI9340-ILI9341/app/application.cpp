//#include <user_config.h>
#include <SmingCore.h>
#include <libraries/Adafruit_ILI9341/Adafruit_ILI9341.h>
#include <c_types.h>
//#include <libraries/Adafruit_GFX/Adafruit_GFX.h>


// If you want, you can define WiFi settings globally in Eclipse Environment Variables
#ifndef WIFI_SSID
    #define WIFI_SSID "PleaseEnterSSID" // Put you SSID and Password here
    #define WIFI_PWD "PleaseEnterPass"
#endif

/*
 * Hardware SPI mode:
 * GND      (GND)       GND
 * VCC      (VCC)       3.3v
 * SCK      (CLK)       GPIO14
 * SDI      (MOSI)      GPIO13
 * RES      (RESET)     GPIO16
 * DC       (DC)        GPIO0
 * CS       (CS)        GPIO2
 * LED      (BL)        Resistor to 3.3V
 * SDO      (MISO)      <nc>  
 */
//#define TFT_SCLK    14
//#define TFT_MOSI    13
//#define TFT_RST     16
//#define TFT_DC      0
//#define TFT_CS      2
#define TFT_SCLK    14
#define TFT_MOSI    13
#define TFT_RST     4
#define TFT_DC      5
#define TFT_CS      15



Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

Timer guiTimer;

int r=0;

int ara=4,yerara=15;
int u1=100;
int u2=320-(u1+ara);
int s1=0;
int s2=(u1+ara);
int p1=50;

int g=28;
int y=90;
int satir=6;

uint32_t startTime;

String lists[]={"a","b","c","d","e","f"};

// -> crash/reset (wdt?) bei timer init dieser Funktion ?
void basicGui() {
    debugf("running basicGui()\n");
    startTime = millis();
    tft.setTextSize(1);

    tft.setRotation(1);

    tft.setTextSize(3);
    tft.fillRect(s1, 0, u1 * 2, 48, ILI9341_OLIVE);
    tft.setCursor(15, 15);
    tft.println("Sming");
    tft.setTextSize(2);
    tft.fillRect((u1 * 2) + ara, 0, 318 - (u1 * 2), 48, ILI9341_RED);
    for (int a = 0; a < satir; a++) {
        tft.setTextColor(ILI9341_GREEN);
        tft.fillRect(s1, p1, u1, g, ILI9341_DARKCYAN);
        tft.setCursor(s1 + yerara, p1 + 6);
        tft.setTextColor(ILI9341_WHITE);
        tft.println(lists[a]);
        tft.fillRect(s2, p1, u2, g, ILI9341_DARKCYAN);
        tft.setCursor(s2 + yerara, p1 + 6);
        tft.println(r);
        p1 += g + 4;
    }
    p1 = 50;
    r++;
    debugf("-GUI displayed in %d ms\n", millis() - startTime);
}

void init() {
    Serial.begin(SERIAL_BAUD_RATE); // 115200 by default
    Serial.systemDebugOutput(true); // Allow debug output to serial
    Serial.commandProcessing(false);
    //WifiStation.config(WIFI_SSID, WIFI_PWD);
    WifiStation.enable(false);
    //WifiAccessPoint.enable(false);
    //  delay(2000);
    debugf("Display start");
    startTime = millis();
    
    // text display tests
    //tft.begin();
    //tft.init();
    tft.init(4000000);
    debugf("Init done");
    debugf("-clearscreen\n");
    tft.fillScreen(ILI9341_BLACK);
    debugf("-Initialized in %d ms\n", millis() - startTime);
    
    tft.setRotation(1);
    debugf("-setrotation(1) done");
    tft.setTextSize(2);
    debugf("-seTextSize(2) done");
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(0, 0);
    tft.setCursor(60, 60);
    tft.println("Sming  Framework");
    tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // text
    tft.setCursor(60, 75);
    tft.println("              v1.1");
    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(60, 90);
    tft.println("ili9340-40C-41 ");
    tft.setCursor(60, 125);
    tft.println("have fun with Sming");
    debugf("initial print done");
    //delay(2000);
    //debugf("delay done");
    tft.fillScreen(ILI9341_BLACK);
    debugf("fillScreen(0) done");
    guiTimer.initializeMs(1000, basicGui).start();
    //guiTimer.initializeMs(1000, basicGui).start(FALSE);
    debugf("timer task started");
    //runTest();
}
