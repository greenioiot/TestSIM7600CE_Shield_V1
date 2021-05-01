#include <Adafruit_MLX90614.h>

#include <TFT_eSPI.h>
#include <SPI.h>

#include "BluetoothSerial.h""

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


#include <TFT_eSPI.h>
#include "FS.h"

#include "RTClib.h"
#include "Free_Fonts.h"

#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32



// #define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_MODEM_SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
//#define SerialAT xz
HardwareSerial xz(1);



Adafruit_MLX90614 mlx = Adafruit_MLX90614();

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library
#define TFT_GREY 0x5AEB // New colour
TFT_eSprite img = TFT_eSprite(&tft);  // Declare Sprite object "spr" with pointer to "tft" object



// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

/*
Tests enabled
*/
#define TINY_GSM_TEST_GPRS true
#define TINY_GSM_TEST_WIFI false
#define TINY_GSM_TEST_CALL false
#define TINY_GSM_TEST_SMS false
#define TINY_GSM_TEST_USSD false
#define TINY_GSM_TEST_BATTERY false
#define TINY_GSM_TEST_GPS true
// powerdown modem after tests
#define TINY_GSM_POWERDOWN true

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]  = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

#include <TinyGsmClient.h>
#include <Ticker.h>

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(xz, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(xz);
#endif

Ticker tick;


#define uS_TO_S_FACTOR          1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP           60          /* Time ESP32 will go to sleep (in seconds) */

#define PIN_TX                  14
#define PIN_RX                  27
#define UART_BAUD               115200
#define PWR_PIN                 4
#define LED_PIN                 12
#define POWER_PIN               23  // 
#define IND_PIN                 36


void _initLCD() {


  tft.fillScreen(TFT_BLACK);
  // TFT

  splash();

  // MLX
  mlx.begin();

}

void splash() {
  int xpos =  0;
  int ypos = 40;
  tft.init();
  // Swap the colour byte order when rendering
  tft.setSwapBytes(true);
  tft.setRotation(1);  // landscape

  tft.fillScreen(TFT_BLACK);
  // Draw the icons
  //  tft.pushImage(tft.width() / 2 - logoWidth / 2, 39, logoWidth, logoHeight, logo);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(TC_DATUM); // Centre text on x,y position

  tft.setFreeFont(FSB9);
  xpos = tft.width()/ 2; // Half the screen width
  ypos = 150;
  tft.drawString("Greenio", xpos, ypos, GFXFF);  // Draw the text string in the selected GFX free font

  delay(3000);

  tft.setTextFont(GLCD);
  tft.setRotation(1);
  tft.fillScreen(TFT_WHITE);
  // Select the font
  ypos += tft.fontHeight(GFXFF);                      // Get the font height and move ypos down
  tft.setFreeFont(FSB9);
  //  tft.pushImage(tft.width()/2 - (Splash2Width/2)-15, 3, Splash2Width, Splash2Height, Splash2);



  delay(1200);
  tft.setTextPadding(180);
  tft.setTextColor(TFT_GREEN);
  tft.setTextDatum(MC_DATUM);
  Serial.println("Start...");

  Serial.println("end");
}


void setup()
{
  // Set console baud rate
  SerialMon.begin(115200);
  SerialBT.begin("Emer32");
  SerialBT.println("Start...");
  _initLCD();
  delay(10);

  // Onboard LED light, it can be used freely
  //    pinMode(LED_PIN, OUTPUT);
  //    digitalWrite(LED_PIN, LOW);

  // POWER_PIN : This pin controls the power supply of the SIM7600
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);

  // PWR_PIN ： This Pin is the PWR-KEY of the SIM7600
  // The time of active low level impulse of PWRKEY pin to power on module , type 500 ms
  //    pinMode(PWR_PIN, OUTPUT);
  //    digitalWrite(PWR_PIN, HIGH);
  //    delay(500);
  //    digitalWrite(PWR_PIN, LOW);

  // IND_PIN: It is connected to the SIM7600 status Pin,
  // through which you can know whether the module starts normally.
  pinMode(IND_PIN, INPUT);

  attachInterrupt(IND_PIN, []() {
    detachInterrupt(IND_PIN);
    // If SIM7600 starts normally, then set the onboard LED to flash once every 1 second
    tick.attach_ms(1000, []() {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    });
  }, CHANGE);

  DBG("Wait...");

  delay(3000);

  //    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);

  xz.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem...");
  if (!modem.init()) {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return;
  }

  //Set to GSM mode, please refer to manual 5.11 AT+CNMP Preferred mode selection for more parameters
  String result;
  do {
    result = modem.setNetworkMode(13);
    delay(500);
  } while (result != "OK");


#if 0
  //https://github.com/vshymanskyy/TinyGSM/pull/405
  uint8_t mode = modem.getGNSSMode();
  Serial.print("Get GNSS Mode: ");
  Serial.println(mode);

  /**
  CGNSSMODE: <gnss_mode>,<dpo_mode>
  This command is used to configure GPS, GLONASS, BEIDOU and QZSS support mode.
  gnss_mode:
  0 : GLONASS
  1 : BEIDOU
  2 : GALILEO
  3 : QZSS
  dpo_mode :
  0 disable
  1 enable
  */
  Serial.print("Set GNSS Mode BEIDOU : ");
  String res = modem.setGNSSMode(1, 1);
  Serial.println(res);
  delay(1000);
#endif

}

void loop()
{
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  DBG("Initializing modem...");
  if (!modem.restart()) {
    DBG("Failed to restart modem, delaying 10s and retrying");
    return;
  }

  String name = modem.getModemName();
  DBG("Modem Name: ", name);

  String modemInfo = modem.getModemInfo();
  DBG("Modem Info: ", modemInfo);

#if TINY_GSM_TEST_GPRS
  // Unlock your SIM card with a PIN if needed
  if ( GSM_PIN && modem.getSimStatus() != 3 ) {
    modem.simUnlock(GSM_PIN);
  }
#endif

#if TINY_GSM_TEST_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

  DBG("Waiting for network...");
  if (!modem.waitForNetwork()) {
    delay(10000);
    return;
  }

  if (modem.isNetworkConnected()) {
    DBG("Network connected");
  }


#if TINY_GSM_TEST_GPRS
  DBG("Connecting to", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    delay(10000);
    return;
  }

  bool res = modem.isGprsConnected();
  DBG("GPRS status : ", res ? "connected" : "not connected");

  String ccid = modem.getSimCCID();
  DBG("CCID : ", ccid);
  SerialBT.print("ccid : ");SerialBT.println(ccid);
  String imei = modem.getIMEI();
  DBG("IMEI : ", imei);

  String cop = modem.getOperator();
  DBG("Operator : ", cop);

  IPAddress local = modem.localIP();
  DBG("Local IP : ", local);

  int csq = modem.getSignalQuality();
  DBG("Signal quality : ", csq);

#endif

// Test the GSM location functions
#if defined(TINY_GSM_MODEM_HAS_GSM_LOCATION)
  modem.getGsmLocationRaw();
  modem.getGsmLocation();
  float glatitude  = -9999;
  float glongitude = -9999;
  float gacc       = 0;
  int   gyear      = 0;
  int   gmonth     = 0;
  int   gday       = 0;
  int   ghour      = 0;
  int   gmin       = 0;
  int   gsec       = 0;
  modem.getGsmLocation(&glatitude, &glongitude);
  modem.getGsmLocation(&glatitude, &glongitude, &gacc, &gyear, &gmonth, &gday,
                       &ghour, &gmin, &gsec);
  modem.getGsmLocationTime(&gyear, &gmonth, &gday, &ghour, &gmin, &gsec);
#endif

#if TINY_GSM_TEST_GPRS
  modem.gprsDisconnect();
  if (!modem.isGprsConnected()) {
    DBG("GPRS disconnected");
  } else {
    DBG("GPRS disconnect : Failed.");
  }
#endif

#if TINY_GSM_TEST_GPS
  modem.enableGPS();
  float lat,  lon;
  int xpos =  0;
  int ypos = 40;
  while (10) {
    if (modem.getGPS(&lat, &lon)) {
      Serial.printf("lat : % f lon : % f\n", lat, lon);
      

      tick.attach_ms(200, []() {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      });
      break;
    } else {
      Serial.print("getGPS ");
      Serial.println(millis());

    }
    SerialBT.print("Loc:");SerialBT.print(lat); SerialBT.print(",");SerialBT.println(lon);
    delay(2000);
  }
  modem.disableGPS();
#endif


  // Test Battery functions
#if defined(TINY_GSM_MODEM_HAS_BATTERY)
  uint8_t  chargeState   = 0;
  int8_t   chargePercent = 0;
  uint16_t milliVolts    = 0;
  modem.getBattStats(chargeState, chargePercent, milliVolts);
#endif

  // Test the temperature function
#if defined(TINY_GSM_MODEM_HAS_TEMPERATURE)
  modem.getTemperature();
#endif


#if TINY_GSM_POWERDOWN
  // Try to power-off (modem may decide to restart automatically)
  // To turn off modem completely, please use Reset/Enable pins
  modem.poweroff();
  DBG("Poweroff.");
#endif

  // Test is complete Set it to sleep mode
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  delay(200);
  esp_deep_sleep_start();
}
