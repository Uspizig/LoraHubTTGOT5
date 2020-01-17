//Test Eink Display with MCCi Lora stack and TTGO T5 Board
//Join after 2-3 Versuchen 
//Fixed in this: Packages are transmitted faster
//Problems: No RX Data seen so far.
//fix Battery Level Detection
//Include TPL51110 for Power Shutdown
//ABP after OTAA

//Changes done in the MCCI LMIC Stack:
//#define LMICbandplan_getInitialDrJoin() (EU868_DR_SF8) in lmic_bandplan_eu868.h
//#define LMIC_PRINTF_TO Serial //USpizig  in config.h bringt aber keinerlei zusätzliche debug Ausgaben
//in config.h Debug Messages an und RFM95 fediniert
//in oslmic.h ersetze #define OSTICKS_PER_SEC von 32768 in 50000 //Uspizig bringt aber nix
//in hal.cpp replaced spi-begin to spi.begin(14, 2, 15, 26); to match pinning
/*
static void hal_spi_init () {
    //SPI.begin();//Original
  //SPI.begin(lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, 0x00);
  SPI.begin(14, 2, 15, 26);
}
*/
//for Debugging
//#define SingleChannelMode 1 //to check on own gateway Join Behaviour
#define LMIC_DEBUG_LEVEL 1
#define geraet1 //soldered DIO0 directly to GPIO 34 and DIO01 to GPIO25  == Board identifies as COM26
//#define geraet2 //Both DIO00 and DIO01 soldered with a Diode to GPIO 25  == Board identifies as COM7


// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc  
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

#ifdef geraet1
  const unsigned TX_INTERVAL = 30; 
#else //limit uplinks on testboard
  const unsigned TX_INTERVAL = 180; 
#endif

//WetterSymbols Size
int wettercount = 1;
#define LEFT 1
#define CENTER 2
#define RIGHT 3
boolean LargeIcon = true, SmallIcon = false;
//250x122 Pixel
int wetter_symbol_x = 180; 
int wetter_symbol_y = 80; 
int status_symbol_x = 215;
int status_symbol_y = 125;
boolean wetter_symbol_size = true;
#define Large  5           // For icon drawing, needs to be odd number for best effect
#define Small  3            // For icon drawing, needs to be odd number for best effect

//Welcher Tempsensor angeschlossen  
  #define BMP280_CONNECTED
  //#define BME280_CONNECTED 0

//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4//19
  #define E_INK_PIN_SPI_RST  16//21
  #define E_INK_PIN_SPI_DC   17//22
  #define E_INK_PIN_SPI_CS   5
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO T5 Addon Board
  #define LORA_PIN_SPI_MOSI 15//19
  #define LORA_PIN_SPI_MISO 2//n/A
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO1 25
  #define LORA_PIN_SPI_DIO0 34

//Battery Pin
  #define BATTERY_PIN 35 //needs to be changed later on
  
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "LoraEncoder.h"

//Temperatur:
#include <Wire.h>
#ifdef BMP280_CONNECTED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
#endif

#ifdef BME280_CONNECTED
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

#define sda 21 ///* I2C Pin Definition */
#define scl 22 ///* I2C Pin Definition */
float temp;
float pressure;
float humidity;

/*EINK Teile*/

#include <GxEPD2_BW.h>
#include "GxEPD2_boards_added.h"
// select one and adapt to your mapping, can use full buffer size (full HEIGHT)
//GxEPD2_BW<GxEPD2_213, GxEPD2_213::HEIGHT> display(GxEPD2_213(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDE0213B1, phased out
//GxEPD2_BW<GxEPD2_213_B72, GxEPD2_213_B72::HEIGHT> display(GxEPD2_213_B72(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B72
//GxEPD2_BW<GxEPD2_213_flex, GxEPD2_213_flex::HEIGHT> display(GxEPD2_213_flex(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEW0213I5F
//GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> display(GxEPD2_290(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));
//GxEPD2_BW<GxEPD2_290_T5, GxEPD2_290_T5::HEIGHT> display(GxEPD2_290_T5(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEW029T5
//GxEPD2_BW<GxEPD2_it60, GxEPD2_it60::HEIGHT> display(GxEPD2_it60(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY)); // GDEH0213B73



//Wifi
#include "WiFi.h"

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>



uint32_t start_time;
uint32_t next_time;
uint32_t previous_time;
uint32_t previous_full_update;

uint32_t total_seconds = 0;
uint32_t startup_seconds = 0;
uint32_t seconds, minutes, hours, days;

uint32_t join_total_seconds = 0;
uint32_t join_seconds, join_minutes, join_hours, join_days;

//Partial Update:
const uint32_t partial_update_period_s = 30;
//const uint32_t full_update_period_s = 60 * 60 * 60;//alle 2.5 Tage
const uint32_t full_update_period_s = 60 * 60;//(jede Stunde)
//const uint32_t full_update_period_s = 60;(jede Minute)
/*EINK Teile ENDE*/

#ifdef geraet1
//LORA Keys 1
//Lora APP ID xxxxx
  static const u1_t PROGMEM APPEUI[8]={ XXXX };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ XXX };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { XXX };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#else
//LORA Keys 2
 
//Lora App ID xxxx
  static const u1_t PROGMEM APPEUI[8]={ XXX };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ XXX};
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { XXX };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

int verbunden_indicator = 0;
int Packet_Transmission_ongoing = 0; // Display wird nur aktualisiert wenn kein Paket gesendet wird
int packet_counter =0;
static osjob_t sendjob;
devaddr_t DeviceName = 0;

// Pin mapping

const lmic_pinmap lmic_pins = {
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    #ifdef geraet1
      .dio = {LORA_PIN_SPI_DIO0, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
    #else
      .dio = {LORA_PIN_SPI_DIO1, LORA_PIN_SPI_DIO1, LMIC_UNUSED_PIN}, 
    #endif
    /*
    //workaround to use 1 pin for all 3 radio dio pins
    // optional: set polarity of rxtx pin.
    .rxtx_rx_active = 0,
    // optional: set RSSI cal for listen-before-talk
    // this value is in dB, and is added to RSSI
    // measured prior to decision.
    // Must include noise guardband! Ignored in US,
    // EU, IN, other markets where LBT is not required.
    .rssi_cal = 0,
    // optional: override LMIC_SPI_FREQ if non-zero
    .spi_freq = 0,*/
    .rxtx_rx_active = 0,//kopiert von TTGO Board
    .rssi_cal = 10,//kopiert von TTGO Board
    //.spi_freq = 8000000, /* 8MHz */ //kopiert von TTGO Board
    .spi_freq = 4000000, //4 MHZ from GxEPD2 https://github.com/ZinggJM/GxEPD2/tree/master/extras/sw_spi
};



void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
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
            verbunden_indicator = 1;
            break;
        case EV_JOINED:
            verbunden_indicator = 2;
            total_seconds =0;
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              DeviceName = devaddr;
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            verbunden_indicator = 4;
            packet_counter++;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            Packet_Transmission_ongoing = 0;
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            verbunden_indicator = 3;
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: No JoinAccept"));
            break;
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        byte buffer[2];
        LoraEncoder encoder(buffer);
        encoder.writeTemperature(temp);
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
        //Serial.print("Temp:");Serial.print(temp); 
        Serial.println(F("Packet queued"));
        Packet_Transmission_ongoing = 1;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

//Inits EINk Display 
void setup_eink(void){
  display.epd2.init(E_INK_PIN_SPI_SCK, E_INK_PIN_SPI_DIN, 115200, true, false); 
  delay(100);
  display.init(115200);
  Serial.println("Display setup done");
  Display_GrundAnzeige();
}

//Init Lora Stack, sets ADR Mode, 
void setup_lora(void){
  pinMode(LORA_PIN_SPI_DIO1, INPUT_PULLDOWN);//to enable PullDown but update your ESP32 Lib to avoid https://esp32.com/viewtopic.php?t=439
  pinMode(LORA_PIN_SPI_DIO0, INPUT_PULLDOWN);
  // LMIC init
    os_init();
    
    LMIC_reset(); // Reset the MAC state. Session and pending data transfers will be discarded.
    
    //Debugging Purpose for Single Channel
    #ifdef SingleChannelMode
      #define CHANNEL  1
      for (uint8_t i = 0; i < 9; i++) {
        if (i != CHANNEL) {
          LMIC_disableChannel(i);
        }
      }
    #endif
    
    //Adaptive Data Rate Mode https://www.thethingsnetwork.org/docs/lorawan/adaptive-data-rate.html 
    //LMIC_setAdrMode(1); //Adaptiert Datenrate nach 64 Paketen
    LMIC_setLinkCheckMode(0);
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

    //Eingebaut in /src/lmic.c geht nicht
    //LMIC_dn2dr = EU868_DR_SF9;//https://github.com/mcci-catena/arduino-lmic/issues/455
    //LMIC_selectSubBand(0); //https://github.com/mcci-catena/arduino-lorawan/issues/74
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println(F("Starting"));

    Serial.println("** Stopping WiFi+BT");
    WiFi.mode(WIFI_OFF);
    btStop();
    
    #ifdef BMP280_CONNECTED
      bool status2 = BMP_Start();//alles ok?
      BMP_Test();
      delay(2000);
    #endif
    
    #ifdef BME280_CONNECTED
      bool status2 = BME_Start();//alles ok?
      BME_Test();
      delay(2000);
    #endif
   
    setup_eink();   
    setup_lora();    
}

void loop() {
    os_runloop_once();
    if (verbunden_indicator == 4){
      verbunden_indicator = 2;
       #ifdef BMP280_CONNECTED
            Serial.print(" next_time:");Serial.println(next_time);
            BMP_Test();
          #endif
          #ifdef BME280_CONNECTED
            BME_Test();
       #endif
       showPartialUpdate(); 
       showPartialUpdateWetter(wetter_symbol_x, wetter_symbol_y, wetter_symbol_size);
    }
}




//Parts below this line can be used independent from LORA and are only for displaying Stuff on the EINK
void showPartialUpdateWetter(int x, int y, boolean IconSize){
 int box_w = 50;
 int box_h = 50;
 //boolean IconSize = false;
 display.fillRect(x, y, box_w, box_h, GxEPD_WHITE);
 if (wettercount <12){  
  if (wettercount == 1) Sunny(x, y, IconSize, "01d");
  else if (wettercount == 2)  MostlySunny(x, y, IconSize, "02d");
  else if (wettercount == 3)  Cloudy(x, y, IconSize, "03d");
  else if (wettercount == 4)  MostlySunny(x, y, IconSize, "04d");
  else if (wettercount == 5)  ChanceRain(x, y, IconSize, "09d");
  else if (wettercount == 6)  Rain(x, y, IconSize, "10d");
  else if (wettercount == 7)  Tstorms(x, y, IconSize, "11d");
  else if (wettercount == 8)  Snow(x, y, IconSize, "13d");
  else if (wettercount == 9)  Haze(x, y, IconSize, "50d");
  else if (wettercount == 10) Fog(x, y, IconSize, "50n");
  else                        Nodata(x, y, IconSize, "55d");
  wettercount++;
  Serial.print("Wetter-Symbol:");Serial.println(wettercount);
 }
 else{
  wettercount = 1;
 }
 display.setFullWindow();
 display.display(true);
}


void Status_Info(int x, int y, boolean IconSize){
  display.setFont(&FreeMonoBold9pt7b);
  DrawBattery(x, y);
}


void Display_GrundAnzeige(void){
  int x = 10;
  int y = 20;
  start_time = next_time = previous_time = previous_full_update = millis();
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(x, y);
  // full window mode is the initial mode, set it anyway
  display.setFullWindow();
  display.fillScreen(GxEPD_WHITE);
  display.setCursor(x, y);
  display.print("LoRa Joining");
  Status_Info(status_symbol_x, status_symbol_y, false); //Zeigt Battery Status an
  display.display(false); // full update
  Serial.println("Grundanzeige done");
}


void showPartialUpdate()
{
  
  
  total_seconds += (int)TX_INTERVAL;
  seconds = total_seconds % 60;
  minutes = (total_seconds / 60) % 60;
  hours = (total_seconds / 3600) % 24;
  days = (total_seconds / 3600) / 24;
  uint16_t box_x = 10;
  uint16_t box_y = 5;//uint16_t box_y = 15; //y=25 war ohne DEVADDDR
  uint16_t box_w = 170;
  uint16_t box_h = 100;//uint16_t box_h = 20; //60 ohne BMP //war 80
  uint16_t cursor_y = box_y + 16;
  display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(box_x, cursor_y);
  display.print("DEVADD:"); display.print(DeviceName); Serial.print("DeviceName: ");Serial.println(DeviceName);
  display.setCursor(box_x, cursor_y+20);
  display.print(days); display.print("d "); print02d(hours); display.print(":"); print02d(minutes); display.print(":"); print02d(seconds);
  display.setCursor(box_x, cursor_y+40);
  
  if (verbunden_indicator == 1){display.println("Joining");}
  //else if (verbunden_indicator == 2){display.println("Joined");}
  else if (verbunden_indicator == 2){display.print("TX Count:"); display.print(packet_counter);}
  else {display.println("NO PLAN");}
  
  #ifdef BMP280_CONNECTED
    display.setCursor(box_x, cursor_y+60);
    display.print("Temp:"); display.print(temp);
  #endif  
  
  #ifdef BME280_CONNECTED
    display.setCursor(box_x, cursor_y+60);
    display.print("Temp:"); display.print(temp);
  #endif
  display.display(true);
  //display.updateWindow(box_x, box_y, box_w, box_h, true);
  
}

//Adds one Digit if Number is smaller than 10
void print02d(uint32_t d)
{
  if (d < 10) display.print("0");
  display.print(d);
}

//Temperaturmessungen

#ifdef BME280_CONNECTED
    int BME_Start(void){
      bool status;
        Wire.begin(sda, scl);
        status = bme.begin();  
        Serial.print("BME check: "); Serial.print(status);
        if (!status) {
            Serial.println("No BME280 I2C sensor, check wiring!");
            return 0;
        }
        else{
          Serial.println("BME gefunden");
          return 1;
        }
    }
    
    void BME_Test(void){
         
          Serial.print("BME280 I2C: ");
          Serial.print("Temp = ");
          Serial.print(bme.readTemperature());
          Serial.print(" *C ");
      
          Serial.print("Druck = ");
          Serial.print(bme.readPressure() / 100.0F);
          Serial.print(" hPa ");
    
          Serial.print("Feuchte = ");
          Serial.print(bme.readHumidity());
          Serial.print(" % \n");
          
          temp = bme.readTemperature();
          pressure = bme.readPressure()/ 100.0F;
          humidity = bme.readHumidity();
        
    }
#endif

#ifdef BMP280_CONNECTED
    int BMP_Start(void){
      bool status;
        Wire.begin(sda, scl);
        status = bmp.begin();  
        Serial.print("BME check: "); Serial.print(status);
        if (!status) {
            Serial.println("No BMP280 I2C sensor, check wiring!");
            return 0;
        }
        else{
          Serial.println(" BMP280 gefunden");
          /* Default settings from datasheet. */
          bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                      Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                      Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                      Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                      Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
          return 1;
        }
    }
    
    void BMP_Test(void){
         
          /* //toavoid unneccessary time for debug printouts
          Serial.print("BMP280 I2C: ");
          Serial.print("Temp = ");
          Serial.print(bmp.readTemperature());
          Serial.print(" *C ");
      
          Serial.print("Druck = ");
          Serial.print(bmp.readPressure() / 100.0F);
          Serial.print(" hPa ");
    
          Serial.print(F("Approx altitude = "));
          Serial.print(bmp.readAltitude(1013.25)); // Adjusted to local forecast! 
          Serial.println(" m");
          */
    
          
          temp = bmp.readTemperature();
          pressure = bmp.readPressure()/ 100.0F;
        
    }
#endif 

// Wetter Symbole
//#########################################################################################


void DrawBattery(int x, int y) {
  uint8_t percentage = 100;
  float voltage = analogRead(BATTERY_PIN) / 4096.0 * 7.46;//ist noch falscher PIN
  if (voltage > 1 ) { // Only display if there is a valid reading
    //Serial.println("Voltage = " + String(voltage));
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.50) percentage = 0;
    display.drawRect(x + 15, y - 12, 19, 10, GxEPD_BLACK);
    display.fillRect(x + 34, y - 10, 2, 5, GxEPD_BLACK);
    display.fillRect(x + 17, y - 10, 15 * percentage / 100.0, 6, GxEPD_BLACK);
    drawString(x + 10, y - 11, String(percentage) + "%", RIGHT);
    //drawString(x + 13, y + 5,  String(voltage, 2) + "v", CENTER);
  }
}
/*...... you may want to add some code from G6EJD
e.g.
https://github.com/G6EJD/ESP32-e-Paper-Weather-Display/
*/


void drawString(int x, int y, String text, int align) {
  int16_t  x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  display.setTextWrap(false);
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)  x = x - w;
  if (align == CENTER) x = x - w / 2;
  display.setCursor(x, y + h);
  display.print(text);
}