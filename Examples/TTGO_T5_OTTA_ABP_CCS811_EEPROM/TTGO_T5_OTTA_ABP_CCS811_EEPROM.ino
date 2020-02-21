//Test Eink und MCCi Lora mit TTGO T5 Board
//Join nach 2-3 Versuchen 
// Works: Speichert Join Daten in RTC Memory und EEPROM
// Works: Lese Join Daten aus RTC Memory.
// Displays Data on a 2.13 Eink Display TTGOT5
// Geraet3 ist ohne Display! mit BME280, mit CCS811
// Geraet2 mit BME280
// Geraet1 mit BMP280


//Notwendige Änderungen in der Stack:
//#define LMICbandplan_getInitialDrJoin() (EU868_DR_SF8) in lmic_bandplan_eu868.h
//#define LMIC_PRINTF_TO Serial //USpizig  in config.h bringt aber keinerlei zusätzliche debug Ausgaben
//in config.h Debug Messages an und RFM95 fediniert
//in oslmic.h ersetze #define OSTICKS_PER_SEC von 32768 in 50000 //Uspizig bringt aber nix
//in hal.cpp replaced spi-begin to spi.begin(14, 2, 15, 26); to match pinning
/*
static void hal_spi_init () {
    //SPI.begin();//Original
  //SPI.begin(lmic_pins.sck, lmic_pins.miso, lmic_pins.mosi, lmic_pins.nss);
  SPI.begin(14, 2, 15, 26);
}
*/


//for Debugging
//#define SingleChannelMode 1 //to check on own gateway Join Behaviour
#define LMIC_DEBUG_LEVEL 1
#define geraet1 //soldered DIO0 directly to GPIO 34 and DIO01 to GPIO25  == Board identifies as COM26
//#define geraet2 //Both DIO00 and DIO01 soldered with a Diode to GPIO 25  == Board identifies as COM7
//#define geraet3 //ESP32 TTGO Mini V2 COM9


// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc  
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).

#ifdef geraet1
  const unsigned TX_INTERVAL = 60; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet2 //limit uplinks on testboard
  const unsigned TX_INTERVAL = 180; 
  #define BMP_ADRESS 0x76
  #define BMP280_CONNECTED //geraet1
#endif
#ifdef geraet3 //limit uplinks on testboard
  const unsigned TX_INTERVAL = 45; 
  #define BME280_CONNECTED
  #define BME_ADRESS 0x77
  #define CCS811_CONNECTED
  #define CCS811_ADRESS 0x5A
#endif

//const unsigned long UpdateInterval = (60L * 20L - 03) * 1000000L; // Update delay in microseconds
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60        /* Time ESP32 will go to sleep (in seconds) */

//WetterSymbols Size
int wettercount = 1;
#define LEFT 1
#define CENTER 2
#define RIGHT 3
boolean LargeIcon = true, SmallIcon = false;
//250x122 Pixel
int wetter_symbol_x = 210; 
int wetter_symbol_y = 30; 
int status_symbol_x = 210;
int status_symbol_y = 122;
boolean wetter_symbol_size = true;
#define Large  5           // For icon drawing, needs to be odd number for best effect
#define Small  3            // For icon drawing, needs to be odd number for best effect

#ifdef geraet1
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
#endif
#ifdef geraet2
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
#endif
#ifdef geraet3 //ESP32 TTGO MINI V2
//Check here your Version and Pinnings
  #define TTGOT5 1 //neded to indicate the E-INK Library a different pinning
  #define E_INK_PIN_SPI_BUSY 4
  #define E_INK_PIN_SPI_RST  16
  #define E_INK_PIN_SPI_DC   17
  #define E_INK_PIN_SPI_CS   13
  #define E_INK_PIN_SPI_DIN  23
  #define E_INK_PIN_SPI_SCK  18 
  #define E_INK_PIN_SPI_MISO 24//n/A

//Lora Pinning for TTGO ESP32 Mini V2 mit EINK+Lora Addon Board
  #define LORA_PIN_SPI_MOSI 5
  #define LORA_PIN_SPI_MISO 19
  #define LORA_PIN_SPI_SCK  18//14 = 1 Jumperstellung //18 = 3 Jumperstellung
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO1 27
  #define LORA_PIN_SPI_DIO0 27
#endif


//Battery Pin
  #define BATTERY_PIN 35
  
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <EEPROM.h>

//#include "LoraEncoder.h"
#include <LoraMessage.h>

//Temperatur:
#include <Wire.h>
#ifdef BMP280_CONNECTED
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 bmp; // I2C
#endif

#ifdef BME280_CONNECTED
  #define SEALEVELPRESSURE_HPA (1013.25)
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme;
#endif

#ifdef CCS811_CONNECTED 
  #include <SparkFunCCS811.h> //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
  CCS811 myCCS811(CCS811_ADRESS);
#endif


#ifdef geraet3
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#else
  #define sda 21 ///* I2C Pin Definition */
  #define scl 22 ///* I2C Pin Definition */
#endif

float temp = 23;
float pressure = 980;
float humidity = 50;
float ccs811_CO2 = 450;
float ccs811_TVOC = 2;

/*EINK Teile*/

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include "GxEPD2_boards_added.h"
// select one and adapt to your mapping, can use full buffer size (full HEIGHT)
//GxEPD2_BW<GxEPD2_213, GxEPD2_213::HEIGHT> display(GxEPD2_213(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDE0213B1, phased out
//GxEPD2_BW<GxEPD2_213_B72, GxEPD2_213_B72::HEIGHT> display(GxEPD2_213_B72(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEH0213B72
//GxEPD2_BW<GxEPD2_213_flex, GxEPD2_213_flex::HEIGHT> display(GxEPD2_213_flex(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEW0213I5F
//GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> display(GxEPD2_290(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));
//GxEPD2_BW<GxEPD2_290_T5, GxEPD2_290_T5::HEIGHT> display(GxEPD2_290_T5(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // GDEW029T5
//GxEPD2_BW<GxEPD2_it60, GxEPD2_it60::HEIGHT> display(GxEPD2_it60(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4));

#ifdef geraet3
  //GxEPD2_BW<GxEPD2_290, GxEPD2_290::HEIGHT> display(GxEPD2_290(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/  E_INK_PIN_SPI_BUSY));
  //GxEPD2_3C<GxEPD2_270c, GxEPD2_270c::HEIGHT> display(GxEPD2_270c(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY));
  //GxEPD2_BW<GxEPD2_290_T5, GxEPD2_290_T5::HEIGHT> display(GxEPD2_290_T5(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY));
  GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY)); // GDEH0213B73
#else
  GxEPD2_BW<GxEPD2_213_B73, GxEPD2_213_B73::HEIGHT> display(GxEPD2_213_B73(/*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY)); // GDEH0213B73
#endif


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

//Variables for ABP after OTAA Join
u1_t NWKSKEY[16] ;                               // LoRaWAN NwkSKey, network session key.
u1_t APPSKEY[16] ;                               // LoRaWAN AppSKey, application session key.
u4_t DEVADDR ;

#ifdef geraet1
  #define DATAVALID 0xACF2AFC2//1                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet2
  #define DATAVALID 0xACF2AFC2//3                     // Pattern for data valid in EEPROM/RTC memory
#endif
#ifdef geraet3
  #define DATAVALID 0xACF2AFCA                     // Pattern for data valid in EEPROM/RTC memory
#endif

//RealTimeVariables which don't get erased during Sleep
  struct Daten_struct {//https://esp32.com/viewtopic.php?t=11984
    uint16_t Stack[15];
    uint8_t Counter_History;
  };
  RTC_DATA_ATTR uint32_t SAVED_dataValid ;                           // DATAVALID if valid data (joined)
  RTC_DATA_ATTR uint8_t  SAVED_devaddr[4] ;                          // Device address after join
  RTC_DATA_ATTR uint8_t  SAVED_nwkKey[16] ;                          // Network session key after join
  RTC_DATA_ATTR uint8_t  SAVED_artKey[16] ;                          // Aplication session key after join
  RTC_DATA_ATTR uint32_t SAVED_seqnoUp ;                             // Sequence number       
  bool OTAA = true ;   //startup with OTAA if No EEPROM was Saved before
  //History of Data
  RTC_DATA_ATTR  struct Daten_struct Druck_Werte;                 
  RTC_DATA_ATTR  struct Daten_struct Temp_Werte;
  RTC_DATA_ATTR  struct Daten_struct CO2_Werte;
  RTC_DATA_ATTR  struct Daten_struct TVOC_Werte;                                                   
  RTC_DATA_ATTR  struct Daten_struct Feuchte_Werte;

//variables for OTAA JOIN
#ifdef geraet1
//LORA Keys 1
//Lora APP ID 012345
  static const u1_t PROGMEM APPEUI[8]={ xxx };//lsb
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ xxx};//lsb
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

  //msb
  static const u1_t PROGMEM APPKEY[16] = { xxx1 };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif
#ifdef geraet2
//LORA Keys 2
//Lora App ID 02
  static const u1_t PROGMEM APPEUI[8]={ xxx};
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ xxx };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = {xxx };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif
#ifdef geraet3
  
  //sensornet_ccs811_bme280_ App 03
  static const u1_t PROGMEM APPEUI[8]={ xxx };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ xxx };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { xxx };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif


int verbunden_indicator = 0;
int Packet_Transmission_ongoing = 0; // Display wird nur aktualisiert wenn kein Paket gesendet wird
int packet_counter =0;
int packet_counter_rx =0;
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
              saveToRTC(); // Speichere Werte
              //SAVED_dataValid = 0;// Setze zu Testzwecken Datavalid = 0 damit Daten aus dem EEPROM Gelesen werden
              //retrieveKeys(); /Rücklesen der Werte aus EEPROM
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
              for (int i = 0; i < LMIC.dataLen; i++) {
                if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                  Serial.print(F("0"));
                }
                Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
                packet_counter_rx++;
              }
              Serial.println(F(" das war der Inhalt"));
            }
            Packet_Transmission_ongoing = 0;
            saveToRTC();// Versuch
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
        //byte buffer[2];
        //LoraEncoder encoder(buffer);
        //encoder.writeTemperature(temp);
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
        
        LoraMessage message;
        //message.addUnixtime(1468075322);
        //message.addLatLng(-33.905024, 151.26648);
        message.addTemperature(temp);//1
        message.addHumidity(humidity);//2
        message.addUint16(int(pressure));//4
        message.addUint16(int(ccs811_CO2));//5
        message.addUint16(int(ccs811_TVOC));//6
        
        //message.addHumidity(
        LMIC_setTxData2(1, message.getBytes(), message.getLength(), 0);
        
        //Serial.print("Temp:");Serial.print(temp); 
        Serial.println(F("Packet queued"));
        Packet_Transmission_ongoing = 1;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup_air_sensor(void){
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

    #ifdef CCS811_CONNECTED
      CCS811Core::CCS811_Status_e returnCode = myCCS811.beginWithStatus();
      Serial.print("CCS811 begin exited with: ");
      Serial.println(myCCS811.statusString(returnCode));
      CCS811_Test();
    #endif
}

//Inits E-INK Display 
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
    
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    
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

    
    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
    
    // Set data rate and transmit power for uplink moved depending if OTAA or ABP
    
    //LMIC_setDrTxpow(DR_SF12,14);//Langsamster Modus: elendslange AirTime ~820ms aber sichere Übertragung
    //LMIC_setDrTxpow(DR_SF8,14); //Kurz schnell unzuverlässig 
    
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);//https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

    if ( OTAA )
    {
      // start joining
      Serial.println ( "Start joining" ) ;
      LMIC_setDrTxpow(DR_SF9,14); //with SF9 initial Join is faster... yes I know for this Reset Cycle it will stay on SF9 and consume more airtime
      do_send (&sendjob) ;
    }
    else
    {
      Serial.printf ( "starte mit SF8: gespeichertes SAVED_seqnoUp: %d\n", SAVED_seqnoUp ) ;
      LMIC_setDrTxpow(DR_SF8,14); //otherwise stupid ABP will start with SF12!!!
      memdmp ( "No OTAA needed - Set Session, DEVADDR:", (uint8_t*)&DEVADDR, 4 ) ;
      memdmp ( "NWKSKEY:", NWKSKEY, 16 ) ;
      memdmp ( "APPSKEY:", APPSKEY, 16 ) ;
      LMIC_setSession ( 0x13, DEVADDR, NWKSKEY, APPSKEY ) ;
      //Serial.printf ( "Seqnr set to %d\n", SAVED_seqnoUp ) ;
      LMIC.seqnoUp = SAVED_seqnoUp ;  
      do_send (&sendjob) ;
    }
    
    
    //Eingebaut in /src/lmic.c geht nicht
    //LMIC_dn2dr = EU868_DR_SF9;//https://github.com/mcci-catena/arduino-lmic/issues/455
    //LMIC_selectSubBand(0); //https://github.com/mcci-catena/arduino-lorawan/issues/74
    // Start job (sending automatically starts OTAA too)
}
//***************************************************************************************************
//                                      INIT DATA 2 RTC                                                 *
//***************************************************************************************************
// Sets basic Value for RTC Data fields
// Set all Values of Temp Pressure on first startup to a typical value
// If this is a Deep Sleep wakeup and 
//***************************************************************************************************
void INIT_DATA_2_RTC(int value, struct Daten_struct *daten){
  if (daten->Counter_History != 0){ // Da war schonmal ein Wert im RTC Memory. kein erster Startup
    Serial.println("RTC Stack war schon befüllt mit Werten... nur Sleep");
  }
  else{//in dem RTC Memory war noch nie was drin
    Serial.print("\n RTC Stack wird initalisiert mit Werten:");
    for (int i = 0; i < 15; i++){
      daten->Stack[i] = value+i;
      Serial.print(i);Serial.print(":");Serial.print(daten->Stack[i]);Serial.print("/");
    }
    daten->Counter_History = 14;
    Serial.print(" DatenCounter:");Serial.println(daten->Counter_History);
  }
}
//***************************************************************************************************
//                                      SAVE DATA 2 RTC                                                 *
//***************************************************************************************************
// Sets basic Value for RTC Data fields
// Set all Values of Temp Pressure on first startup to a typical value
// Fills in Values according to LiFo, each value is shifted one higher. latest entries gets number 0
//***************************************************************************************************
void SAVE_DATA_2_RTC(int value, struct Daten_struct *daten){
   Serial.println("Gespeicherte RTC-Daten:");
   for (int i = 14; i > 0; i--){
    daten->Stack[i] = daten->Stack[i-1];
    Serial.print(i);Serial.print(":");Serial.print(daten->Stack[i]);Serial.print("/");
   }
   daten->Stack[0] = value;
   Serial.print("0:");Serial.println(daten->Stack[0]);
}

//***************************************************************************************************
//                                      M E M D M P                                                 *
//***************************************************************************************************
// Dump memory for debug
//***************************************************************************************************
void memdmp ( const char* header, uint8_t* p, uint16_t len )
{
  uint16_t i ;                                                        // Loop control

  Serial.print ( header ) ;                                           // Show header
  for ( i = 0 ; i < len ; i++ )
  {
    if ( ( i & 0x0F ) == 0 )                                          // Continue opn next line?
    {
      if ( i > 0 )                                                    // Yes, continuation line?
      {
        Serial.printf ( "\n" ) ;                                      // Yes, print it
      }
      Serial.printf ( "%04X: ", i ) ;                                 // Print index
    }
    Serial.printf ( "%02X ", *p++ ) ;                                 // Print one data byte
  }
  Serial.println() ;
}


//***************************************************************************************************
//                                    S A V E T O R T C                                             *
//***************************************************************************************************
// Save data in RTC memory.  Every 100th call the data will also be saved in EEPROM memory.         *
// The EEPROM is also updates if OTAA was used.                                                     *
// The space in RTC memory is limited to 512 bytes.                                                 *
//***************************************************************************************************
void saveToRTC()
{
  uint16_t        eaddr ;                                  // Address in EEPROM
  uint8_t*        p ;                                      // Points into savdata

  Serial.printf ( "\n Save data to RTC memory:\n" ) ;
  memcpy ( SAVED_devaddr, &LMIC.devaddr, 4 ) ;           // Fill struct to save
  memcpy ( SAVED_nwkKey,  LMIC.nwkKey, 16 ) ;
  memcpy ( SAVED_artKey,  LMIC.artKey, 16 ) ;
  SAVED_seqnoUp = LMIC.seqnoUp ;
  SAVED_dataValid = DATAVALID ;
  memdmp ( "devaddr:", SAVED_devaddr, 4 ) ;  
  memdmp ( "artKey:",  SAVED_artKey, 16 ) ;
  memdmp ( "nwkKey:",  SAVED_nwkKey, 16 ) ;
  Serial.printf ( "SeqnoUp is %d\n", SAVED_seqnoUp ) ;
  Serial.printf ( "SeqnoDown is %d\n", LMIC.seqnoDn ) ;
  if ( ( ( LMIC.seqnoUp % 50 ) == 0 ) || OTAA )           // Need to save data in EEPROM?
  {
    int EEPROM_Data_Counter =0;
    
    Serial.println ( "Saving to EEPROM" ) ;
    p = (uint8_t*)&SAVED_dataValid ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
      
    }
    Serial.printf ( "\n Saved %d Bytes of datavalid to EEPROM", EEPROM_Data_Counter);
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of devadr to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Saved %d Bytes of nwkey to EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of artkey to EEPROM", EEPROM_Data_Counter);   
    
    p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      EEPROM.write ( eaddr, *p++ ) ;                       // Write to EEPROM
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Saved %d Bytes of seqnr to EEPROM", EEPROM_Data_Counter);   
    
    EEPROM.commit() ;                                      // Commit data to EEPROM
    Serial.printf ( "\n EEPROM operation finished Number of bytes Written: %d", EEPROM_Data_Counter);   
  }
}

//***************************************************************************************************
//                                R E T R I E V E K E Y S                                           *
//***************************************************************************************************
// Try to retrieve the keys en seqnr from non-volitile memory.                                      *
//***************************************************************************************************
void retrieveKeys()
{
  uint16_t eaddr ;                                          // Address in EEPROM
  uint8_t* p ;                                              // Pointer into savdata
  
  // return ;                                               // Return if OTAA is required
  
  //Hier Entscheidung ob RTC Gültig ist oder Defaultwerte drin und aus EEPROM Gelesen werden muss
  //z.B bei Reboot, battery getauscht usw.
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC memory valid?
  {
    Serial.println ( "Keys retrieved from RTC memory\n" ) ; // Show retrieve result 
  }
  else
  {
    Serial.println ( "\n Reading Keys from EEPROM :\n" ) ; 
    // No data vailable in RTC memory.  Use EEPROM data. Hole alles aus EEPROM
    int EEPROM_Data_Counter =0;
    p = (uint8_t*)&SAVED_dataValid ;
    for ( eaddr = EEPROM_Data_Counter ; eaddr < sizeof(SAVED_dataValid) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of datavalid from EEPROM", EEPROM_Data_Counter);   
    p = (uint8_t*)&SAVED_devaddr ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of devadr from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_nwkKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
      Serial.printf ( "\n Recovered %d Bytes of nwkKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_artKey ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of artKey from EEPROM", EEPROM_Data_Counter);   
     p = (uint8_t*)& SAVED_seqnoUp ;                               // set target pointer
    for ( eaddr = EEPROM_Data_Counter ; eaddr < (sizeof(SAVED_devaddr)+sizeof(SAVED_dataValid)+ sizeof(SAVED_nwkKey)+sizeof(SAVED_artKey)+sizeof(SAVED_seqnoUp)) ; eaddr++ )
    {
      *p++ = EEPROM.read ( eaddr ) ;                        // Move one byte to savdata
      EEPROM_Data_Counter++;
    }
    Serial.printf ( "\n Recovered %d Bytes of SeqnoUp from EEPROM", EEPROM_Data_Counter);    
    SAVED_seqnoUp += 50 ;                                // Counter may be not up-to-date
    Serial.println ( "Recovered Keys from EEPROM are:\n" ) ; 
    /*
    memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    */
  }

  //check ob jetzt nach EEPROM Lesen die Daten gültig
  if ( SAVED_dataValid == DATAVALID )                     // DATA in RTC or EEPROM memory valid?
  {
    Serial.printf ( "Valid data in NVS\n" ) ;               // Yes, show
    memdmp ( "devaddr is:",
             SAVED_devaddr, 4 ) ;
    memdmp ( "nwksKey is:",
             SAVED_nwkKey, 16 ) ;
    memdmp ( "appsKey is:",
             SAVED_artKey, 16 ) ;
    Serial.printf ( "seqnr is %d\n", SAVED_seqnoUp ) ;
    memcpy ( (uint8_t*)&DEVADDR,
             SAVED_devaddr, sizeof(DEVADDR) ) ;          // LoraWAN DEVADDR, end node device address
    memcpy ( NWKSKEY,
             SAVED_nwkKey,  sizeof(NWKSKEY) ) ;          // LoRaWAN NwkSKey, network session key.
    memcpy ( APPSKEY,
             SAVED_artKey,  sizeof(APPSKEY) ) ;          // LoRaWAN AppSKey, application session key.
    OTAA = false ;                                         // Do not use OTAA
  }
  else
  {
    Serial.printf ( "No saved data, using OTAA\n" ) ;
  }
}

void begin_sleep(){
  //esp_sleep_enable_timer_wakeup(UpdateInterval);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println(F("Starting deep-sleep period..."));
  esp_deep_sleep_start();         // Sleep for e.g. 30 minutes
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    EEPROM.begin ( 512 ) ; 
    Serial.println(F("Starting"));
    
    retrieveKeys(); 
    
    Serial.println("** Stopping WiFi+BT");
    WiFi.mode(WIFI_OFF);
    btStop();

    
    INIT_DATA_2_RTC((int)temp, &Temp_Werte);
    //INIT_DATA_2_RTC((int)(pressure-870.0), &Druck_Werte); //ziehe vom Druck den jemals gemessenen niedrigsten Druck ab 870hPa (min 870 - max 1126) damit reichen uint8_t
    INIT_DATA_2_RTC((int)(pressure), &Druck_Werte); //machs mit uint16_t
    INIT_DATA_2_RTC((int)humidity, &Feuchte_Werte);
    INIT_DATA_2_RTC((int)ccs811_CO2, &CO2_Werte);
    INIT_DATA_2_RTC((int)ccs811_TVOC, &TVOC_Werte);
    Serial.print("\n\nTest der gespeicherten Temp-Werte:");
    for (int i = 0; i<15; i++){
      Serial.print(i);Serial.print(":");Serial.print(Temp_Werte.Stack[i]);Serial.print("/");
    }
    Serial.println("ENDE");
    
    
    setup_air_sensor();
   
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
       #ifdef CCS811_CONNECTED
          CCS811_Test();
       #endif
       #ifdef geraet3
         //do not use display
       #else
          //showPartialUpdate(); 
          showPartialUpdateWetter(wetter_symbol_x, wetter_symbol_y, wetter_symbol_size);
       #endif
       //begin_sleep();
    }
}



//Parts below this line can be used independent from LORA and are only for displaying Stuff on the EINK
void showPartialUpdateWetter(int x, int y, boolean IconSize){
 int box_w = 70;
 int box_h = 65;
 //boolean IconSize = false;
 display.fillRect(x-30, y-25, box_w, box_h, GxEPD_WHITE);
 display.drawRect(x-30, y-25, box_w, box_h, GxEPD_BLACK);
 
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
  //Serial.print("Wetter-Symbol:");Serial.println(wettercount);
 }
 else{
  wettercount = 1;
 }
 Temp_Anzeige(x-20, y+15, IconSize); // externe Temperatur
 #ifdef BMP280_CONNECTED
  AQI_Anzeige(x-20, y+54, IconSize);
 //#endif
 //#ifdef CCS811_CONNECTED
    Verlauf_Anzeige(x-91, y+53, IconSize, &Temp_Werte, "Temp:");
    Verlauf_Anzeige(x-91, y-10, IconSize, &Druck_Werte, "Druck:");
    
    //Druck_Verlauf_Anzeige(x-91, y+53, IconSize);//unten
    //Druck_Verlauf_Anzeige(x-91, y-10, IconSize);//oben
 #endif
 TX_Anzeige(x-201, y+78, IconSize); 
 Online_Anzeige(x-201, y+58, IconSize);
 Temp_Anzeige_Main(x-181, y+2, IconSize);
 display.setFullWindow();
 display.display(true);
}

void AQI_Anzeige(int x, int y, boolean IconSize){
   int box_w = 70;
   int box_h = 40;
   display.fillRect(x-10, y-13, box_w, box_h, GxEPD_WHITE);
   display.drawRect(x-10, y-13, box_w, box_h, GxEPD_BLACK);
   display.setFont(&FreeMonoBold9pt7b);
   display.setCursor(x, y); display.print("AQI:"); 
   display.setCursor(x, y+20); display.print(temp); 
}

void Druck_Verlauf_Anzeige(int x, int y, boolean IconSize){
   int box_w = 70;
   int box_h = 58;
   int x2 = x-9;
   
   display.fillRect(x-10, y-13, box_w, box_h, GxEPD_WHITE);
   display.drawRect(x-10, y-13, box_w, box_h, GxEPD_BLACK);
   display.setFont(&FreeMonoBold9pt7b);

   for (int i = 0; i < 14; i++){
      int delta = 45 - Temp_Werte.Stack[i];
      display.fillRect(x2+i*5,  y+2, 4, 45,        GxEPD_BLACK);//1
      display.fillRect(x2+i*5,  y+2, 4, (45-delta),GxEPD_WHITE);//1
      //display.setCursor(x2+i*5, y+10); display.print(i); 
   }   
   /*display.fillRect(x2-10,  y+10, 4, 30, GxEPD_BLACK);//1
   display.fillRect(x2-5,  y+10, 4, 30, GxEPD_BLACK);//2
   display.fillRect(x2,    y+10, 4, 30, GxEPD_BLACK);//3
   display.fillRect(x2+5,  y+10, 4, 30, GxEPD_BLACK);//4
   display.fillRect(x2+10, y+10, 4, 30, GxEPD_BLACK);//5
   display.fillRect(x2+15, y+10, 4, 30, GxEPD_BLACK);//6
   display.fillRect(x2+20, y+10, 4, 30, GxEPD_BLACK);//7
   display.fillRect(x2+25, y+10, 4, 30, GxEPD_BLACK);//8
   display.fillRect(x2+30, y+10, 4, 30, GxEPD_BLACK);//9
   display.fillRect(x2+35, y+10, 4, 30, GxEPD_BLACK);//10
   display.fillRect(x2+40, y+10, 4, 30, GxEPD_BLACK);//11
   display.fillRect(x2+45, y+10, 4, 30, GxEPD_BLACK);//12
   display.fillRect(x2+50, y+10, 4, 30, GxEPD_BLACK);//13
   //display.fillRect(x2+55, y+10, 4, 30, GxEPD_BLACK);//14

   
   display.fillRect(x2-10,  y+10, 4, 15,GxEPD_WHITE);//1
   display.fillRect(x2-5,  y+10, 4, 25, GxEPD_WHITE);//2
   display.fillRect(x2,    y+10, 4, 10, GxEPD_WHITE);//3
   display.fillRect(x2+5,  y+10, 4,  5, GxEPD_WHITE);//4
   display.fillRect(x2+10, y+10, 4, 15, GxEPD_WHITE);//5
   display.fillRect(x2+15, y+10, 4, 25, GxEPD_WHITE);//6
   display.fillRect(x2+20, y+10, 4, 10, GxEPD_WHITE);//7
   display.fillRect(x2+25, y+10, 4,  5, GxEPD_WHITE);//8
   display.fillRect(x2+30, y+10, 4, 15, GxEPD_WHITE);//9
   display.fillRect(x2+35, y+10, 4, 10, GxEPD_WHITE);//10
   display.fillRect(x2+40, y+10, 4, 5, GxEPD_WHITE);//11
   display.fillRect(x2+45, y+10, 4, 25, GxEPD_WHITE);//12
   display.fillRect(x2+50, y+10, 4, 5, GxEPD_WHITE);//13
   //display.fillRect(x2+55, y+10, 4, 25, GxEPD_WHITE);//14
   */
   display.setFont(&FreeMonoBold9pt7b);
   display.setCursor(x-5, y); display.print("Druck:"); 
}

void Verlauf_Anzeige(int x, int y, boolean IconSize, struct Daten_struct *daten, const char* header){
   int box_w = 70;
   int box_h = 58;
   int x2 = x-9;
   int y2 = y+10;
   int max_value = 0;
   int min_value = 1500;
   int range = 30;

   for (int i = 0; i < 14; i++){//Finde max und Min Value
      if (daten->Stack[i] > max_value) max_value = daten->Stack[i];
      if (daten->Stack[i] < min_value) min_value = daten->Stack[i];
   }
   
   display.fillRect(x-10, y-13, box_w, box_h, GxEPD_WHITE);
   display.drawRect(x-10, y-13, box_w, box_h, GxEPD_BLACK);
   display.setFont(&FreeMonoBold9pt7b);
   display.setCursor(x-5, y); display.print(header); 
   Serial.println(header);Serial.print(": X_Header:");Serial.print(x-5); Serial.print(" / Y_Header:");Serial.print(y);
   Serial.print("; Min_Value: ");Serial.print(min_value);Serial.print(" / Max_Value: ");Serial.println(max_value);
   
   //Balkendiagramm invertiert ohne max Begrenzung VORSICHT!!!
   /*
   for (int i = 0; i < 14; i++){
      int delta = 45 - ((daten->Stack[i])-min_value);
      display.fillRect(x2+i*5,  y+2, 4, 45,        GxEPD_BLACK);//1
      display.fillRect(x2+i*5,  y+2, 4, (45-delta),GxEPD_WHITE);//1
   }
   */
   
   //Liniendiagramm V1
   /*
   for (int i = 0; i < 13; i++){
      int data1 = ((daten->Stack[i])-min_value);
      int data2 = ((daten->Stack[i+1])-min_value);
      Serial.print(";Data1:");Serial.print(data1);Serial.print("/ Data2:");Serial.print(data2);
      Serial.print("/ x_Start:");Serial.print(x2+i*5); Serial.print("/ y_Start:");Serial.print(y2+data1);Serial.print("/ x_Ende:");Serial.print(x2+((i+1)*5));Serial.print("/ y_Ende:");Serial.println(y2+data2);
      display.drawLine(x2+(i*5), y2+data1, x2+((i+1)*5), y2+data2, GxEPD_BLACK);//Ursprungslinie
      display.drawLine(x2+(i*5), y2-1+data1, x2+((i+1)*5), y2-1+data2, GxEPD_BLACK);//Linienverstärkung um 1 nach unten versetzt
      display.drawLine(x2+(i*5), y2-2+data1, x2+((i+1)*5), y2-2+data2, GxEPD_BLACK);//Linienverstärkung um 2 nach unten versetzt
      
   }*/
   //Liniendiagramm V2
   if (max_value - min_value !=0){//nicht durch 0 teilen
      //all save
   }
   else{ //Teile niemals durch Null
     min_value = min_value - 1;
     daten->Stack[0] = daten->Stack[0] -1;
   }
       for (int i = 0; i < 13; i++){
          float data_a = ((float)(max_value - constrain(daten->Stack[i], min_value, max_value))) / ((float)(max_value - min_value));//Serial.print(";Data_a:"); Serial.print(data_a); //constrain prüft ob Werte im Wertebereich
          float data_b = ((float)(max_value - constrain(daten->Stack[i+1], min_value, max_value))) / ((float)(max_value - min_value));// Serial.print(";Data_b:"); Serial.print(data_b); 
          int data1 = data_a * range;//Serial.print(";Data1:");Serial.print(data1);
          int data2 = data_b *range;//Serial.print("/ Data2:");Serial.print(data2);
          
          //Serial.print("/ x_Start:");Serial.print(x2+i*5); Serial.print("/ y_Start:");Serial.print(y2+data1);Serial.print("/ x_Ende:");Serial.print(x2+((i+1)*5));Serial.print("/ y_Ende:");Serial.println(y2+data2);
          display.drawLine(x2+(i*5), y2+data1, x2+((i+1)*5), y2+data2, GxEPD_BLACK);//Ursprungslinie
          display.drawLine(x2+(i*5), y2-1+data1, x2+((i+1)*5), y2-1+data2, GxEPD_BLACK);//Linienverstärkung um 1 nach unten versetzt
          display.drawLine(x2+(i*5), y2-2+data1, x2+((i+1)*5), y2-2+data2, GxEPD_BLACK);//Linienverstärkung um 2 nach unten versetzt
       }
   
   
   
}

void TX_Anzeige(int x, int y, boolean IconSize){
    int box_w = 109;
    int box_h = 25;
    display.fillRect(x-10, y-5, box_w, box_h, GxEPD_WHITE);
    display.drawRect(x-10, y-5, box_w, box_h, GxEPD_BLACK);//AussenKontur
    
    //Uplink
    display.drawLine(x,   y,   x+10, y+10, GxEPD_BLACK);//Pfeil Rechts
    display.drawLine(x+1, y, x+11, y+11, GxEPD_BLACK);
    display.drawLine(x+2, y, x+12, y+12, GxEPD_BLACK);
    display.drawLine(x+3, y, x+13, y+13, GxEPD_BLACK);
    
    display.drawLine(x,   y,   x-10, y+10, GxEPD_BLACK);//Pfeil Linker Teil
    display.drawLine(x-1, y, x-11, y+11, GxEPD_BLACK); 
    display.drawLine(x-2, y, x-12, y+12, GxEPD_BLACK); 
    display.drawLine(x-3, y, x-13, y+13, GxEPD_BLACK); 
    
    display.fillRect(x-2, y, 4, 20, GxEPD_BLACK); //Pfeil Mittelteil
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(x+15, y+15);
    display.print(packet_counter); 

    //Downlink
    int x2 = x+65;
    display.drawLine(x2,   y+18, x2+10, y+10, GxEPD_BLACK);//Pfeil Rechts
    display.drawLine(x2+1, y+18, x2+11, y+10, GxEPD_BLACK);
    display.drawLine(x2+2, y+18, x2+12, y+10, GxEPD_BLACK);
    display.drawLine(x2+3, y+18, x2+13, y+10, GxEPD_BLACK);
    
    display.drawLine(x2,   y+18, x2-10, y+10, GxEPD_BLACK);//Pfeil Linker Teil
    display.drawLine(x2-1, y+18, x2-11, y+10, GxEPD_BLACK); 
    display.drawLine(x2-2, y+18, x2-12, y+10, GxEPD_BLACK); 
    display.drawLine(x2-3, y+18, x2-13, y+10, GxEPD_BLACK); 
    
    display.fillRect(x2-2, y-2, 4, 20, GxEPD_BLACK); //Pfeil Mittelteil
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(x2+15, y+15);
    display.print(packet_counter_rx); 
}

void Online_Anzeige(int x, int y, boolean IconSize){
    int box_w = 109;
    int box_h = 25;
    total_seconds += (int)TX_INTERVAL;
    seconds = total_seconds % 60;
    minutes = (total_seconds / 60) % 60;
    hours = (total_seconds / 3600) % 24;
    days = (total_seconds / 3600) / 24;
    display.fillRect(x-10, y-5, box_w, box_h, GxEPD_WHITE);
    display.drawRect(x-10, y-5, box_w, box_h, GxEPD_BLACK);//AussenKontur   
    display.setFont(&FreeMonoBold9pt7b);
    display.setCursor(x, y+15);
    display.print(days); display.print("d "); print02d(hours); display.print(":"); print02d(minutes); //display.print(":"); print02d(seconds);
}



void Temp_Anzeige(int x, int y, boolean IconSize){
  int x_pos2=x+25;
  int y_pos = 20;
  int c_offset_x = 17;
  int c_offset_y = 5;
  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(x+5, y+y_pos);
  display.print(minutes); //display.print("C"); 
  display.setCursor(x+15, y+y_pos);
  display.print("/"); 
  display.setCursor(x_pos2, y+y_pos);
  print02d(seconds); display.print("C"); 

  display.fillCircle(x+c_offset_x, y+c_offset_y, 3, GxEPD_BLACK);
  display.fillCircle(x+c_offset_x, y+c_offset_y, 2, GxEPD_WHITE);
  display.fillCircle(x_pos2+c_offset_x+3, y+c_offset_y, 3, GxEPD_BLACK);
  display.fillCircle(x_pos2+c_offset_x+3, y+c_offset_y, 2, GxEPD_WHITE);

  
}

void Temp_Anzeige_Main(int x, int y, boolean IconSize){
  int box_w = 109;
  int box_h = 75;
  int x_pos2=x+25;
  int y_pos = 20;
  int c_offset_x = 63;
  int c_offset_y = -12;
  display.fillRect(x-30, y-25, box_w, box_h, GxEPD_WHITE);
  display.drawRect(x-30, y-25, box_w, box_h, GxEPD_BLACK);

  
  display.setFont(&FreeMonoBold9pt7b);
  display.setCursor(x+5, y); display.print(temp);
  display.setCursor(x+65, y);display.print("C");
  display.setCursor(x+5, y+y_pos); display.print((int)pressure); 
  display.setCursor(x+40, y+y_pos); display.print("hPa"); 

  display.fillCircle(x+c_offset_x, y+c_offset_y, 3, GxEPD_BLACK);
  display.fillCircle(x+c_offset_x, y+c_offset_y, 2, GxEPD_WHITE);
  //display.fillCircle(x_pos2+c_offset_x+3, y+c_offset_y, 3, GxEPD_BLACK);
  //display.fillCircle(x_pos2+c_offset_x+3, y+c_offset_y, 2, GxEPD_WHITE);

  //Symbole
  display.drawRect(x-25, y-20, box_w/4, box_h/3, GxEPD_BLACK);
  display.drawRect(x-25, y, box_w/4, box_h/3, GxEPD_BLACK);
  //display.drawRect(x-25, y, box_w/5, box_h/6, GxEPD_BLACK);
  display.setCursor(x-20, y-5);display.print("T:");
  display.setCursor(x-20, y+20);display.print("P:");
}

void Status_Info(int x, int y, boolean IconSize){
  display.setFont(&FreeMonoBold9pt7b);
  DrawBattery(x, y+5);
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
  display.setCursor(x, y+20);
  display.print("Joining");
  display.setCursor(x, y+40);
  display.print("TTN");
  display.setCursor(x, y+60);
  display.print("Network");
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
  //else if (verbunden_indicator == 2){display.print("TX Count:"); display.print(packet_counter);}
  else if (verbunden_indicator == 2){
    display.print("Temp:"); display.print(temp);
    display.setCursor(box_x, cursor_y+60);
    display.print("Press:"); display.print(pressure);
  }
  else {display.println("NO PLAN");}
  
  /*
  #ifdef BMP280_CONNECTED
    display.setCursor(box_x, cursor_y+60);
    display.print("Temp:"); display.print(temp);
  #endif  
  
  #ifdef BME280_CONNECTED
    display.setCursor(box_x, cursor_y+60);
    display.print("Temp:"); display.print(temp);
  #endif
  */
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
        status = bme.begin(BME_ADRESS);   
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
          SAVE_DATA_2_RTC((int)temp, &Temp_Werte);
          SAVE_DATA_2_RTC((int)humidity, &Feuchte_Werte);
          //SAVE_DATA_2_RTC((int)(pressure-870.0), &Druck_Werte);//Alte Version mit Uint8_t 
          SAVE_DATA_2_RTC((int)(pressure), &Druck_Werte);
        
    }
#endif

#ifdef BMP280_CONNECTED
    int BMP_Start(void){
      bool status;
        Wire.begin(sda, scl);
        
          status = bmp.begin(BMP_ADRESS,0x58); //alternative BMP Adress
          //status = bmp.begin();  
        
        
        Serial.print("BMP check: "); Serial.print(status);
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
         
           //toavoid unneccessary time for debug printouts
          /*
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
          //SAVE_DATA_2_RTC((int)temp, Temp_Werte);Serial.print("Temp:");Serial.println((int)temp);
          //SAVE_DATA_2_RTC((int)pressure, Druck_Werte);Serial.print("Druck:");Serial.println((int)pressure);
          SAVE_DATA_2_RTC((int)temp, &Temp_Werte);
          //SAVE_DATA_2_RTC((int)(pressure-870.0), &Druck_Werte); //870 = minimal jemals gemessener Druck-Alte Version
          SAVE_DATA_2_RTC((int)(pressure), &Druck_Werte);
    }
#endif 

#ifdef CCS811_CONNECTED
  void CCS811_Test(void){
      if (myCCS811.dataAvailable()){
          //Calling this function updates the global tVOC and eCO2 variables
          myCCS811.readAlgorithmResults();
          ccs811_CO2 = float(myCCS811.getCO2());
          ccs811_TVOC = float(myCCS811.getTVOC());
          
          Serial.print("CCS811 I2C: CO2 = "); Serial.print(ccs811_CO2); Serial.print(" ppm");
          Serial.print(" TVOC: "); Serial.print(ccs811_TVOC); Serial.println(" ppb");
          //Serial.print(myCCS811.getCO2());
          //Serial.print(myCCS811.getTVOC());
          
          #ifdef BME280_CONNECTED //for Calibration of Temp and humidity
            float BMEtempC = bme.readTemperature();
            float BMEhumid = bme.readHumidity();
            //Recalibration of CCS822 with BME280 Data
            myCCS811.setEnvironmentalData(BMEhumid, BMEtempC);
            Serial.print("Set CCS811 new values (deg C, %): ");
            Serial.print(BMEtempC); Serial.print(",");
            Serial.println(BMEhumid); Serial.println();
          #endif    
      }
  }
#endif

// Wetter Symbole
//#########################################################################################


void DrawBattery(int x, int y) {
  int box_w = 70;
  int box_h = 15;
  
  uint8_t percentage = 100;
  float voltage = analogRead(BATTERY_PIN) / 4096.0 * 7.46;//ist noch falscher PIN
  if (voltage > 1 ) { // Only display if there is a valid reading
    //Serial.println("Voltage = " + String(voltage));
    percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
    if (voltage >= 4.20) percentage = 100;
    if (voltage <= 3.50) percentage = 0;
    display.fillRect(x-30, y-15, box_w, box_h, GxEPD_WHITE);
    display.drawRect(x-30, y-15, box_w, box_h, GxEPD_BLACK);
    display.drawRect(x + 15, y - 12, 19, 10, GxEPD_BLACK);
    display.fillRect(x + 34, y - 10, 2, 5, GxEPD_BLACK);
    display.fillRect(x + 17, y - 10, 15 * percentage / 100.0, 6, GxEPD_BLACK);
    drawString(x + 10, y - 14, String(percentage) + "%", RIGHT);
    //drawString(x + 13, y + 5,  String(voltage, 2) + "v", CENTER);
  }
}
