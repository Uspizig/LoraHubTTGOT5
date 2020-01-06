//Demo File für TTGO T5 V2.3 Board mit LORA HUB
/*
 * This Sketch tries to connect to the TTN Network
 * Current Connection Status is shown on the Eink Display every 30 Sec
 * 
 */
//TTGo T5 V2.3 Source:https://github.com/lewisxhe/TTGO-EPaper-Series
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
  #define LORA_PIN_SPI_RST  12  
  #define LORA_PIN_SPI_DIO  25

#include <SPI.h>
#include <lmic.h> //Do not forget to insert this Pull Request: https://github.com/matthijskooijman/arduino-lmic/pull/77/files
#include <hal/hal.h>
#include <GxEPD.h> // https://github.com/ZinggJM/GxEPD
#include "LoraEncoder.h" //https://github.com/thesolarnomad/lora-serialization

// select the display class to use, only one

//#include <GxGDE0213B1/GxGDE0213B1.h> // 2.13" b/w //geht nicht bei TTGO T5
//#include <GxGDEH0213B72/GxGDEH0213B72.h>  // 2.13" b/w new panel //geht nicht bei TTGO T5
#include <GxGDEH0213B73/GxGDEH0213B73.h>  // 2.13" b/w newer panel b/w, new replacement for GDE0213B1, GDEH0213B72



// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.h>
#include <GxIO/GxIO.h>


  
  
// eink SPI pin definitions:
SPIClass einkSPI(HSPI);
GxIO_Class io(einkSPI, /*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST); // arbitrary selection of 17, 16
GxEPD_Class display(io, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY); // arbitrary selection of (16), 4

// This key should be in little endian format(LSB).
static const u1_t PROGMEM APPEUI[8]={ XXXX};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format(LSB).
static const u1_t PROGMEM DEVEUI[8]=XXXX };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (MSB)
static const u1_t PROGMEM APPKEY[16] = { XXXX};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
String payload = "42";
const unsigned TX_INTERVAL = 10;
#define DISP_INTERVAL 30000
int port = 1;
int sec_A;
int sec_B;
int verbunden_indicator = 0;
float temp;
float pressure;
float humidity;

const lmic_pinmap lmic_pins = {
    .mosi = LORA_PIN_SPI_MOSI,
    .miso = LORA_PIN_SPI_MISO,
    .sck = LORA_PIN_SPI_SCK,
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LORA_PIN_SPI_DIO, LORA_PIN_SPI_DIO, LORA_PIN_SPI_DIO}, 
    //workaround to use 1 pin for all 3 radio dio pins
};
/*
 * const lmic_pinmap lmic_pins = {
    .mosi = LORA_PIN_SPI_MOSI,
    .miso = LORA_PIN_SPI_MISO,
    .sck = LORA_PIN_SPI_SCK,
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LMIC_UNUSED_PIN, LMIC_UNUSED_PIN, LMIC_UNUSED_PIN}, //workaround to use 0 pin for all 3 radio dio pins
    //This one does not work on OTAA only ABP
};
*/
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
            Serial.println(F("EV_JOINED"));
            verbunden_indicator = 2;
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
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
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            verbunden_indicator = 3;
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        Serial.println(F("Packet wird zum senden bereit gemacht"));
        byte buffer[2];
        LoraEncoder encoder(buffer);
        if (port == 1){
          encoder.writeTemperature(temp);
        }
        else if (port == 2){
          encoder.writeHumidity(humidity);
        }
        else if (port == 3){
          uint16_t i = int(pressure);
          encoder.writeUint16(i);
        }
        else{
          //encoder.writeUint16(0);
        }
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, buffer, sizeof(buffer), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}



void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("setup");
  einkSPI.begin(E_INK_PIN_SPI_SCK,E_INK_PIN_SPI_MISO,E_INK_PIN_SPI_DIN,E_INK_PIN_SPI_CS);//reMapping of SPI Pins
  display.init(115200); // enable diagnostic output on Serial 
  SPI.end(); //released SPI PINS
  
  sec_A = millis();
  
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    do_send(&sendjob); // Start job (sending automatically starts OTAA too) 
   Serial.println("Display setup done");
  
}

void loop()
{
  
  
  //drawCornerTest();
  
  if ((sec_A - sec_B) > DISP_INTERVAL) {
      //BME_Test();
      payload ="";
      sec_B = millis();
      if (port < 4){
        port++;
        showFont("FreeMonoBold12pt7b", &FreeMonoBold12pt7b);  
      }
      else{
        port = 1;
        showFont("FreeMonoBold9pt7b", &FreeMonoBold9pt7b);
      }
  }
    sec_A = millis();
    os_runloop_once();
    
}

void showFont(const char name[], const GFXfont* f)
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f);
  display.setCursor(0, 0);
  display.println();
  display.println("Status:");
  if (verbunden_indicator == 1){display.println("Joining");}
  else if (verbunden_indicator == 2){display.println("Joined");}
  else if (verbunden_indicator == 3){display.println("TX Ready");}
  else {display.println("NO PLAN");}
  display.println("Sec:");
  display.println(sec_B/1000);
  display.println("Ende");
  display.update();
  delay(5000);
}


void drawCornerTest()
{
  display.drawCornerTest();
  delay(5000);
  uint8_t rotation = display.getRotation();
  for (uint16_t r = 0; r < 4; r++)
  {
    display.setRotation(r);
    display.fillScreen(GxEPD_WHITE);
    display.fillRect(0, 0, 8, 8, GxEPD_BLACK);
    display.fillRect(display.width() - 18, 0, 16, 16, GxEPD_BLACK);
    display.fillRect(display.width() - 25, display.height() - 25, 24, 24, GxEPD_BLACK);
    display.fillRect(0, display.height() - 33, 32, 32, GxEPD_BLACK);
    display.update();
    delay(5000);
  }
  display.setRotation(rotation); // restore
}
