//Demo File für TTGO T5 V2.3 Board mit LORA HUB
//Uspizig: Tested nur Partielles Update des Displays
//Geht bei ESP32 und TTGOT5 EINK Display
//Versucht zu joinen und Infos rauszublasen, 
//FEHLERBILD: JOIN kommt häufig nicht zustande: Ursache: https://www.thethingsnetwork.org/forum/t/need-help-with-mcci-lmic-and-ttn-join-wait-issue/30846

/*
 * This Sketch tries to connect to the TTN Network
 * Current Connection Status is shown on the Eink Display every 30 Sec
 * 
 */

//for Debugging
//#define SingelChannelMode 1
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
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO  25


#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
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

//Debugging
int DIO_State = 5;
int packet_counter =0;

//Temperatur:
#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temp;
float pressure;
float humidity;
  
  
// eink SPI pin definitions:
SPIClass einkSPI(VSPI);
GxIO_Class io(einkSPI, /*CS=5*/ E_INK_PIN_SPI_CS, /*DC=*/ E_INK_PIN_SPI_DC, /*RST=*/ E_INK_PIN_SPI_RST); // arbitrary selection of 17, 16
GxEPD_Class display(io, /*RST=*/ E_INK_PIN_SPI_RST, /*BUSY=*/ E_INK_PIN_SPI_BUSY); // arbitrary selection of (16), 4


#ifdef TTGOT5
  // This key should be in little endian format, see above.
  static const u1_t PROGMEM APPEUI[8]={ xxxxx };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  static const u1_t PROGMEM DEVEUI[8]={ xxxxx };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  static const u1_t PROGMEM APPKEY[16] = { xxxxx };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

#else

  static const u1_t PROGMEM APPEUI[8]={ xxxxx };
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  // This should also be in little endian format, see above.
  static const u1_t PROGMEM DEVEUI[8]={ xxxxx };
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
  
  // This key should be in big endian format (or, since it is not really a
  static const u1_t PROGMEM APPKEY[16] = { xxxxx };
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

#endif

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
String payload = "42";
const unsigned TX_INTERVAL = 10;
#define DISP_INTERVAL 30000
int port = 1;
int sec_A;
int sec_B;
int verbunden_indicator = 0;

//Partial Update:
const uint32_t partial_update_period_s = 10;
const uint32_t full_update_period_s = 60 * 60 * 60;


uint32_t start_time;
uint32_t next_time;
uint32_t previous_time;
uint32_t previous_full_update;

uint32_t total_seconds = 0;
uint32_t seconds, minutes, hours, days;
//Partial Update

const lmic_pinmap lmic_pins = {
    .mosi = LORA_PIN_SPI_MOSI,
    .miso = LORA_PIN_SPI_MISO,
    .sck = LORA_PIN_SPI_SCK,
    .nss = LORA_PIN_SPI_NSS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {LORA_PIN_SPI_DIO, LORA_PIN_SPI_DIO, LMIC_UNUSED_PIN}, 
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
    //This one does not work on OTAA
};
*/

void single_channel(void){
  Serial.println(F("SingleChannel Aktiv: "));
    #define CHANNEL  2
    
    for (uint8_t i = 0; i < 9; i++) {
      if (i != CHANNEL) {
        LMIC_disableChannel(i);
      }
    }
    //Test für eine Frequenz für Single Channel Gateway
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
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            verbunden_indicator = 2;
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
            packet_counter++;
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

void setup_eink(void){
  einkSPI.begin(E_INK_PIN_SPI_SCK,E_INK_PIN_SPI_MISO,E_INK_PIN_SPI_DIN,E_INK_PIN_SPI_CS);//reMapping of SPI Pins
  display.init(115200); // enable diagnostic output on Serial 
   Serial.println("Display setup done");
   Display_GrundAnzeige();
    
}

void setup_lora(void){
  SPI.end(); //released SPI PINS
  SPI.begin(LORA_PIN_SPI_SCK,LORA_PIN_SPI_MISO,LORA_PIN_SPI_MOSI,LORA_PIN_SPI_NSS); //CLK,MISO,MOSI,SS
  pinMode(LORA_PIN_SPI_DIO, INPUT_PULLDOWN);
  //pinMode(LORA_PIN_SPI_DIO, INPUT);
  DIO_State = digitalRead(LORA_PIN_SPI_DIO);
  Serial.print(F("GPIO25 vor OS_Init: "));Serial.println(DIO_State);

}

void setup()
{
  Serial.begin(115200); Serial.println(F("")); Serial.println(F("Starting ESP32 EINk LORA TTGO T5"));
  setup_eink(); //Initaliziere SPI Busse 
  setup_lora();
  bool status2 = BME_Start();//alles ok;
    
  //sec_A = millis();
  
    // LMIC init
    os_init();
    
    #ifdef SingelChannelMode //Test für 1 Frequenz für Debugging in TTN Console
      single_channel();
    #endif
    
    // Reset the MAC state. Session and pending data transfers will be discarded.*/
    DIO_State = digitalRead(LORA_PIN_SPI_DIO);
    Serial.print(F("GPIO25 nach OS_Init: "));Serial.println(DIO_State);
    
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    do_send(&sendjob); // Start job (sending automatically starts OTAA too) 
  
  
}

void loop()
{
  
  
  //drawCornerTest();
  if (verbunden_indicator >1){
    uint32_t actual = millis();
      while (actual < next_time)
      {
        // the "BlinkWithoutDelay" method works also for overflowed millis
        if ((actual - previous_time) > (partial_update_period_s * 1000))
        {
          Serial.print(actual - previous_time); Serial.print(" > "); Serial.println(partial_update_period_s * 1000);
          break;
        }
        delay(100);
        actual = millis();
      }
      //Serial.print("actual: "); Serial.print(actual); Serial.print(" previous: "); Serial.println(previous_time);
      if ((actual - previous_full_update) > full_update_period_s * 1000)
      {
        display.update();
        previous_full_update = actual;
      }
      previous_time = actual;
      next_time += uint32_t(partial_update_period_s * 1000);
      total_seconds += partial_update_period_s;
      seconds = total_seconds % 60;
      minutes = (total_seconds / 60) % 60;
      hours = (total_seconds / 3600) % 24;
      days = (total_seconds / 3600) / 24;
      showPartialUpdate();
  }
    if ((sec_A - sec_B) > DISP_INTERVAL) {
      //BME_Test();
      payload ="";
      sec_B = millis();
      
    }
    sec_A = millis();
    os_runloop_once();
    
}

void Display_GrundAnzeige(void){
  display.setRotation(1);
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold12pt7b);
  display.setCursor(0, 0);
  display.println();
  display.println("Status:");
  
  //display.drawExampleBitmap(BitmapExample1, 0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_BLACK);
  display.update();
  display.setFont(&FreeMonoBold12pt7b);
  // partial update to full screen to preset for partial update of box window
  // (this avoids strange background effects)
  //display.drawExampleBitmap(BitmapExample1, sizeof(BitmapExample1), GxEPD::bm_default | GxEPD::bm_partial_update);
  start_time = next_time = previous_time = previous_full_update = millis();
  display.setRotation(1);
  display.update();
}

void showPartialUpdate()
{
  uint16_t box_x = 10;
  uint16_t box_y = 35;//uint16_t box_y = 15;
  uint16_t box_w = 170;
  uint16_t box_h = 60;//uint16_t box_h = 20;
  uint16_t cursor_y = box_y + 16;
  display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
  display.setCursor(box_x, cursor_y);
  display.print(days); display.print("d "); print02d(hours); display.print(":"); print02d(minutes); display.print(":"); print02d(seconds);
  display.setCursor(box_x, cursor_y+20);
  if (verbunden_indicator == 1){display.println("Joining");}
  else if (verbunden_indicator == 2){display.println("Joined");}
  else if (verbunden_indicator == 3){display.print("TX Count:"); display.print(packet_counter);}
  else {display.println("NO PLAN");}
  display.updateWindow(box_x, box_y, box_w, box_h, true);
}

void showFont(const char name[], const GFXfont* f)
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f);
  display.setCursor(0, 70);
  display.println();
  //display.println(name);
  //display.println(" !\"#$%&'()*+,-./");
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

void print02d(uint32_t d)
{
  if (d < 10) display.print("0");
  display.print(d);
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

int BME_Start(void){
  bool status;
    status = bme.begin();  
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
