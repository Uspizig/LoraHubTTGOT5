#define TTGOT5 1
#define debug
//Lora Pinning for TTGO T5 Addon Board
  #define LORA_PIN_SPI_MOSI 15//19
  #define LORA_PIN_SPI_MISO 2//n/A
  #define LORA_PIN_SPI_SCK  14
  #define LORA_PIN_SPI_NSS  26  
  #define LORA_PIN_SPI_RST  33  
  #define LORA_PIN_SPI_DIO  25

/*******************************************************************************
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


#include "LoraEncoder.h"

#include <Adafruit_BME280.h>
Adafruit_BME280 bme;
float temp;
float pressure;
float humidity;

int port = 1;
int sec_A;
int sec_B;
int verbunden_indicator = 0;

#ifdef TTGOT5
// This key should be in little endian format, see above.
static const u1_t PROGMEM APPEUI[8]={ xxxx };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]={ xxxx };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { xxxx };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

#else

static const u1_t PROGMEM APPEUI[8]={ xxxx };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ xxxx };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
static const u1_t PROGMEM APPKEY[16] = { xxxx };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

#endif

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;
String payload = "42";
const unsigned TX_INTERVAL = 10;
#define DISP_INTERVAL 10000

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
//const unsigned TX_INTERVAL = 60;


//ESP8266 Selbstbau Lora V1.0
// Pin mapping
/*
const lmic_pinmap lmic_pins = {
    .nss = 16,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {15, 15, LMIC_UNUSED_PIN},
};
*/

//ESP32 Selbstbau Lora V1.0
// Pin mapping untested
/*
const lmic_pinmap lmic_pins = {
    .nss = 26,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {5, 5, LMIC_UNUSED_PIN},
};*/

#ifdef TTGOT5
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
#else
//ESP32 Selbstbau Lora V1.0
// Pin mapping untested
/*
const lmic_pinmap lmic_pins = {
    .nss = 26,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {5, 5, LMIC_UNUSED_PIN},
};*/
#endif


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
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            verbunden_indicator = 1;
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
        if (port < 4){
          port++;  
        }
        else{
          port = 1;
        }
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

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    
    bool status2 = BME_Start();//alles ok;
    sec_A = millis();
    
    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    

    LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);
    //LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    if ((sec_A - sec_B) > DISP_INTERVAL) {
      //BME_Test();
      payload ="";
      sec_B = millis();
      
    }
    sec_A = millis();
    os_runloop_once();
}

int BME_Start(void){
  bool status;
    status = bme.begin();  
    if (!status) {
        Serial.println("Could not find a valid BME280 I2C sensor, check wiring!");
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
