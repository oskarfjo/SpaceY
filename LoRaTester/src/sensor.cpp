#include "sensor.h"
#include <Arduino.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_RST   9 // RST PIN
#define RFM95_CS    8 // CS PIN
#define RFM95_INT   7 // INT PIN
#define RFM95_FREQ  868.0 // LoRa Frequency

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool calInit = false;

// Expected launch command
const char* launchCmd = "LAUNCH";
const char* armCmd = "ARM";

bool launchSignaled = false;
bool armSignaled = false;

void initSensors(){
    Wire.begin();
    Wire.setClock(400000);

    // Initialize LoRa
    pinMode(RFM95_RST, OUTPUT);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);
    digitalWrite(RFM95_RST, LOW);
    delay(10);
    digitalWrite(RFM95_RST, HIGH);
    delay(10);

    if(!rf95.init()){
        Serial.println("Failed to initilize LoRa!");
        return;
    }

    if (!rf95.setFrequency(RFM95_FREQ)) {
        Serial.println("Frequency set failed");
        return;
    }
    rf95.setTxPower(23); // , false);
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
    Serial.println("Lora initialized!");

}


void sendLoRaMessage(String message) {
    Serial.print("Sending: "); Serial.println(message);
    rf95.send((uint8_t*)message.c_str(), message.length());
    rf95.waitPacketSent();
    Serial.println("Message sent!");  
}

void receiveLoRaMessage() {
    if (rf95.available()) {
        Serial.println("available message");

        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            // Null-terminate the received message
            buf[len] = 0;
            
            // Check if it's the launch command
            if (strcmp((char*)buf, launchCmd) == 0 && !launchSignaled) {
                Serial.println("LAUNCH COMMAND RECEIVED!");
                launchSignaled = true;
                Serial.println("launch");
            } else if (strcmp((char*)buf, armCmd) == 0 && !armSignaled) {
                Serial.println("ARM COMMAND RECEIVED!");
                armSignaled = true;
                Serial.println("arm");
            }
            
            // Debug output
            Serial.print("Received: ");
            Serial.println((char*)buf);
            Serial.print("RSSI: ");
            Serial.println(rf95.lastRssi(), DEC);
        } else {
            Serial.println("nothing that fits");
        }
    }
}
