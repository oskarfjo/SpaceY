#include "sensor.h"
#include "flightData.h"
#include "actuator.h"
#include <Arduino.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_AHRS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

#define GPS_SERIAL Serial8
#define GPS_BAUD 9600

#define RFM95_RST   9 // RST PIN
#define RFM95_CS    8 // CS PIN
#define RFM95_INT   7 // INT PIN
#define RFM95_FREQ  868.0 // LoRa Frequency

#define GPS_PPS 33
#define GPS_FIX 36

#define FILTER_UPDATE_RATE_HZ 100
#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

// Sensor objekt
Adafruit_ISM330DHCX imu;
Adafruit_LIS3MDL mag;
Adafruit_Madgwick filter;
Adafruit_DPS310 bar = Adafruit_DPS310();

Adafruit_GPS GPS(&GPS_SERIAL);
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void updateFilterBeta(float gx, float gy, float gz);

bool calInit = false;

// Expected launch command
const char* launchCmd = "LAUNCH";
const char* armCmd = "ARM";

void initSensors(){
    Wire.begin();
    Wire.setClock(400000);
    GPS_SERIAL.begin(GPS_BAUD);
    filter.begin(FILTER_UPDATE_RATE_HZ);
    filter.setBeta(sensorData.currentBeta);

    if (!imu.begin_I2C()){
        Serial.println("Failed to initialize ISM330DHCX!");
        return;
    }

    imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    imu.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("ISM330DHCX initialized!");

    if (!mag.begin_I2C()){
        Serial.println("Failed to initialize LIS3MDL!");
        return;
    }
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
    Serial.println("LIS3MDL initialized!");

    if (!bar.begin_I2C()){
        Serial.println("Failed to initialize DPS310!");
        return;
    }
    bar.configurePressure(DPS310_128HZ, DPS310_128SAMPLES);
    bar.configureTemperature(DPS310_128HZ, DPS310_128SAMPLES);
    Serial.println("DPS310 initialized!");

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    delay(50);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
    delay(50);
    GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
    delay(50);

    pinMode(GPS_PPS, INPUT);
    pinMode(GPS_FIX, INPUT);
    Serial.println("GPS NOT initialized!");

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

void readImu(){
    sensors_event_t accel, gyro, mag_data, temp;

    // reads imu
    imu.getEvent(&accel, &gyro, &temp);
    // reads magnetometer
    mag.getEvent(&mag_data);

    if (!calInit) {
    cal.calibrate(mag_data);
    cal.calibrate(accel);
    cal.calibrate(gyro);
    calInit = true;
    }
    
    // Convert gyro readings to degrees per second
    float gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    float gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    float gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    if (true) {
    Serial.println(F("Gyro data [gx,gy,gz]: "));
    Serial.println(gx);
    Serial.println(gy);
    Serial.println(gz);
    Serial.println(F(""));
    Serial.println(F("Mag data [mx,my,mz]: "));
    Serial.println(mag_data.magnetic.x);
    Serial.println(mag_data.magnetic.y);
    Serial.println(mag_data.magnetic.z);
    Serial.println(F(""));
    Serial.println(F("Accel data [ax,ay,az]: "));
    Serial.println(accel.acceleration.x);
    Serial.println(accel.acceleration.y);
    Serial.println(accel.acceleration.z);
    Serial.println(F(""));
    Serial.println(F("---------------"));
    Serial.println(F(""));
    }
    // set appropriate filter gain
    updateFilterBeta(gx, gy, gz);
    
    // Update the AHRS filter
    filter.update(gx, gy, gz, 
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                  mag_data.magnetic.x, mag_data.magnetic.y, mag_data.magnetic.z);
    
    // Get the heading (yaw), pitch, and roll
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();

    if (true) {
        Serial.println(F("MADWICK VALS"));
        Serial.print(F("Filter beta = ")); Serial.println(sensorData.currentBeta);
        Serial.println(F(""));
        Serial.print(F("Roll = ")); Serial.println(roll);
        Serial.print(F("Pitch = ")); Serial.println(pitch);
        Serial.print(F("Heading = ")); Serial.println(heading);
        Serial.println(F(""));
        Serial.println(F("---------------"));
        Serial.println(F(""));
        }

    // the filter maps roll=[-180,180] which causes f.eks. 179.99... to jump to -179.99... when bicking horizontal
    // this is a fatal flaw that should never be experienced, but it can happen with faulty sensor readings
    // -90deg would mean straight down, so this splits the reading into either 3 or 4 quadrant of the unit circle
    // this approach ensures that the controlsystem maxes out in the correct direction even if it passes roll<0 or roll>180
    if (roll < 0 && roll >= -90) {
        roll = 1;
    } else if (roll < -90) {
        roll = 179;
    }

    sensorData.heading = heading;
    sensorData.pitch = pitch;
    sensorData.roll = roll;
    sensorData.gyroX = gx;
    sensorData.gyroY = gy;
    sensorData.gyroZ = -gz; // - due to position of imu
    sensorData.accelX = accel.acceleration.x;
    sensorData.accelY = accel.acceleration.y;
    sensorData.accelZ = accel.acceleration.z;
}

void readPressure(){
    sensors_event_t temp_event, pressure_event;

    bar.getEvents(&temp_event, &pressure_event);

    sensorData.pressure = pressure_event.pressure;
    //barData[1] = temp_event.temperature;
    //barData[2] = bar.readAltitude();
}

void updateFilterBeta(float gx, float gy, float gz) {
    // the madwickfilter needs a higher beta when the imu experiences high gyro
    // this sets an apropriate beta for the current sensorreading

    // arbitrary definitions of what is considered high and low measurements
    const float lowGyro = 20.0; // deg/s
    const float highGyro = 100.0; // deg/s

    // the beta will be in the interval [minBeta, maxBeta]
    const float minBeta = 0.3;
    const float maxBeta = 0.8;

    float gyroMagnitude = sqrt(gx*gx + gy*gy + gz*gz);

    // calculates apropriate beta
    float newBeta;
    if (gyroMagnitude <= lowGyro) {
        newBeta = minBeta;
    } else if (gyroMagnitude >= highGyro) {
        newBeta = maxBeta;
    } else {
        float ratio = (gyroMagnitude - lowGyro) / (highGyro - lowGyro);
        newBeta = minBeta + ratio * (maxBeta - minBeta);
    }

    float alpha;
    if (systemFlag.flightPhase == systemFlag.LAUNCHED) {
        alpha = 0.3;
    } else {
        alpha = 0.05;
    }

    // ensures smoth transition to new beta
    sensorData.currentBeta = sensorData.currentBeta * (1-alpha) + newBeta*alpha;

    // updates the AHRS filter beta
    filter.setBeta(sensorData.currentBeta);
}

void readGps(float gpsData[10]){
    while (GPS_SERIAL.available()) {
        GPS.read(); 
    }

    if (!GPS.newNMEAreceived()) {
        return;  
    }

    if (!GPS.parse(GPS.lastNMEA())) {
        return;  
    }

    gpsData[0] = GPS.hour;
    gpsData[1] = GPS.minute;
    gpsData[2] = GPS.seconds;
    gpsData[3] = GPS.latitudeDegrees * (GPS.lat == 'N' ? 1.0 : -1.0);
    gpsData[4] = GPS.latitudeDegrees;
    gpsData[5] = GPS.longitudeDegrees * (GPS.lon == 'E' ? 1.0 : -1.0);
    gpsData[6] = GPS.longitudeDegrees;
    gpsData[7] = GPS.altitude;
    gpsData[8] = GPS.speed;
    gpsData[9] = GPS.satellites;
    
}

void sendLoRaMessage(String message) {
    Serial.print("Sending: "); Serial.println(message);
    rf95.send((uint8_t*)message.c_str(), message.length());
    rf95.waitPacketSent();
    buzzer(100);
    Serial.println("Message sent!");  
}

void receiveLoRaMessage() {
    if (rf95.available()) {
        //Serial.println("available message");

        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);

        if (rf95.recv(buf, &len)) {
            // Null-terminate the received message
            buf[len] = 0;
            String message = String((char*)buf);
            
            // Check for proper message format: "START,CMD1,CMD2,CMD3,CMD4,END"
            if (message.startsWith("START,") && message.endsWith(",END")) {
                // Split the message by commas
                int positions[6] = {0};  // To store positions of commands
                int count = 0;
                
                for (int i = 0; i < (int)message.length() && count < 6; i++) {
                    if (message.charAt(i) == ',') {
                        positions[count] = i;
                        count++;
                    }
                }
                
                // Verify we have the correct number of commas (5 for 6 fields)
                if (count == 5) {
                    // Extract command strings
                    String commands[4];
                    for (int i = 0; i < 4; i++) {
                        commands[i] = message.substring(positions[i] + 1, positions[i + 1]);
                    }
                    
                    // Check for ARM command
                    if (commands[0] == "ARM" && !systemFlag.armSignaled) {
                        Serial.println("ARM COMMAND RECEIVED!");
                        systemFlag.armSignaled = true;
                        Serial.println("arm");
                    } else if (commands[0] == "NULL" && systemFlag.armSignaled) {
                        systemFlag.armSignaled = false;
                        disarmIgnition();
                    }
                    
                    // Check for LAUNCH command
                    if (commands[1] == "LAUNCH" && !systemFlag.launchSignaled) {
                        Serial.println("LAUNCH COMMAND RECEIVED!");
                        systemFlag.launchSignaled = true;
                        Serial.println("launch");
                    }
                    
                    // Check for PARACHUTE command
                    if (commands[2] == "PARACHUTE") {
                        Serial.println("PARACHUTE COMMAND RECEIVED!");
                        deployParachute(true);
                        systemFlag.parachuteSignaled = true;
                    } else if (commands[2] == "NULL") {
                        deployParachute(false);
                        systemFlag.parachuteSignaled = false;
                    }
                    
                    // Print full message for debugging
                    if (false) {
                        Serial.print("Received valid message: ");
                        Serial.println(message);
                        Serial.print("Commands: ");
                        for (int i = 0; i < 4; i++) {
                            Serial.print(commands[i]);
                            Serial.print(" | ");
                        }
                        Serial.println();
                        Serial.print("RSSI: ");
                        Serial.println(rf95.lastRssi(), DEC);
                    }
                } else {
                    Serial.println("Invalid message format: wrong number of commands");
                }
            } else {
                Serial.println("Invalid message format: missing START or END");
            }
        } else {
            Serial.print("Message receive failed");
        }
    }
}