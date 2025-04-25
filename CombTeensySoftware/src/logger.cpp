#include "flightData.h"
#include "logger.h"
#include <Arduino.h>
#include <SdFat.h>

SdFat SD;
FsFile storedData;

int logBufferCount = 0;
const int LOG_BUFFER_SIZE = 20;
char logBuffer[LOG_BUFFER_SIZE][128];

bool debug = false;

void initLog() {
    while (!Serial) {}
  
    Serial.println("--- Initializing SD card ---");
  
    if (!SD.begin(10)) {
      Serial.println("SD initialization failed!");
      return;
    }
  
    Serial.println("SD initialized successfully");
  
    if (SD.exists("data_log.csv")) {
      Serial.println("Existing data_log.csv found - removing it");
      SD.remove("data_log.csv");
    }
  
    storedData = SD.open("data_log.csv", O_WRITE | O_CREAT | O_APPEND);
  
    if (!storedData) {
      Serial.println("Failed to open log file!");
      return;
    }
    if (storedData.fileSize() == 0) {
      storedData.println("Time(s), dt, altitude, pitchMeasured, rollMeasured, pitchGimbal, rollGimbal, pitchError, rollError"); // headers at the top csv
      storedData.flush();
    }
  }
  
  
  void logData() {
    if (!storedData) {
      Serial.println("Log file not open!");
      return;
    }

    unsigned long timeSeconds = millis()/1000;
  
    snprintf(logBuffer[logBufferCount], 128,
             "%lu,%.5f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
             timeSeconds, ctrlData.dt, sensorData.altitude, sensorData.pitch, sensorData.roll, ctrlData.gimbalPitchAngle,
             ctrlData.gimbalRollAngle, ctrlData.pitchError, ctrlData.rollError);
  
    logBufferCount++;
  
    if (logBufferCount >= LOG_BUFFER_SIZE) {
      flushLogBuffer();
    }
    if (debug) {
      Serial.println("data buffered");
    }
  }
  
  void flushLogBuffer() {
    if (!storedData) {
      Serial.println("Log file not open!");
      return;
    }
    for (int i = 0; i < logBufferCount; i++) {
      storedData.println(logBuffer[i]);
    }
    storedData.flush();
    if (debug) {
      Serial.println("buffer flushed to SD");
    }
    logBufferCount = 0;
  }