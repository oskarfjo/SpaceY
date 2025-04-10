#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <SdFat.h>

SdFat SD;
FsFile storedData;

const unsigned long logInterval = 100; // ms
unsigned long logTimePrev = 0;

int logBufferCount = 0;
const int LOG_BUFFER_SIZE = 20;
char logBuffer[LOG_BUFFER_SIZE][128];

void logData();
void flushLogBuffer();
void initLog();

// store of values from last loop
unsigned long timePrev = 0;
unsigned long dt = 0;

double altitude = 0.0;
double pitchMeasured = 0.0;
double rollMeasured = 0.0;
double gimbalPitchAngle = 0.0;
double gimbalRollAngle = 0.0;
double pitchSet = 0.0;
double rollSet = 0.0; 
double pitchError = 0.0;
double rollError = 0.0;


//////////
// INIT //
//////////

void setup() {
  initLog();


  Serial.begin(115200);
  delay(2000);

  // init prev values
  timePrev = micros();
}


//////////
// LOOP //
//////////

void loop() {
  unsigned long timeCur = micros(); // time now in micros ; 10^{-6}s
  dt = (timeCur - timePrev) * 1e-6; // dt in seconds
  timePrev = timeCur; // updates timePrev
  // NB! averag dt = 0.006s, but when logData() is run the system slows to dt = 0.015s for that loop

  if ((timeCur - logTimePrev >= logInterval)) {
    logTimePrev = timeCur;
    logData();
  }
}


///////////////
// FUNCTIONS //
///////////////

void initLog() {
  while (!Serial) {}

  Serial.println("--- Initializing SD card ---");

  if (false) {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD initialization failed!");
    return;
  }
  }

  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD initialization failed!");
    
    // Print the error code from SdFat
    if (SD.sdErrorCode()) {
      Serial.print("SD errorCode: 0x");
      Serial.println(SD.sdErrorCode(), HEX);
      Serial.print("SD errorData: 0x");
      Serial.println(SD.sdErrorData(), HEX);
    }
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
    storedData.println("Time, dt, altitude, pitchMeasured, rollMeasured, pitchGimbal, rollGimbal, pitchSet, rollSet, pitchError, rollError"); // headers at the top csv
    storedData.flush();
  }
}


void logData() {
  if (!storedData) {
    Serial.println("Log file not open!");
    return;
  }

  snprintf(logBuffer[logBufferCount], 128,
    "%lu,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
    millis(), dt, altitude, pitchMeasured, rollMeasured, gimbalPitchAngle,
    gimbalRollAngle, pitchSet, rollSet, pitchError, rollError);

  logBufferCount++;

  if (logBufferCount >= LOG_BUFFER_SIZE) {
    flushLogBuffer();
  }

  Serial.println("data buffered");
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

  Serial.println("buffer flushed to SD");

  logBufferCount = 0;
}