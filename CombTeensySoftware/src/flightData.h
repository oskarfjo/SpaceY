#ifndef FLIGHT_DATA_H
#define FLIGHT_DATA_H

struct SensorData 
{
    float pressure;
    float roll;     // x
    float heading;  // y
    float pitch;    // z
    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;

    float initPressure;

    float altitude = 0.0;
    float altitudeMax = 0.0;
    float verticalVelocity = 0.0;

    float currentBeta = 0.3; // dynamic madwick beta
};

struct Flags
{
    bool armed = false;
    bool launchSignaled = false;
    bool armSignaled = false;
    bool parachuteSignaled = false;

    enum FlightPhase {
        PREEFLIGHT = 0,
        LAUNCHED = 1,
        FLIGHT = 2,
        APOGEE = 3,
        DESCENT = 4,
        GROUND = 5,
      };

    enum ProgramMode {
        LIVE = 0,
        LAB = 1,
        SIM = 2,
    };
      
    ProgramMode programMode = LAB;

    FlightPhase flightPhase = PREEFLIGHT;
};

struct CtrlData
{
    double dt = 0.0;
    
    double pitchSet = 0.0;
    double rollSet = 90.0;
    double filteredPitchDotError = 0.0;
    double filteredRollDotError = 0.0;
    double pitchErrorInt = 0.0;
    double rollErrorInt = 0.0;

    double gimbalPitchAngle = 0.0;
    double gimbalRollAngle = 0.0;
    
    double pitchError = 0.0;
    double rollError = 0.0;
};


extern SensorData sensorData;
extern Flags systemFlag;
extern CtrlData ctrlData;

#endif // FLIGHT_DATA_H