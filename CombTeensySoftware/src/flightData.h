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

    float altitude = 0.0;
    float altitudeMax = 0.0;
};

struct Flags
{
    bool armed = false;
    bool launchSignaled = false;
    bool armSignaled = false;
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

struct SimData
{
    bool const simMode = false;
};


extern SensorData sensorData;
extern Flags systemFlag;
extern CtrlData ctrlData;
extern SimData simData;

#endif // FLIGHT_DATA_H