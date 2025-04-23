#include <Arduino.h>
#include <Adafruit_AHRS.h>
#include "SimulatorInterface.h"

Adafruit_Madgwick sim_filter;

void initSimulatorinterface(){
    Serial.begin(115200);
    sim_filter.begin(100);
    sim_filter.setBeta(0.3);
    while (!Serial) { delay(10); }  // Wait for serial connection
    Serial.println("Teensy is ready");
}


void processSerialData(char *message, float simRead[12]) {
    if (strncmp(message, "DATA,", 5) == 0) {
        float values[10] = {0};
        char *token = strtok(message + 5, ",");
        int i = 0;
        while (token != NULL && i < 10) {
            values[i++] = atof(token);
            token = strtok(NULL, ",");
        }

        if (i == 10) {
            // Update AHRS filter with new IMU data
            sim_filter.update(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
            /*
            VALUES FROM SIM:
            0=gx    1=gy    2=gz    3=acc.x     4=acc.y     5=acc.z     6=mag.x     7=mag.y     8=mag.z     9=air pressure
            */

            // Store processed values in simRead
            simRead[0] = sim_filter.getYaw();
            simRead[1] = sim_filter.getPitch();
            simRead[2] = sim_filter.getRoll();
            simRead[3] = values[0];
            simRead[4] = values[2];
            simRead[5] = values[9]; // Air pressure

            // Send IMU data over Serial to ROS 2
            //Serial.printf("SENS VAL,%.4f,%.4f,%.4f,%.4f\n", simRead[0], simRead[1], simRead[2], simRead[5]);
        }
    }
}



void readSimulator(float simRead[12]) {
    static char inputBuffer[100];
    static uint8_t index = 0;

    while (Serial.available()) {
        char receivedChar = Serial.read();

        if (receivedChar == '\n') {
            inputBuffer[index] = '\0';
            processSerialData(inputBuffer, simRead);
            index = 0;
        } else if (index < sizeof(inputBuffer) - 1) {
            inputBuffer[index++] = receivedChar;
        }
    }
}



void publishSimulator(float simPub[4], float simRead[12]){

    // TEST MODUS FOR Å SJEKKE OM GAZEBO REAGERER
    static bool testMode = false;  // Skru av/på test her
    static float angle = -8.0;
    static bool increasing = true;

    if (testMode) {
        // Generer testbevegelse mellom -10 og 10 grader
        if (increasing) {
            angle += 0.05;
            if (angle >= 8.0)
                increasing = false;
        } else {
            angle -= 0.05;
            if (angle <= -8.0)
                increasing = true;
        }

        simPub[0] = angle;
        simPub[1] = angle;
    }

    //BRUK DINNA DERSOM DU VIL BERRE SENDE SERVO VERDIER, IDK FOR LATENCY??
    /*
    // Konverterer til radian uten matte bibliotek, 71/4068 = pi/180
    float RadServo0 = (simPub[0] * 71) / 4068;
    float RadServo1 = (simPub[1] * 71) / 4068;
    char pubmsg[50]; // Allocate enough space for message
    snprintf(pubmsg, sizeof(pubmsg), "SERVO,%.6f,%.6f", RadServo0, RadServo1); // Format as a single string
    //Serial.println(pubmsg); // Send entire message at once
    */

    // DINNA PRINTER MED IMU OG BAROMETER VERDIER
    // Konverterer til radian uten matte bibliotek, 71/4068 = pi/180
    float RadServo0 = (simPub[0] * 71) / 4068;
    float RadServo1 = (simPub[1] * 71) / 4068;
    char pubmsg[100];
    snprintf(pubmsg, sizeof(pubmsg),
        "FULL,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f",
        RadServo0, RadServo1, simPub[3], simPub[4], simRead[0], simRead[1], simRead[2], simRead[5]);


    Serial.println(pubmsg);
}