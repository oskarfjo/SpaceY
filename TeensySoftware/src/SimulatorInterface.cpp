#include <Arduino.h>
#include <Adafruit_AHRS.h>

Adafruit_Madgwick sim_filter; // kan byttast til MadGwick

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

            // Store processed values in simRead
            simRead[0] = sim_filter.getYaw();
            simRead[1] = sim_filter.getPitch();
            simRead[2] = sim_filter.getRoll();
            simRead[3] = values[9]; // Air pressure

            // Send IMU data over Serial to ROS 2
            Serial.printf("IMU,%.4f,%.4f,%.4f,%.4f\n", simRead[0], simRead[1], simRead[2], simRead[3]);
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



void publishSimulator(float simPub[2]){
    
    char pubmsg[50]; // Allocate enough space for message
    snprintf(pubmsg, sizeof(pubmsg), "SERVO,%.6f,%.6f", simPub[0], simPub[1]); // Format as a single string
    Serial.println(pubmsg); // Send entire message at once
    
    /*
    Serial.print("SERVO,");
    Serial.print(simPub[0]);
    Serial.print(",");
    Serial.println(simPub[1]);
    */
}