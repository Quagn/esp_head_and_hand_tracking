#include "Wire.h"
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <MPU6050_light.h>

uint8_t broadcastAddress[] = {0xE8, 0x68, 0xE7, 0xD2, 0x09, 0xA4};  // MAC receiver

MPU6050 mpu(Wire);
int16_t ReP = 0, ReR = 0, ReY = 0;      // reset value

unsigned long timer = 0;
unsigned long dataTimer = 0;  // Timer for data update

typedef struct sensor_data {
    int8_t Pitch;
    int8_t Roll;
    int8_t Yaw;
} sensor_data;
sensor_data sensorData;

void setup() {
    WiFi.mode(WIFI_STA);              // Set device as a Wi-Fi Station
    esp_now_init();                   // Init ESP-NOW
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

    Wire.begin(2, 0);   
    mpu.begin();        
    mpu.calcOffsets();
    
    pinMode(3, INPUT);                // Center switch
}

void loop() {
    unsigned long currentMillis = millis();
    
    // Update MPU6050 data
    mpu.update();

    // Check for reset button
    if (digitalRead(3) == LOW) {
        ReP = mpu.getAngleX();
        ReR = mpu.getAngleY();
        ReY = mpu.getAngleZ();
    }
    
    // Update data at a faster rate than sending
    if (currentMillis - dataTimer > 62.5) {     // Update data every 62.5ms
        sensorData.Pitch = mpu.getAngleX() - ReP;
        sensorData.Roll  = mpu.getAngleY() - ReR;
        sensorData.Yaw   = mpu.getAngleZ() - ReY;
        dataTimer = currentMillis;
    }
    
    // Send data at a slower rate
    if (currentMillis - timer > 250) { // Send data every 250ms
        // Send the data using ESP-NOW
        esp_now_send(broadcastAddress, reinterpret_cast<uint8_t*>(&sensorData), sizeof(sensorData));
        timer = currentMillis;
    }
}
