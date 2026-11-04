
#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// Alle Messwerte in einer Struktur
struct IMUData
{
    float accX;
    float accY;
    float accZ;

    float gyroX;
    float gyroY;
    float gyroZ;

    float temperature;
};

// Eigene Funktionen
bool IMU_begin(TwoWire &bus);

bool IMU_read(IMUData &data);

float IMU_g();

void IMU_gReset();

#endif
