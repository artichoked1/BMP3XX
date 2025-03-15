#include "BMP3XX.h"

#define BMP3XX_ADDRESS 0x77 // I2C address of the sensor

BMP3XX bmp;

void setup() {
    // Open serial and wait for a connection
    Serial.begin(9600);
    while (!Serial);

    Serial.println("Initializing BMP3XX...");
    // Configure sensor's settings.
    bmp.configure(BMP3XX_OVERSAMPLING_4X, BMP3XX_OVERSAMPLING_4X, BMP3XX_IIR_FILTER_OFF, BMP3XX_ODR_50HZ);
    // Initialize sensor.
    if (!bmp.begin()) {
        Serial.println("BMP3XX init failed!");
        while (1);
    }
    Serial.println("BMP3XX initialized.");
}

void loop() {
    // Read temp
    Serial.print("Temperature: ");
    Serial.print(bmp.readTemperature());
    Serial.println(" °C");

    // Read pressure
    Serial.print("Pressure: ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    delay(1000);
}
