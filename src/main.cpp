#include <Arduino.h>
#include <Wire.h>

#include <DHT.h>
#include <DHT_U.h>
#include <SHTSensor.h>
#include <HIHReader.h>

#define BAUDRATE                            115200

#define DHT22_PIN                           A0
#define DHT22_READ_RETRIES                  100

typedef struct {
    float dht22_t   = .0;
    float dht22_h   = .0;
    float sht85_t   = .0;
    float sht85_h   = .0;
    float hih8121_t = .0;
    float hih8121_h = .0;
    float hh10d     = .0;
    float tmp102    = .0;
} wis_sensor_data_t;

void read_dht22(wis_sensor_data_t *sensor_data);

DHT dht(DHT22_PIN, DHT22);
SHTSensor sht85;
HIHReader hih8121(0x27);

wis_sensor_data_t sensor_data;


void setup() {
    Serial.begin(BAUDRATE);

    dht.begin();
}

void loop() {
    
    
    // read_dht22(&sensor_data);
    // Serial.print("DHT22 : ");
    // Serial.print(sensor_data.dht22_t); Serial.print(" C, ");
    // Serial.print(sensor_data.dht22_h); Serial.print(" RH");

    delay(1000);
}

void read_dht22(wis_sensor_data_t *sensor_data) {
    uint8_t retries = DHT22_READ_RETRIES;

    do {
        delay(100);
        sensor_data->dht22_t = dht.readTemperature();
        sensor_data->dht22_h = dht.readHumidity();
        if (isnan(sensor_data->dht22_t) || isnan(sensor_data->dht22_h)) {
            Serial.println("Failed to read from DHT sensor!");
            delay(100);
        }
        retries--;
    } while ((isnan(isnan(sensor_data->dht22_t)) || isnan(sensor_data->dht22_h)) && retries);
}