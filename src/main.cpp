#include <Arduino.h>
#include <EEPROM.h>
#include "bsec.h"
#include <Adafruit_NeoPixel.h>

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define PIN 6
#define NUMPIXELS 8
#define SAVE_STATE_PERIOD 7200000 // Every 120 Minutes (120 * 60 * 1000)

void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);
void readSensor(void);
void showData(void);

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Bsec iaqSensor;

int iaqHistory[8] = {0};
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void setup() {
	Serial.begin(9600);
  pixels.begin();
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
  Wire.begin();
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  
  Serial.println("BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix));
  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
  
  loadState();

  bsec_virtual_sensor_t sensorList[1] = {
    BSEC_OUTPUT_IAQ,
  };
  iaqSensor.updateSubscription(sensorList, 1, BSEC_SAMPLE_RATE_LP);
  
}
