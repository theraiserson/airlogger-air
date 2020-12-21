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
#define SHOW_INTERVAL 5 // Defines how many minutes are presented by one NeoPixel

void checkIaqSensorStatus(void);
void errLeds(int);
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

unsigned long now = 0;
unsigned short intervalSum = 0;
unsigned char intervalCount = 0;
unsigned char lastAccuracy = 0;
unsigned char lastIaq = 0;


void loop() {
  now = millis();
  readSensors();
  showData();
}

void checkIaqSensorStatus(void) { 
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
      for (;;)
        errLeds(1); /* Halt in case of failure */
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
      for (;;)
        errLeds(2); /* Halt in case of failure */
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
  }
}

void errLeds(int errCode)
{
  unsigned short colorPreset[3] = {0,0,0};
  unsigned short colorSet[3] = {0,0,0};
  pixels.clear();
  if (errCode <= 2) {
    colorPreset = {255,0,0};
  }
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 10; j++) {
      if (j < 5) {
        colorSet[0] = (colorPreset[0] / 5) * (j + 1);
        colorSet[1] = (colorPreset[1] / 5) * (j + 1);
        colorSet[2] = (colorPreset[2] / 5) * (j + 1);
      } else {
        colorSet[0] = (colorPreset[0] / 5) * (10 - j);
        colorSet[1] = (colorPreset[1] / 5) * (10 - j);
        colorSet[2] = (colorPreset[2] / 5) * (10 - j);
      }
      if (errCode%2 == 1) {
        pixels.setPixelColor(i, pixels.Color(colorSet); // Even errCodes will result in lights runningen from left to the right.
      } else {
        pixels.setPixelColor(7 - i, pixels.Color(colorSet); // Odd errCodes vice versa.
      }
      pixels.show();
      delay(25); // 250ms per NeoPixel, 2 seconds for all eight.
    }
  }
}

void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}
                             
void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
                             
void readSensor(void) {
  if (iaqSensor.run()) {
    lastAccuracy = iaqSensor.iaqAccuracy;
    lastIaq = iaqSensor.iaq;
    if (lastAccuracy > 1) {
      intervalCount++;
      intervalSum += lastIaq;
      updateState();
    }
  } else checkIaqSensorStatus();
}

void showData(void) {
  
}
                             
                             
