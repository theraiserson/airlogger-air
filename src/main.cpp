#include <Arduino.h>
#include <EEPROM.h>
#include "bsec.h"
#include <Adafruit_NeoPixel.h>

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define PIN 14
#define NUMPIXELS 8
#define SAVE_STATE_PERIOD (unsigned long) 7200000 // Every 120 Minutes (120 * 60 * 1000)
#define SHOW_INTERVAL 300000 // Defines how many minutes are presented by one NeoPixel (Ervery 5 * 60 * 1000)

void checkIaqSensorStatus(void);
void errLeds(int);
void loadState(void);
void updateState(void);
void readSensor(void);
void processData(void);
void showData(void);

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
Bsec iaqSensor;

int iaqHistory[8] = {0};
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void setup() {
	Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(64);
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

bool newData = false;
unsigned long now = 0;
unsigned short intervalSum = 0;
unsigned char intervalCount = 0;
unsigned char lastAccuracy = 0;
unsigned char lastIaq = 0;
unsigned int lastDivider = 0;
unsigned int rollingIaq[8] = {0};
unsigned short lastThreeIaq[3] = {0};


void loop() {
  now = millis();
  readSensor();
  processData();
  showData();
}

void checkIaqSensorStatus(void) { 
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      Serial.println("BSEC error code : " + String(iaqSensor.status));
      for (;;)
        errLeds(1); /* Halt in case of failure */
    } else {
      Serial.println("BSEC warning code : " + String(iaqSensor.status));
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      Serial.println("BME680 error code : " + String(iaqSensor.bme680Status));
      for (;;)
        errLeds(2); /* Halt in case of failure */
    } else {
      Serial.println("BME680 warning code : " + String(iaqSensor.bme680Status));
    }
  }
}

void errLeds(int errCode)
{
  unsigned short colorPreset[3] = {0,0,0};
  unsigned short colorSet[3] = {0,0,0};
  pixels.clear();
  if (errCode <= 2) {
    colorPreset[0] = 255;
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
        pixels.setPixelColor(i, colorSet[0], colorSet[1], colorSet[2]); // Even errCodes will result in lights runningen from left to the right.
      } else {
        pixels.setPixelColor(7 - i, colorSet[0], colorSet[1], colorSet[2]); // Odd errCodes vice versa.
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
    if ((stateUpdateCounter * SAVE_STATE_PERIOD) < millis()) {
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
    Serial.println(String(now) + " IAQ: " + String(lastIaq) + " Accuracy: " + String(lastAccuracy));
    if (lastAccuracy > 1) {
      intervalCount++;
      intervalSum += lastIaq;
      updateState();
    }
    newData = true;
  } else checkIaqSensorStatus();
}

void processData(void) {
  if (newData) {
    if ((now / SHOW_INTERVAL) > lastDivider) {
      for (char i = 0; i < 7; i++)
      {
        rollingIaq[7-i] = rollingIaq[6-i];
      }
      lastDivider = (now / SHOW_INTERVAL);
      intervalSum = 0;
      intervalCount = 0;
      rollingIaq[0] = 0;
    }    
    if (intervalCount != 0) rollingIaq[0] = (intervalSum / intervalCount);
    lastThreeIaq[2] = lastThreeIaq[1];
    lastThreeIaq[1] = lastThreeIaq[0];
    lastThreeIaq[0] = lastIaq;

    newData = false;
  }
}

void showData(void) {
  short lastThreeIaqAv = (lastThreeIaq[0] + lastThreeIaq[1] + lastThreeIaq[2]) / 3;
  if (lastThreeIaqAv < 75) {
    pixels.setPixelColor(0, 0, 127, 0);
  } else if (lastThreeIaqAv < 150) {
    pixels.setPixelColor(0, 127, 127, 0);
  } else if (lastThreeIaqAv < 200) {
    pixels.setPixelColor(0, 80, 50, 15);
  } else if (lastThreeIaqAv < 250) {
    pixels.setPixelColor(0, 127, 0, 0);
  } else {
    pixels.setPixelColor(0, 102, 0, 51);
  }
   
  for (int i = 1; i < 8; i++)
  {
    if (rollingIaq[i] < 75) {
      pixels.setPixelColor(i, 0, 127, 0);
    } else if (rollingIaq[i] < 150) {
      pixels.setPixelColor(i, 127, 127, 0);
    } else if (rollingIaq[i] < 200) {
      pixels.setPixelColor(i, 80, 50, 15);
    } else if (rollingIaq[i] < 250) {
      pixels.setPixelColor(i, 127, 0, 0);
    } else {
      pixels.setPixelColor(i, 102, 0, 51);
    }
  }
  pixels.show();

}
                             
                             