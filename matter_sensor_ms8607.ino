/*
   matter_sensor_ms8607.ino
    Board: SparkFun Thing Plus MGM240P
    Sensor: Adafruit MS8607 StemmaQT
    Works as Full Thread Device (FTD) until SED is supported in the Arduino core.

   sketch based on:

      Matter multiple sensor example

      The example shows how to create multiple sensor instances with the Arduino Matter API.

      The example creates a Matter temparature and humidity sensor device and publishes data through them.
      The device has to be commissioned to a Matter hub first.

      Compatible boards:
      - SparkFun Thing Plus MGM240P
      - xG24 Explorer Kit

      Author: Tamas Jozsi (Silicon Labs)
*/
#define DEBUG false

#include <Matter.h>
#include <MatterTemperature.h>
#include <MatterHumidity.h>
#include <MatterPressure.h>

MatterTemperature matter_temp_sensor;
MatterHumidity matter_humidity_sensor;
MatterPressure matter_pressure_sensor;

#include "Wire.h"
#include <SparkFun_PHT_MS8607_Arduino_Library.h>
MS8607 barometricSensor;
static const uint8_t I2C_PWR = PB2;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  if (DEBUG) { Serial.begin(115200); }

  // I2C Power Pin
  pinMode(I2C_PWR, OUTPUT);
  digitalWrite(I2C_PWR, LOW);

  //--- Matter -----------------------------------------------------------------------
  Matter.begin();
  matter_temp_sensor.begin();
  matter_humidity_sensor.begin();
  matter_pressure_sensor.begin();

  if (DEBUG) { 
    Serial.println("Matter temperature, pressure and humidity sensor");

    if (!Matter.isDeviceCommissioned()) {
      Serial.println("Matter device is not commissioned");
      Serial.println("Commission it to your Matter hub with the manual pairing code or QR code");
      Serial.printf("Manual pairing code: %s\n", Matter.getManualPairingCode().c_str());
      Serial.printf("QR code URL: %s\n", Matter.getOnboardingQRCodeUrl().c_str());
    }
  }
  while (!Matter.isDeviceCommissioned()) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }

  if (!Matter.isDeviceConnected()) {
    if (DEBUG) { Serial.println("Waiting for network connection..."); }
    digitalWrite(LED_BUILTIN, HIGH);
  }
  while (!Matter.isDeviceConnected()) {
    delay(200);
  }
  if (DEBUG) { Serial.println("Device connected"); }
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  //--- Sensor MS8607 ----------------------------------------------------------------
  // SparFun Thing Plus Matter
  // SDA = PB4
  // SCL = PB3
  // power on i2c...
  digitalWrite(I2C_PWR, HIGH);
  delay(100);
  Wire.begin();

  if (barometricSensor.begin() == false) {
    if (DEBUG) { Serial.println("Failed to find MS8607 chip"); }
    while (1) { delay(10); }
  }
  if (DEBUG) { Serial.println("MS8607 Found!"); }

  int err = barometricSensor.set_humidity_resolution(MS8607_humidity_resolution_12b);
  if (DEBUG) {
    if (err != MS8607_status_ok) { 
      Serial.print("Problem setting the MS8607 sensor humidity resolution. Error code = "); 
      Serial.println(err);
    }
  }

  err = barometricSensor.disable_heater();
  if (DEBUG) {
    if (err != MS8607_status_ok) { 
      Serial.print("Problem disabling the MS8607 humidity sensor heater. Error code = "); 
      Serial.println(err);
    }
  }
  // take measurement
  float current_temp = barometricSensor.getTemperature();
  matter_temp_sensor.set_measured_value_celsius(current_temp);
  if (DEBUG) { Serial.printf("Current temperature: %.02f C\n", current_temp); }

  float current_humidity = barometricSensor.getHumidity();
  matter_humidity_sensor.set_measured_value(current_humidity);
  if (DEBUG) { Serial.printf("Current humidity: %.01f%%\n", current_humidity); }

  float current_pressure_hpa = barometricSensor.getPressure();
  matter_pressure_sensor.set_measured_value(current_pressure_hpa);
  if (DEBUG) { Serial.printf("Current pressure: %.01f hPa\n", current_pressure_hpa); }

  // power down i2c...
  Wire.end();
  digitalWrite(I2C_PWR, LOW);

  delay(60000);
}
