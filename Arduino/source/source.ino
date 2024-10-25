#include <Wire.h>
#include "SCD30.h"         
#include "Seeed_HM330X.h"  
#include <MKRWAN.h>       
#include <ArduinoLowPower.h>

// Pin Definitions
const int mosfetPin = 2;  // Pin to control the MOSFET (D2)
const int mq9Pin = A0;    // MQ9 sensor pin (A0)

// LoRaWAN Credentials
LoRaModem modem;
const char *appEui = "0000000000000000";
const char *appKey = "";

// Timer for sleep intervals
const unsigned long sleepInterval = 15 * 60 * 1000; // 15 minutes in milliseconds

HM330X sensor;
HM330XErrorCode errorCode;

void setup() { 

  Serial.begin(9600);       // Initialize serial communication
  Wire.begin();             // Initialize I2C communication

  Serial.print("Starting...");

  // Initialize SCD30 sensor
  if (!scd30.isAvailable()) {
    Serial.println("Failed to initialize SCD30 sensor!");
    while (1);
  }

  // Initialize HM3301 sensor
  errorCode = sensor.init();
  if (errorCode != NO_ERROR) {
    Serial.print("HM330X init error: ");
    Serial.println(errorCode);
    while (1);
  }

  // Initialize MOSFET control pin
  pinMode(mosfetPin, OUTPUT);
  digitalWrite(mosfetPin, LOW);  // Ensure sensors are initially off 

  // Initialize low power mode
  LowPower.attachInterruptWakeup(RTC_ALARM_WAKEUP, wakeUp, CHANGE);

  // Initialize LoRaWAN
  if (!modem.begin(EU868)) { 
    Serial.println("Failed to start LoRa modem!");
    while (1);
  }  

  // Join LoRaWAN network
  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Failed to join LoRaWAN network!");
    while (1);
  }
}

void loop() {
  // Wake up and turn on sensors
  Serial.println("Waking up and turning on sensors...");
  digitalWrite(mosfetPin, HIGH);  // Turn on sensors
  delay(30000);                   // Wait 30 seconds for sensors to stabilize

  // Read data from the sensors
  Serial.println("Reading data from sensors...");

  float scd30_data[3] = {0};
  float co2, temperature, humidity;

  // Read from SCD30 (CO2, temperature, humidity)
  if (scd30.isAvailable()) {
    scd30.getCarbonDioxideConcentration(scd30_data);
    co2 = scd30_data[0];
    temperature = scd30_data[1];
    humidity = scd30_data[2];

    Serial.print("CO2: "); Serial.println(co2);
    Serial.print("Temperature: "); Serial.println(temperature);
    Serial.print("Humidity: "); Serial.println(humidity);
  }

  // Read from HM3301 (particulate matter)
  uint8_t data[30];
  uint16_t pm1_0, pm2_5, pm10;
  errorCode = sensor.read_sensor_value(data, 29);
  if (errorCode == NO_ERROR) {
    pm2_5 = (data[6] << 8) | data[7];
    Serial.print("PM2.5: "); Serial.println(pm2_5);

  } else {
    Serial.print("HM330X read error: ");
    Serial.println(errorCode);
  }

  // Read MQ9 data
  float coConcentration = readMQ9();
  Serial.print("CO Concentration: "); Serial.println(coConcentration);

  String devEUI  = modem.deviceEUI();
  Serial.print("Device EUI: ");
  Serial.println(devEUI);

  byte devEUIBytes[8];


if (devEUI.length() == 16) {
  for (int i = 0; i < 8; i++) {
    String byteString = devEUI.substring(i * 2, i * 2 + 2);  // Get two hex digits
    devEUIBytes[i] = (byte) strtol(byteString.c_str(), NULL, 16);  // Convert hex string to byte
  }

  // Now devEUIBytes contains the 8 bytes of the EUI
  Serial.print("Parsed EUI bytes: ");
  for (int i = 0; i < 8; i++) {
      if (devEUIBytes[i] < 16) {
          Serial.print("0");  // Add leading zero for single-digit hex
      }
      Serial.print(devEUIBytes[i], HEX);
      Serial.print(" ");
  }
  Serial.println();
} else {
  Serial.println("Error: Device EUI has an incorrect length.");
}

 Serial.print("Teste");

  // Prepare payload
  byte payload[17];
  payload[0] = highByte((uint16_t)co2);
  payload[1] = lowByte((uint16_t)co2);
  payload[2] = highByte((uint16_t)(temperature * 100));  // Multiply by 100 to avoid decimals
  payload[3] = lowByte((uint16_t)(temperature * 100));
  payload[4] = highByte((uint16_t)(humidity * 100));
  payload[5] = lowByte((uint16_t)(humidity * 100));
  payload[6] = highByte(pm2_5);
  payload[7] = lowByte(pm2_5);
  payload[8] = lowByte((uint16_t)(coConcentration)); 

  for (int i = 0; i < 8; i++) {
    payload[9 + i] = devEUIBytes[i];
  }

  // Send data via LoRaWAN
  Serial.println("Sending data via LoRaWAN...");
  modem.beginPacket();
  modem.write(payload, sizeof(payload));
  int result = modem.endPacket(false);   // true to send and wait for confirmation
  if (result == 1) {
    Serial.println("LoRaWAN packet sent successfully.");
  } else {
    Serial.print("LoRaWAN packet transmission failed. Error code: ");
    Serial.println(result);
  }

  // Turn off MOSFET (sensors)
  Serial.println("Turning off sensors...");
  digitalWrite(mosfetPin, LOW);  // Turn off SCD30 and HM3301

  // Enter deep sleep
  goToSleep();
}

float mq9R0Value() {
  float sensor_volt;
  float RS_air; 
  float R0;  
  float sensorValue;

  /*--- Get a average data by testing 100 times ---*/
  for(int x = 0 ; x < 100 ; x++)
  {
      sensorValue = sensorValue + analogRead(A0);
  }
  sensorValue = sensorValue/100.0;
  /*-----------------------------------------------*/

  sensor_volt = sensorValue/1024*5.0;
  RS_air = (5.0-sensor_volt)/sensor_volt;
  R0 = RS_air/9.9; 

  return R0;
}

float readMQ9() {
  float sensor_volt;
  float RS_gas; 
  float ratio; 
  int sensorValue = analogRead(mq9Pin);
  sensor_volt=(float)sensorValue/1024*5.0;
  RS_gas = (5.0-sensor_volt)/sensor_volt; 

        
  ratio = RS_gas/mq9R0Value();  // ratio = RS/R0
        

  Serial.print("sensor_volt = ");
  Serial.println(sensor_volt);
  Serial.print("RS_ratio = ");
  Serial.println(RS_gas);
  Serial.print("Rs/R0 = ");
  Serial.println(ratio);

  Serial.print("\n\n");

  return ratio;
}

void goToSleep() {
  Serial.println("Entering deep sleep ...");
  LowPower.deepSleep(sleepInterval);  // Sleep
}

void wakeUp() {
  // Do nothing, the loop will continue after waking up
}
