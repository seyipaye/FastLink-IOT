#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "esp_camera.h"
#include <WebSerial.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */
float Voltage = 0.0;
float adc_max_voltage = 6.144; // v
float adc_resolution = 32768;  // v
float R1 = 2000.0;
float R2 = 660.0;
float milli_volt = adc_max_voltage / adc_resolution;

const char *ssid = "Oluwaseyifunmi's Hotspot";
const char *password = "Password7";

String serverName = "fastlink-mt6m.onrender.com"; // REPLACE WITH YOUR DOMAIN NAME

String serverPath = "/api/v1/iot/upload-image"; // The default serverPath should be upload.php

const int serverPort = 443; // server port for HTTPS

WiFiClientSecure client;
HTTPClient https;

AsyncWebServer server(80);

#define ADC_GPIO_NUM 34;

const int timerInterval = 30000;  // time between each HTTP POST image
unsigned long previousMillis = 0; // last time image was sent
// put function declarations here:

// int myFunction(int, int);
void sendData(String data);
void initWifi();
void print(String);
void println(String);

void setup()
{
  pinMode(34, INPUT); // It is necessary to declare the input pin
  Serial.begin(9600);
  WebSerial.begin(&server);
  ads.begin();

  initWifi();
  server.begin();

  // initCamera();

  // sendData("vfdffdf");
}

void loop()
{
  // delay(1000);
  // long value = analogRead(34);

  // Serial.println(value);
  // WebSerial.println(value);

  int16_t adc0;

  adc0 = ads.readADC_SingleEnded(0);

  Voltage = (adc0 * milli_volt * ((R1 + R2) / R2));

  Serial.print("AIN0: ");
  Serial.print(adc0);
  Serial.print("\tVoltage: ");
  Serial.println(Voltage, 7);
  Serial.println();

  delay(1000);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval)
  {
    // sendData("1234 V");
    previousMillis = currentMillis;
  }
}

void sendData(String data)
{
  client.setInsecure(); // skip certificate validation

  // Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED)
  {

    // Your Domain name with URL path or IP address with path
    https.begin(client, serverName);

    // Specify content-type header
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    // Data to send with HTTP POST
    String httpRequestData = "voltage=1223.33&id=1";

    // Send HTTP POST request
    int httpResponseCode = https.POST(httpRequestData);

    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);

    // Free resources
    https.end();
  }
  else
  {
    Serial.println("WiFi Disconnected");
  }
}

void print(String value)
{
  // Serial.print(value);
  WebSerial.print(value);
}

void println(String value)
{
  // Serial.println(value);
  WebSerial.println(value);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}

void initWifi()
{
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
}