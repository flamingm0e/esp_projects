#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP085_U.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>

/************************* WiFi Access Point *********************************/

#define wifi_ssid ""
#define wifi_password ""
#define hostname ""

/************************* MQTT Broker Setup *********************************/

#define mqtt_server ""
#define mqtt_user ""
#define mqtt_password ""
#define mqtt_clientId ""

/****************************** Feeds ***************************************/

#define TOPIC_TEMP "sensors/temp"
#define TOPIC_PRESS "sensors/pressure"
#define TOPIC_DEVICE "sensors/device"
#define TOPIC_CLIENTID "sensors/clientId"

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

#define SDA D6
#define SCL D5

/***************************  Code ************************************/
//fahrenheit = 9.0/5.0*celsius+32

void setup_wifi();
void reconnect();
float convertToF(float tempVal);
bool checkBound(float newValue, float prevValue, float maxDiff);

WiFiClient espClient;
PubSubClient client(espClient);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setup() {
  Serial.begin(115200);
  delay(10);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  Wire.begin(SDA, SCL); //sda,scl
  Serial.println("Sensor Test");
  if (!bmp.begin())
  {
    Serial.print("Ooops, no BMP180 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  else {
    Serial.println("BMP180 ready.");
    delay(1000);
  }

  /* OTA code */
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network
  if (!MDNS.begin(hostname, WiFi.localIP())) {
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.print("mDNS responder started: ");
  Serial.print(hostname);
  Serial.println(".local");
  httpUpdater.setup(&httpServer);
  httpServer.begin();
  MDNS.addService("http", "tcp", 80);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(wifi_ssid);

  WiFi.begin(wifi_ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    // if (client.connect(mqtt_clientId, mqtt_username, mqtt_password)) {
    if (client.connect(mqtt_clientId)) {
      Serial.println("connected");
      client.publish(TOPIC_CLIENTID, mqtt_clientId, true);
      client.publish(TOPIC_DEVICE, "BMP180", true);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}

float convertToF(float tempVal) {
  float fah;
  fah = 9.0 / 5.0 * tempVal + 32;
  return fah;
}

bool checkBound(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

long lastMsg = 0;
float diff = 1.0;
float temp = 0.00;
float tempF = 0.00;
float pressure = 0.00;

void loop() {
  client.loop();
  //start the OTA handler
  ArduinoOTA.handle();
  httpServer.handleClient();

  long now = millis();

  if(now - lastMsg > 1000) {
    lastMsg = now;

    if (!client.connected()) {
      reconnect();
    }

    /* Get a new sensor event */
    sensors_event_t bmpEvent;
    bmp.getEvent(&bmpEvent);

    if (bmpEvent.pressure)
    {
      bmp.getTemperature(&temp);
      float newPress = bmpEvent.pressure;
      float newTempF = convertToF(temp);

      if (checkBound(newTempF, tempF, diff)) {
        tempF = newTempF;
        Serial.print("New temperature:");
        Serial.println(String(tempF).c_str());
        client.publish(TOPIC_TEMP, String(tempF).c_str(), true);
      }

      if (checkBound(newPress, pressure, 3.00)) {
        pressure = newPress;
        Serial.print("New pressure:");
        Serial.println(String(pressure).c_str());
        client.publish(TOPIC_PRESS, String(pressure).c_str(), true);
      }
    } else {
        Serial.println("Sensor error");
    }
  }
}
