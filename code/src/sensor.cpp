#include <ESP8266WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            D6        // Pin which is connected to the DHT sensor.
#define PIRPIN            D4
#define RESPIN            D3
#define LDRPIN            A0

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "f1cyber"
#define AIO_KEY         "3fbe669b986242eebd71a8d07ab7da28"

#define DHTTYPE           DHT11     // DHT 22 (AM2302)

int sensorValue;
byte pirState;
byte oldPirState;

DHT_Unified dht(DHTPIN, DHTTYPE);
WiFiClient client;

int delayTime =  10000;//300000;  //Wait 5 minutes before sending data to web
int startDelay = 0;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temps1");
Adafruit_MQTT_Publish hum = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/hums1");
Adafruit_MQTT_Publish ldr = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lights1");

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
}

void motionChange() {
    pirState = digitalRead(PIRPIN);
}

void resetNode() {
  noInterrupts();
  WiFiManager wifiManager;
  wifiManager.resetSettings();
  ESP.restart();
}

void setup()
{

  Serial.begin(115200); // starts the serial port at 9600
  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", "", 40);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt user", "", 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", "", 40);
  WiFiManagerParameter custom_mqtt_key("key", "mqtt key", "", 40);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_key);

  wifiManager.autoConnect("AutoConnectAP");

  //wifiManager.resetSettings();

  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
  }

  Serial.println();
  Serial.println("connected...yeey :)");
  // Initialize device.
  pinMode(RESPIN, INPUT_PULLUP);
  pinMode(PIRPIN, INPUT);

  attachInterrupt(PIRPIN, motionChange, CHANGE);
  attachInterrupt(RESPIN, resetNode, CHANGE);

  oldPirState = !digitalRead(PIRPIN);

  delay(2500);
  dht.begin();
  delay(2500);

  #ifdef S_DEBUG
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");
  Serial.println("------------------------------------");
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Humidity");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println("%");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println("%");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println("%");
  Serial.println("------------------------------------");
  #endif

  ArduinoOTA.onStart([]() {
  String type;
  if (ArduinoOTA.getCommand() == U_FLASH)
    type = "sketch";
  else // U_SPIFFS
    type = "filesystem";

  // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  Serial.println("Start updating " + type);
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
}

void loop()
{
  ArduinoOTA.handle();

  MQTT_connect();

  if (pirState != oldPirState) {
    Serial.println("pir state changed");
    oldPirState = pirState;
  }

  if (millis() - startDelay < delayTime) {
  }
  else {
    startDelay = millis();
    sensorValue = analogRead(A0); // read analog input pin 0
    ldr.publish(sensorValue);

    // Delay between measurements.
    // Get temperature event and print its value.
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    if (isnan(event.temperature)) {
      Serial.println(event.temperature);
      Serial.println("Error reading temperature!");
    }
    else {
      temp.publish(event.temperature);
      Serial.print("Temperature: ");
      Serial.print(event.temperature);
      Serial.println(" *C");
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
      Serial.println("Error reading humidity!");
    }
    else {
      hum.publish(event.relative_humidity);
      Serial.print("Humidity: ");
      Serial.print(event.relative_humidity);
      Serial.println("%");
    }
  }
  if(! mqtt.ping()) { mqtt.disconnect(); }
}
