#include <ESP8266WiFi.h>
#include <settings.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <EEPROM.h>

#define S_DEBUG
#define DHTPIN           D6        // Pin which is connected to the DHT sensor.
#define PIRPIN           D2        // Pin for Passive Infrared Sensor
#define RESPIN           D3        // Reset PIN for software reset
#define LDRPIN           A0        // Pin for the LDR
#define DHTTYPE          DHT11     // DHT 11

SYSCFG settings
{
    "io.adafruit.com",
    "1883",
    "username",
    "userkey",
    "sensor",
    "username/feeds/temps1",
    "username/feeds/hums1",
    "username/feeds/pirs1",
    "username/feeds/ldrs1",
    "300000"
};

int ldrValue;
byte pirState;
byte oldPirState;

//flag for saving data
bool shouldSaveConfig = false;

DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;

PubSubClient mqtt(client);

int delayTime =  300000;  //Wait 5 minutes before sending data to web
int startDelay = 0;

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while (!mqtt.connect(settings.mqtt_sensor_name, settings.mqtt_user, settings.mqtt_key)) { // connect will return 0 for connected
       Serial.println(mqtt.state());
       Serial.println("Retrying MQTT connection in 15 seconds...");
       mqtt.disconnect();
       delay(15000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }

  Serial.println("MQTT Connected!");
}

// Store structure in EEPROM(flash)
void storeStruct(void *data_source, size_t size)
{
  EEPROM.begin(size * 2);
  for(size_t i = 0; i < size; i++)
  {
    char data = ((char *)data_source)[i];
    EEPROM.write(i, data);
  }
  EEPROM.commit();
}

// load settings from EEPROM(flash)
void loadStruct(void *data_dest, size_t size)
{
    EEPROM.begin(size * 2);
    for(size_t i = 0; i < size; i++)
    {
        char data = EEPROM.read(i);
        ((char *)data_dest)[i] = data;
    }
}

void motionChange() {
     pirState = digitalRead(PIRPIN);
}

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

// reset the node incase of need to change mqtt or wifi settings
void resetNode() {
  noInterrupts();
  EEPROM.begin(512);

  // write a 0 to all 512 bytes of the EEPROM to clear it
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.end();

  WiFiManager wifiManager;
  wifiManager.resetSettings();
  ESP.restart();
}

void setup()
{

  Serial.begin(115200); // starts the serial port at 115200

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", settings.mqtt_host, 33);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT User", settings.mqtt_user, 33);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", settings.mqtt_port, 33);
  WiFiManagerParameter custom_mqtt_key("key", "MQTT Key", settings.mqtt_key, 33);
  WiFiManagerParameter custom_mqtt_sensor_name("sensorname","MQTT Sensor Name", settings.mqtt_sensor_name,33);
  WiFiManagerParameter custom_mqtt_topic_temp("topictemp", "MQTT Topic Temperature", settings.mqtt_topic_temp, 40);
  WiFiManagerParameter custom_mqtt_topic_hum("topichum", "MQTT Topic Humidity", settings.mqtt_topic_hum, 40);
  WiFiManagerParameter custom_mqtt_topic_pir("topicpir", "MQTT Topic Movement", settings.mqtt_topic_pir, 40);
  WiFiManagerParameter custom_mqtt_topic_ldr("topicldr", "MQTT Topic Light", settings.mqtt_topic_ldr, 40);
  WiFiManagerParameter custom_mqtt_interval("interval", "MQTT Send Interval", settings.mqtt_interval, 10);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_sensor_name);
  wifiManager.addParameter(&custom_mqtt_topic_temp);
  wifiManager.addParameter(&custom_mqtt_topic_hum);
  wifiManager.addParameter(&custom_mqtt_topic_pir);
  wifiManager.addParameter(&custom_mqtt_topic_ldr);
  wifiManager.addParameter(&custom_mqtt_interval);

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.autoConnect("AutoConnectSensor","getthere");

  while (WiFi.status() != WL_CONNECTED) {
    delay(1500);
  }

  Serial.println();
  Serial.println("connected...yeey :)");

  Serial.println("loading settings");
  loadStruct(&settings, sizeof(settings));

  if (shouldSaveConfig) {
    Serial.println("Saving new config");

    strcpy(settings.mqtt_host,custom_mqtt_server.getValue());
    strcpy(settings.mqtt_user,custom_mqtt_user.getValue());
    strcpy(settings.mqtt_port,custom_mqtt_port.getValue());
    strcpy(settings.mqtt_key,custom_mqtt_key.getValue());
    strcpy(settings.mqtt_sensor_name, custom_mqtt_sensor_name.getValue());
    strcpy(settings.mqtt_topic_temp,custom_mqtt_topic_temp.getValue());
    strcpy(settings.mqtt_topic_hum,custom_mqtt_topic_hum.getValue());
    strcpy(settings.mqtt_topic_ldr,custom_mqtt_topic_ldr.getValue());
    strcpy(settings.mqtt_topic_pir,custom_mqtt_topic_pir.getValue());
    strcpy(settings.mqtt_interval,custom_mqtt_interval.getValue());

    storeStruct(&settings, sizeof(settings));
  }

  // Set a resetpint and use internal pullup
  pinMode(RESPIN, INPUT_PULLUP);

  // set the pirpin to input mode
  pinMode(PIRPIN, INPUT);

  // Set the pin interrupts
  attachInterrupt(PIRPIN, motionChange, CHANGE);
  attachInterrupt(RESPIN, resetNode, CHANGE);

  oldPirState = !digitalRead(PIRPIN);

  Serial.println("settings");
  Serial.println(settings.mqtt_host);
  Serial.println(settings.mqtt_port);
  Serial.println(settings.mqtt_key);
  Serial.println(settings.mqtt_sensor_name);
  Serial.println(settings.mqtt_topic_temp);
  Serial.println(settings.mqtt_topic_ldr);
  Serial.println(settings.mqtt_topic_hum);
  Serial.println(settings.mqtt_topic_pir);
  Serial.println(settings.mqtt_interval);

  // set delay to interval from settings
  delayTime = atoi(settings.mqtt_interval);

  dht.begin();

  mqtt.setServer(settings.mqtt_host, atoi(settings.mqtt_port));

}

void loop()
{
  if (pirState != oldPirState) {
    #ifdef S_DEBUG
      Serial.println("pir state changed");
    #endif
    MQTT_connect();
    mqtt.publish(settings.mqtt_topic_pir, String(pirState).c_str());

    oldPirState = pirState;
  }

  if (millis() - startDelay < delayTime) {}
  else {
    MQTT_connect();

    ldrValue = analogRead(A0); // read analog input pin 0

    // Get temperature from dht
    float temperature = dht.readTemperature(false,true);

    #ifdef S_DEBUG
    if (isnan(temperature)) {
      Serial.println("Error reading temperature!");
    }
    else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.println(" *C");
    }
    #endif

    // Get humidity event and print its value.
    float humidity = dht.readHumidity(true);

    #ifdef S_DEBUG
    if (isnan(humidity)) {
      Serial.println("Error reading humidity!");
    }
    else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
    }
    #endif

    if (!mqtt.publish(settings.mqtt_topic_ldr, String(ldrValue).c_str(), true)) {
      Serial.println("failed to publish ldr");
    }
    delay(1000);
    if (!isnan(temperature)) {
      if (!mqtt.publish(settings.mqtt_topic_temp, String(temperature).c_str(), true)) {
        Serial.println("failed to publish temperature");
      }
    }
    delay(1000);

    if (!isnan(humidity)) {
      if (!mqtt.publish(settings.mqtt_topic_hum, String(humidity).c_str(),true)) {
        Serial.println("failed to publish humidity");
      }
    }
    startDelay = millis();
  }

}
