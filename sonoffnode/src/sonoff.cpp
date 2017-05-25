#include <ESP8266WiFi.h>
#include <settings.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <EEPROM.h>

#define S_DEBUG
#define LEDPIN           13        // led PIN for notifications
#define RELAYPIN         12        // Pin for the RELAY

SYSCFG settings
{
    "io.adafruit.com",
    "1883",
    "f1ybcer",
    "key",
    "sensor",
    "f1cyber/feeds/relays1"
};

//flag for saving data
bool shouldSaveConfig = false;

WiFiClient client;

PubSubClient mqtt(client);

void reconnect() {
  // Loop until we're reconnected
  while (!mqtt.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqtt.connect(settings.mqtt_sensor_name, settings.mqtt_user, settings.mqtt_key)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // ... and resubscribe
      mqtt.subscribe(settings.mqtt_topic_relay);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
// function for receiving subcribed messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(RELAYPIN, LOW);   // Turn the LED on (Note that LOW is the voltage level
    digitalWrite(LEDPIN, LOW);   // Turn the LED on (Note that LOW is the voltage level

    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(RELAYPIN, HIGH);  // Turn the LED off by making the voltage HIGH
    digitalWrite(LEDPIN, HIGH);
  }

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
  WiFiManagerParameter custom_mqtt_topic_relay("topicname","MQTT Relay topic", settings.mqtt_topic_relay,33);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_sensor_name);
  wifiManager.addParameter(&custom_mqtt_topic_relay);

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
    strcpy(settings.mqtt_topic_relay, custom_mqtt_topic_relay.getValue());

    storeStruct(&settings, sizeof(settings));
  }

  // Set a resetpint and use internal pullup
  //pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output

  pinMode(RELAYPIN,OUTPUT);
  pinMode(LEDPIN,OUTPUT);

  Serial.println("settings");
  Serial.println(settings.mqtt_host);
  Serial.println(settings.mqtt_port);
  Serial.println(settings.mqtt_user);
  Serial.println(settings.mqtt_key);
  Serial.println(settings.mqtt_sensor_name);
  Serial.println(settings.mqtt_topic_relay);

  mqtt.setServer(settings.mqtt_host, atoi(settings.mqtt_port));
  mqtt.setCallback(mqttCallback);
}

void loop()
{
  if (!mqtt.connected()) {
    reconnect();
  }
  delay(1000);
  mqtt.loop();
}
