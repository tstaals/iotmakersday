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
#define BUTTONPIN        0         // Pin of button

SYSCFG settings
{
    "io.adafruit.com",
    "1883",
    "f1cyber",
    "key",
    "sensor",
    "f1cyber/feeds/relays1",
    "Aan",
    "Uit"
};

//flag for saving data
bool shouldSaveConfig = false;
unsigned long buttonDownTime = 0;
byte lastButtonState = 1;
byte buttonPressHandled = 0;

WiFiClient client;

PubSubClient mqtt(client);

void publishRelay() {
  bool on = digitalRead(RELAYPIN) == HIGH;
  mqtt.publish(settings.mqtt_topic_relay,  String((on ? settings.mqtt_topic_relay_on : settings.mqtt_topic_relay_off)).c_str());
}

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
  for (int i = 0; i < 4; i++)
  {
    digitalWrite(LEDPIN, HIGH);
    delay(100);
    digitalWrite(LEDPIN, LOW);
    delay(100);
  }

  char dataPayload[length+1];
  memcpy(dataPayload, payload, sizeof(dataPayload));
  dataPayload[sizeof(dataPayload)-1] = 0;
  if (strcmp(dataPayload, settings.mqtt_topic_relay_on) == 0) {
    digitalWrite(RELAYPIN, LOW);
    digitalWrite(LEDPIN, LOW);
  } else {
    digitalWrite(RELAYPIN, HIGH);
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
  WiFiManagerParameter custom_mqtt_topic_relay_on("topicon","MQTT Relay Aan", settings.mqtt_topic_relay_on,4);
  WiFiManagerParameter custom_mqtt_topic_relay_off("topicoff","MQTT Relay Uit", settings.mqtt_topic_relay_off,4);


  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_key);
  wifiManager.addParameter(&custom_mqtt_sensor_name);
  wifiManager.addParameter(&custom_mqtt_topic_relay);
  wifiManager.addParameter(&custom_mqtt_topic_relay_on);
  wifiManager.addParameter(&custom_mqtt_topic_relay_off);

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
    strcpy(settings.mqtt_topic_relay_on, custom_mqtt_topic_relay_on.getValue());
    strcpy(settings.mqtt_topic_relay_off, custom_mqtt_topic_relay_off.getValue());

    storeStruct(&settings, sizeof(settings));
  }

  // Set a resetpint and use internal pullup
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);

  Serial.println("settings");
  Serial.println(settings.mqtt_host);
  Serial.println(settings.mqtt_port);
  Serial.println(settings.mqtt_user);
  Serial.println(settings.mqtt_key);
  Serial.println(settings.mqtt_sensor_name);
  Serial.println(settings.mqtt_topic_relay);
  Serial.println(settings.mqtt_topic_relay_on);
  Serial.println(settings.mqtt_topic_relay_off);

  mqtt.setServer(settings.mqtt_host, atoi(settings.mqtt_port));
  mqtt.setCallback(mqttCallback);
}

void loop()
{
  if (!mqtt.connected()) {
    reconnect();
  }

  byte buttonState = digitalRead(BUTTONPIN);

  if ( buttonState != lastButtonState ) {
    if (buttonState == LOW) {
      buttonDownTime     = millis();
      buttonPressHandled = 0;
    } else {
      unsigned long dt = millis() - buttonDownTime;
      if ( dt >= 90 && dt <= 900 && buttonPressHandled == 0 ) {
        publishRelay();
        buttonPressHandled = 1;
      }
      if ( dt >= 901 && dt <= 2000 && buttonPressHandled == 0 ) {
        resetNode();
        buttonPressHandled = 1;
      }
    }

    lastButtonState = buttonState;
  }

  mqtt.loop();
}
