#include "Arduino.h"
#include "ESP8266WiFi.h"
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "PubSubClient.h"
#include "SimpleTimer.h"
#include <ArduinoJson.h>
#include <RemoteDebug.h>
#include <NTPClient.h>

#include "secrets.h"

#define ROOMBA_WAKEUP D6

struct Sensors
{
  bool hasData;
  unsigned int bytesRead;
  bool bumpRight;
  bool bumpLeft;
  bool wheelDropRight;
  bool wheelDropLeft;
  unsigned int dirtLevel;
  int chargingState;
  int current;
  int voltage;
  int temperature;
  unsigned int batteryCharge;
  unsigned int batteryCapacity;
  unsigned int batteryPercent;
  bool isExternalCharger;
  bool isHome;
};


WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
SimpleTimer timer;
RemoteDebug Debug;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

/* 
def _twosComplementInt2bytes( highByte, lowByte ):
    """ returns an int which has the same value
    as the twosComplement value stored in
    the two bytes passed in
    
    the output range should be -32768 to 32767
    
    chars or ints can be input, both will be
    truncated to 8 bits
    """
    # take everything except the top bit
    topbit = _bitOfByte( 7, highByte )
    lowerbits = highByte & 127
    unsignedInt = lowerbits << 8 | (lowByte & 0xFF)
    if topbit == 1:
        # with sufficient thought, I've convinced
        # myself of this... we'll see, I suppose.
        return unsignedInt - (1 << 15)
    else:
        return unsignedInt

*/

int getBitFromByte(int byte, int bit)
{
  return (byte >> bit) & 1;
}


// the output range should be -128 to 128
int signed1BytesInt(int byte)
{
  int topBit = getBitFromByte(byte, 7);
  int lowerBits = byte & 127;
  if (topBit == 1) {
  return lowerBits - (1 << 7);
  } else {
  return lowerBits;
}   
}

// the output range should be -32768 to 32767
int signed2BytesInt(int highByte, int lowByte)
 {
   // 1. Take everything except the top bit from the high byte
   int topBit = getBitFromByte(highByte, 7);
   int lowerBits = highByte & 127;
   int unsignedInt = lowerBits << 8 | (lowByte & 0xFF);

  if (topBit == 1) {
    return unsignedInt - (1 << 15);
  } else {
    return unsignedInt;
  }    
 }

 int unsigned2BytesInt(int highByte, int lowByte)
 {
   return highByte << 8 | lowByte;
 }

void wakeUp() 
{
  debugV("Waking up...");
  pinMode(ROOMBA_WAKEUP, OUTPUT);
  delay(100);
  digitalWrite(ROOMBA_WAKEUP, HIGH);
  delay(100);
  digitalWrite(ROOMBA_WAKEUP, LOW);
  delay(500);
  digitalWrite(ROOMBA_WAKEUP, HIGH);
  debugV("Wakeup complete!");
}

void clean() {
  debugV("Sending 'clean' command");
  Serial.write(128);
  delay(50);
  Serial.write(135);
}

void cleanMax() {
  debugV("Sending 'cleanMax' command");
  Serial.write(128);
  delay(50);
  Serial.write(136);
}

void cleanSpot() {
  debugV("Sending 'cleanSpot' command");
  Serial.write(128);
  delay(50);
  Serial.write(134);
}

void seekDock() {
  debugV("Sending 'seekDock' command");
  Serial.write(128);
  delay(50);
  Serial.write(143);
}

void stop() {
  debugV("Sending 'stop' command");
  Serial.write(128);
  delay(50);
  Serial.write(173);
}

void powerOff() {
  debugV("Sending 'powerOff' command");
  Serial.write(128);
  delay(50);
  Serial.write(133);
}

void flush() {
  debugV("Flushing serial buffer");
  int bytes = Serial.available();
  char buf[bytes];
  if(bytes > 0) {
    Serial.readBytes(buf, bytes);
  }
}


bool updateTime() 
{
  debugV("Updating time with NTP");
  timeClient.begin();
  timeClient.setTimeOffset(NTP_TIME_OFFSET);
  bool result = timeClient.forceUpdate();
  if(result) {
    int day = timeClient.getDay();
    int hour = timeClient.getHours();
    int minutes = timeClient.getMinutes();

    Serial.write(128);
    delay(50);

    Serial.write(168);
    Serial.write(day);
    Serial.write(hour);
    Serial.write(minutes);
     debugV("Updating time success (%d:%d:%d)", day, hour, minutes);
  }

  return result;
  
}

Sensors updateSensors() {
  Sensors sensors;

  // List of sensors to update
  // 7 - bumps & wheel drops, 1 byte | 0
  // 15 - dirt detect, 1 byte | 8
  // 21 - charging state, 1 byte | 16
  // 22 - Voltage, 2 bytes unsigned | 17-18
  // 23 - Current, 2 bytes unsigned | 19-20
  // 25 - Battery charge, 2 bytes | 22-23
  // 26 - Battery capacity, 2 bytes | 24-25
  
  flush();

  debugV("Updating sensors start...");
  Serial.write(128);
  Serial.write(142);
  Serial.write(6);

  delay(100);
 
  int i = Serial.available();
  char sensorbytes[100];
  if(i > 0) {
    Serial.readBytes(sensorbytes, i);

    sensors.hasData = true;
    sensors.bytesRead = i;
    sensors.bumpRight = sensorbytes[0] & 1;
    sensors.bumpLeft = sensorbytes[0] & 2;
    sensors.wheelDropRight = sensorbytes[0] & 4;
    sensors.wheelDropLeft = sensorbytes[0] & 8;

    sensors.dirtLevel = sensorbytes[8];


    sensors.chargingState = sensorbytes[16];
    sensors.isHome = sensors.chargingState > 0 && sensors.chargingState < 4;

    sensors.voltage = unsigned2BytesInt(sensorbytes[17], sensorbytes[18]);
    sensors.current = signed2BytesInt(sensorbytes[19], sensorbytes[20]);
    sensors.temperature = signed1BytesInt(sensorbytes[21]);


    sensors.batteryCharge = unsigned2BytesInt(sensorbytes[22],sensorbytes[23]);
    sensors.batteryCapacity = unsigned2BytesInt(sensorbytes[24], sensorbytes[25]);
    sensors.batteryPercent = 100 * sensors.batteryCharge / sensors.batteryCapacity;
    debugV("Updating sensors success!");
  } else {
    sensors.hasData = false;
    debugV("Updating sensors failed");
  }

  return sensors;
}


String getState(Sensors sensors) {

  if(sensors.isHome) {
    return "docked"; //TODO Add "return_to_base" status based on last command (probably)
  }

  if(sensors.current < MIN_CLEANING_CURRENT) {
    return "cleaning";
  }

  return "idle";
}

String getFanSpeed(String state) {

  return state == "cleaning" ? "max" : "off";
}

void publishSensorsInformation() {
  if(!mqttClient.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }
  
  Sensors sensors = updateSensors();
  if(sensors.hasData) {

    StaticJsonDocument<500> doc;
    
    String state = getState(sensors);
    doc["state"] = state;
    doc["fan_speed"] = getFanSpeed(state);
    doc["battery_level"] = sensors.batteryPercent;

    doc["bump_left"] = sensors.bumpLeft;
    doc["bump_right"] = sensors.bumpRight;

    doc["wheel_drop_left"] = sensors.wheelDropLeft;
    doc["wheel_drop_right"] = sensors.wheelDropRight;

    doc["dirt_level"] = sensors.dirtLevel;

    doc["voltage"] = sensors.voltage;

    doc["battery_charge"] = sensors.batteryCharge;
    doc["battery_capacity"] = sensors.batteryCapacity;

    doc["charging_state"] = sensors.chargingState;
    doc["current"] = sensors.current;

    doc["temperature"] = sensors.temperature;

    doc["bytes"] = sensors.bytesRead;

    String str;
    serializeJson(doc, str);
    mqttClient.publish(MQTT_STATE_TOPIC, str.c_str());
    debugV("Sensors info published to MQTT");
  }
}

void publishHomeAssistantAutoDiscovery(String uniqueId, String name, String valueTemplate, String deviceClass, String unitOfMeasurement, String topicType, bool isVacuum)
{
  DynamicJsonDocument doc(500);
  String fullName;
  String fullId;
  if(name == "") {
    fullName =  String(MQTT_CLIENT_NAME);
  } else {
    fullName = String(MQTT_CLIENT_NAME) + " " + name;
  }

  if(uniqueId == "") {
    fullId =  String(HASS_UNIQUE_ID);
  } else {
    fullId = String(HASS_UNIQUE_ID) + "_" + uniqueId;
  }

  doc["name"] = fullName;
  doc["unique_id"] = fullId;
  doc["state_topic"] = MQTT_STATE_TOPIC;
  doc["availability_topic"] = MQTT_AVAILABILITY_TOPIC;

  if(valueTemplate != "") {
    doc["value_template"] = "{{ value_json."+ valueTemplate + " }}";
  }

  if(deviceClass != "") {
    doc["device_class"] = deviceClass;
  }

  if(unitOfMeasurement != "") {
    doc["unit_of_measurement"] = unitOfMeasurement;
  }

  JsonObject device = doc.createNestedObject("device");

  if(isVacuum) {
    device["manufacturer"] = HASS_MANUFACTURER;
    device["model"] = HASS_MODEL;
    device["name"] = HASS_NAME;
    device["sw_version"] = HASS_VERSION;

    JsonArray connections = device.createNestedArray("connections");
    JsonArray connection = connections.createNestedArray();
    connection.add("mac");
    connection.add(WiFi.macAddress());

  } else {
    device["via_device"] = HASS_UNIQUE_ID;
  }
  
  JsonArray identifiers = device.createNestedArray("identifiers");
  identifiers.add(HASS_UNIQUE_ID);

  String str;
  serializeJson(doc, str);
  String topic = "homeassistant/" + topicType + "/" + fullId + "/config";
  mqttClient.publish(topic.c_str(), str.c_str(), true);
  debugV("Auto discovery info published to MQTT!");
}

void onMQTTMessage(char* topic, byte* payload, unsigned int length) 
{
  String newTopic = topic;
  payload[length] = '\0';
  String command = String((char *)payload);
  debugV("MQTT received command: %s", command.c_str());
  if (newTopic == MQTT_COMMAND_TOPIC) {
    wakeUp();

    // Official Home Assistant commands
    if (command == "start" || command == "pause" || command == "start_pause" || command == "stop") {
      clean();
    }
    else if (command == "clean_spot") {
      cleanSpot();
    }
    else if (command == "return_to_base") {
      seekDock();
    }
    else if (command == "turn_off") {
      powerOff();
    }
  }
}


void connectMQTT() 
{
  while (!mqttClient.connected()) 
  {
     // Attempt to connect
      if (mqttClient.connect(MQTT_CLIENT_NAME, MQTT_USER, MQTT_PASSWORD, MQTT_AVAILABILITY_TOPIC, 0, true, "offline")) 
      {
        debugV("MQTT connection sucessful, subscribing to command topic: %s", MQTT_COMMAND_TOPIC);
        mqttClient.subscribe(MQTT_COMMAND_TOPIC);
        mqttClient.publish(MQTT_AVAILABILITY_TOPIC, "online", true);
        publishHomeAssistantAutoDiscovery("", "", "", "", "", "vacuum", true);
        publishHomeAssistantAutoDiscovery("state", "State", "state", "", "", "sensor", false);
        publishHomeAssistantAutoDiscovery("battery", "Battery", "battery_level", "battery", "%", "sensor", false);
        publishHomeAssistantAutoDiscovery("voltage", "Voltage", "voltage / 1000", "battery", "V", "sensor", false);
        publishHomeAssistantAutoDiscovery("temperature", "Temperature", "temperature", "temperature", "°C", "sensor", false);
        publishHomeAssistantAutoDiscovery("dirt_level", "Dirt Level", "dirt_level / 255 * 100 | round(0)", "temperature", "%", "sensor", false);
      } 
      else 
      {
        delay(MQTT_RECONNECT_DELAY);
      }
  }
}

void connectWifi() 
{
  WiFi.hostname(WIFI_CLIENT_NAME);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(WIFI_RECONNECT_DELAY);
  }
}

void startOTA()
{
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOST_NAME);
  ArduinoOTA.begin();
}

void startMQTT()
{
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(onMQTTMessage);
  mqttClient.setBufferSize(MQTT_BUFFER_SIZE);
  connectMQTT();
}

void startDebug()
{
  Debug.begin(WIFI_CLIENT_NAME);
  Debug.setResetCmdEnabled(true);
  
}

void setup() {
  Serial.begin(115200);
  Serial.swap();

  connectWifi();
  startDebug();
  startOTA();
  wakeUp();
  startMQTT();
  updateTime();

  timer.setInterval(ROOMBA_WAKEUP_INTERVAL, wakeUp);
  timer.setInterval(MQTT_PUBLISH_INTERVAL, publishSensorsInformation);
}

void loop() {
  // Reconnect if connection has been lost
  if(!mqttClient.connected()) {
    connectMQTT();
  }

  mqttClient.loop();
  timer.run();
  ArduinoOTA.handle();
  Debug.handle();
}
