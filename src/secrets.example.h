// WiFi Settings
const char* WIFI_CLIENT_NAME = "Roomba";
const char* WIFI_SSID = "SSID";
const char* WIFI_PASSWORD = "12345678";
const int WIFI_RECONNECT_DELAY = 1000;

// MQTT Connection Settings
const char* MQTT_HOST = "192.168.1.123";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "";
const char *MQTT_PASSWORD = "YOUR_MQTT_PASSWORD";
const char *MQTT_CLIENT_NAME = "Roomba";

// MQTT Topic Settings
const char *MQTT_DISCOVERY_TOPIC = "homeassistant/vacuum/roomba/config";
const char *MQTT_STATE_TOPIC = "roomba/state";
const char *MQTT_COMMAND_TOPIC = "roomba/command";
const char *MQTT_AVAILABILITY_TOPIC = "roomba/status";
const int MQTT_PUBLISH_INTERVAL = 10000;
const int MQTT_RECONNECT_DELAY = 5000;
const int MQTT_BUFFER_SIZE = 1024;

// Vacuum Settings
const int ROOMBA_WAKEUP_INTERVAL = 180000;
const int MIN_CLEANING_CURRENT = -500;

// OTA Settings
const int OTA_PORT = 8266;
const char* OTA_HOST_NAME = "roomba";

// Home Assistant Settings
const char* HASS_UNIQUE_ID = "roomba";
const char* HASS_NAME = "Roomba";
const char* HASS_MANUFACTURER = "iRobot";
const char* HASS_MODEL = "Roomba 780";
const char* HASS_VERSION = "1.0.0-dev.0";

//NTP Settings
const char* NTP_SERVER = "pool.ntp.org";
const int NTP_UPDATE_INTERVAL = 24 * 3600 * 1000;
const int NTP_TIME_OFFSET = 3600 * 3;


