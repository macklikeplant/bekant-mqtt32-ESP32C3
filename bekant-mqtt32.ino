#include "lin.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <EEPROM.h>

// ============================================================
// ESP32-C3 Super Mini pin assignments
// Available GPIOs: 0,1,2,3,4,5,6,7,8,9,10,20,21
// GPIO 11-17 are used for flash, 18-19 are USB D-/D+
// ============================================================
#define UP_BTN   2
#define DOWN_BTN 3
#define MEM1_BTN 4
#define MEM2_BTN 5
#define BTN_DEBOUNCE_MS 25

// No built-in LED on C3 Super Mini - all LED calls are no-ops via macro
#define LED -1
#define LED_ON HIGH
#define LED_WRITE(val) do { if (LED >= 0) digitalWrite(LED, val); } while(0)

// LIN serial: use UART1 on GPIO 20 (RX) and GPIO 21 (TX)
#define TX_PIN 21
#define RX_PIN 20
HardwareSerial SerialLIN(1);

Lin lin(SerialLIN, TX_PIN, RX_PIN);

// WiFi and MQTT settings - fill these in
const char* ssid           = "YOUR_WIFI_SSID";
const char* password       = "YOUR_WIFI_PASSWORD";
const char* mqtt_server    = "YOUR_MQTT_SERVER_IP";
const int   mqtt_port      = 1883;
const char* mqtt_user      = "";          // leave blank if no auth
const char* mqtt_pass      = "";          // leave blank if no auth
const char* mqtt_client_id = "bekant_desk";

// MQTT topics
const char* topic_height              = "bekant/height";
const char* topic_height_cm           = "bekant/height_cm";
const char* topic_height_percent      = "bekant/height_percent";
const char* topic_status              = "bekant/status";
const char* topic_command             = "bekant/command";
const char* topic_log                 = "bekant/log";
const char* topic_availability        = "bekant/availability";
const char* topic_wifi_rssi           = "bekant/wifi_rssi";
const char* topic_min_height_cm       = "bekant/min_height_cm";
const char* topic_max_height_cm       = "bekant/max_height_cm";
const char* topic_set_min_height      = "bekant/set_min_height";
const char* topic_set_max_height      = "bekant/set_max_height";
const char* topic_child_lock          = "bekant/child_lock";
const char* topic_set_child_lock      = "bekant/set_child_lock";
const char* topic_memory1_height_cm   = "bekant/memory1_height_cm";
const char* topic_memory2_height_cm   = "bekant/memory2_height_cm";
const char* topic_memory3_height_cm   = "bekant/memory3_height_cm";
const char* topic_memory4_height_cm   = "bekant/memory4_height_cm";
const char* topic_set_memory1_height  = "bekant/set_memory1_height";
const char* topic_set_memory2_height  = "bekant/set_memory2_height";
const char* topic_set_memory3_height  = "bekant/set_memory3_height";
const char* topic_set_memory4_height  = "bekant/set_memory4_height";
const char* topic_memory1_recall      = "bekant/memory1_recall";
const char* topic_memory2_recall      = "bekant/memory2_recall";
const char* topic_memory3_recall      = "bekant/memory3_recall";
const char* topic_memory4_recall      = "bekant/memory4_recall";
const char* topic_restart             = "bekant/restart";

// Home Assistant MQTT Discovery
const char* ha_discovery_prefix = "homeassistant";
const char* device_id           = "bekant_desk";
const char* device_name         = "Bekant Desk";

// LIN command definitions
#define LIN_CMD_IDLE            252
#define LIN_CMD_RAISE           134
#define LIN_CMD_LOWER           133
#define LIN_CMD_FINE            135
#define LIN_CMD_FINISH          132
#define LIN_CMD_PREMOVE         196
#define LIN_CMD_RECALIBRATE     189
#define LIN_CMD_RECALIBRATE_END 188

#define HYSTERESIS    137
#define MOVE_OFFSET   159

#define DANGER_MAX_HEIGHT (6777 - HYSTERESIS)
#define DANGER_MIN_HEIGHT (162  + HYSTERESIS)

#define LIN_COMM_FAILURE_THRESHOLD 20

// Height conversion
#define CM_TO_ENCODER_UNITS 105.7
#define ENCODER_TO_CM(units) (((units) - DANGER_MIN_HEIGHT) / CM_TO_ENCODER_UNITS + (float)minHeightCm)
#define CM_TO_ENCODER(cm)    ((uint16_t)(((cm) - (float)minHeightCm) * CM_TO_ENCODER_UNITS + DANGER_MIN_HEIGHT))

// EEPROM addresses
#define EEPROM_MIN_HEIGHT_SLOT  0
#define EEPROM_MAX_HEIGHT_SLOT  2
#define EEPROM_CHILD_LOCK_SLOT  6
#define EEPROM_MEMORY1_SLOT     8
#define EEPROM_MEMORY2_SLOT    10
#define EEPROM_MEMORY3_SLOT    12
#define EEPROM_MEMORY4_SLOT    14

WiFiClient   espClient;
PubSubClient client(espClient);

// FreeRTOS queues
QueueHandle_t mqttLogQueue;
#define MAX_LOG_MESSAGE_LENGTH 128
#define MQTT_LOG_QUEUE_SIZE    20

QueueHandle_t buttonCommandQueue;
#define BUTTON_COMMAND_QUEUE_SIZE 10

enum class ButtonCommand {
  UP_PRESS,
  UP_RELEASE,
  DOWN_PRESS,
  DOWN_RELEASE,
  MEM1_SHORT_PRESS,
  MEM1_LONG_PRESS,
  MEM2_SHORT_PRESS,
  MEM2_LONG_PRESS,
  BOTH_PRESS,
  BOTH_RELEASE
};

volatile bool button_up_pressed   = false;
volatile bool button_down_pressed = false;
volatile bool button_mem1_pressed = false;
volatile bool button_mem2_pressed = false;

uint16_t minHeightCm    = 65;
uint16_t maxHeightCm    = 120;
bool     childLockEnabled = false;
uint16_t memory1Height  = DANGER_MIN_HEIGHT;
uint16_t memory2Height  = DANGER_MAX_HEIGHT;
uint16_t memory3Height  = DANGER_MIN_HEIGHT;
uint16_t memory4Height  = DANGER_MAX_HEIGHT;

enum class State {
  OFF, STARTING, UP, DOWN,
  STOPPING1, STOPPING2, STOPPING3, STOPPING4,
  STARTING_RECAL, RECAL, END_RECAL,
};

State lastState    = State::OFF;
int   height       = 0;
State state        = State::OFF;
State previous_state = State::OFF;
uint16_t targetHeight    = 0;

enum class Command { NONE, UP, DOWN, RESET };
Command user_cmd         = Command::NONE;
bool    mqtt_command_active = false;

unsigned long reset_press_start   = 0;
bool          both_buttons_pressed = false;
const unsigned long RESET_DURATION           = 10000;
const unsigned long MEMORY_LONG_PRESS_DURATION = 5000;

unsigned long t = 0;

bool memory1PublishPending = false;
bool memory2PublishPending = false;
bool memory3PublishPending = false;
bool memory4PublishPending = false;

// Forward declarations
void sendInitPacket(byte a1, byte a2, byte a3 = 255, byte a4 = 255);
byte recvInitPacket();
void loadSettingsFromEEPROM();
void saveHeightLimitToEEPROM(uint16_t slot, uint16_t value);
void saveMemoryHeightToEEPROM(uint16_t slot, uint16_t value);
void publishMemoryHeights(bool force = false);
void startMemoryMove(uint16_t memoryHeight, const char* label);
void publishState();
void publishHomeAssistantDiscovery();
void connectWiFi();
void connectMQTT();
void mqttLog(const char* message);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void buttonPollTask(void* parameter);
void wifiMqttTask(void* parameter);
void linInit();
void reset(bool triggered);

// ============================================================
// Timing helper
// ============================================================
void delay_until(unsigned long ms) {
  unsigned long end = t + (1000 * ms);
  unsigned long d   = end - micros();
  if (d > 1000000) { t = micros(); return; }
  if (d > 15000) { unsigned long d2 = (d - 15000) / 1000; delay(d2); d = end - micros(); }
  delayMicroseconds(d);
  t = end;
}

// ============================================================
// LIN helpers
// ============================================================
void sendInitPacket(byte a1, byte a2, byte a3, byte a4) {
  static byte packet[8] = {0, 0, 0, 0, 255, 255, 255, 255};
  packet[0] = a1; packet[1] = a2; packet[2] = a3; packet[3] = a4;
  delay_until(10);
  lin.send(60, packet, 8);
}

byte recvInitPacket() {
  static byte resp[8];
  delay_until(10);
  return lin.recv(61, resp, 8);
}

void linInit() {
  static const byte magicPacket[3] = {246, 255, 191};
  delay(250);
  uint8_t initA = 0;
  t = micros();

  sendInitPacket(255, 7); recvInitPacket();
  sendInitPacket(255, 7); recvInitPacket();
  sendInitPacket(255, 1, 7); recvInitPacket();
  sendInitPacket(208, 2, 7); recvInitPacket();

  bool motorAFound = false;
  while (initA < 8) {
    sendInitPacket(initA, 2, 7);
    if (recvInitPacket() > 0) { motorAFound = true; break; }
    initA++;
  }
  if (!motorAFound) {
    mqttLog("Init: ERROR - Motor A not found! Restarting...");
    delay(1000); ESP.restart(); return;
  }

  sendInitPacket(initA, 6, 9,  0); recvInitPacket();
  sendInitPacket(initA, 6, 12, 0); recvInitPacket();
  sendInitPacket(initA, 6, 13, 0); recvInitPacket();
  sendInitPacket(initA, 6, 10, 0); recvInitPacket();
  sendInitPacket(initA, 6, 11, 0); recvInitPacket();
  sendInitPacket(initA, 4, 0,  0); recvInitPacket();

  byte initB = initA + 1;
  bool motorBFound = false;
  while (initB < 8) {
    sendInitPacket(initB, 2, 0, 0);
    if (recvInitPacket() > 0) { motorBFound = true; break; }
    initB++;
  }
  if (!motorBFound) {
    mqttLog("Init: ERROR - Motor B not found! Restarting...");
    delay(1000); ESP.restart(); return;
  }

  sendInitPacket(initB, 6, 9,  0); recvInitPacket();
  sendInitPacket(initB, 6, 12, 0); recvInitPacket();
  sendInitPacket(initB, 6, 13, 0); recvInitPacket();
  sendInitPacket(initB, 6, 10, 0); recvInitPacket();
  sendInitPacket(initB, 6, 11, 0); recvInitPacket();
  sendInitPacket(initB, 4, 1,  0); recvInitPacket();

  uint8_t initC = initB + 1;
  while (initC < 8) {
    sendInitPacket(initC, 2, 1, 0); recvInitPacket(); initC++;
  }

  sendInitPacket(208, 1, 7, 0); recvInitPacket();
  sendInitPacket(208, 2, 7, 0); recvInitPacket();
  delay_until(15);
  lin.send(18, magicPacket, 3);
  delay(5);
}

// ============================================================
// LED / command helpers (LED_WRITE is a no-op when LED == -1)
// ============================================================
void up(bool pushed) {
  LED_WRITE(pushed ? LED_ON : !LED_ON);
  user_cmd = pushed ? Command::UP : Command::NONE;
}

void down(bool pushed) {
  LED_WRITE(pushed ? LED_ON : !LED_ON);
  user_cmd = pushed ? Command::DOWN : Command::NONE;
}

void reset(bool triggered) {
  LED_WRITE(triggered ? LED_ON : !LED_ON);
  user_cmd = triggered ? Command::RESET : Command::NONE;
}

// ============================================================
// MQTT logging (non-blocking, queue-based)
// ============================================================
void mqttLog(const char* message) {
  char* msg = (char*)malloc(MAX_LOG_MESSAGE_LENGTH);
  if (msg != NULL) {
    strncpy(msg, message, MAX_LOG_MESSAGE_LENGTH - 1);
    msg[MAX_LOG_MESSAGE_LENGTH - 1] = '\0';
    xQueueSend(mqttLogQueue, &msg, 0);
  }
}

// ============================================================
// MQTT callback
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
  String topicStr(topic);

  if (topicStr == topic_set_min_height) {
    int newMin = message.toInt();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newMin >= dangerMinCm && newMin <= dangerMaxCm && newMin < maxHeightCm) {
      minHeightCm = newMin;
      saveHeightLimitToEEPROM(EEPROM_MIN_HEIGHT_SLOT, minHeightCm);
      client.publish(topic_min_height_cm, String(minHeightCm).c_str(), true);
    } else {
      mqttLog(("Invalid min height: " + String(newMin) + "cm").c_str());
    }
  } else if (topicStr == topic_set_max_height) {
    int newMax = message.toInt();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newMax >= dangerMinCm && newMax <= dangerMaxCm && newMax > minHeightCm) {
      maxHeightCm = newMax;
      saveHeightLimitToEEPROM(EEPROM_MAX_HEIGHT_SLOT, maxHeightCm);
      client.publish(topic_max_height_cm, String(maxHeightCm).c_str(), true);
    } else {
      mqttLog(("Invalid max height: " + String(newMax) + "cm").c_str());
    }
  } else if (topicStr == topic_set_child_lock) {
    if (message == "ON" || message == "on" || message == "1" || message == "true") {
      childLockEnabled = true;
      EEPROM.writeBool(EEPROM_CHILD_LOCK_SLOT, childLockEnabled);
      EEPROM.commit();
      client.publish(topic_child_lock, "ON", true);
      if (state == State::STARTING || state == State::UP || state == State::DOWN ||
          state == State::STARTING_RECAL || state == State::RECAL) {
        user_cmd = Command::NONE;
        mqtt_command_active = false;
        if      (state == State::UP   || lastState == State::UP)   targetHeight = height + MOVE_OFFSET;
        else if (state == State::DOWN || lastState == State::DOWN) targetHeight = height - HYSTERESIS;
        else                                                        targetHeight = height;
      }
    } else if (message == "OFF" || message == "off" || message == "0" || message == "false") {
      childLockEnabled = false;
      EEPROM.writeBool(EEPROM_CHILD_LOCK_SLOT, childLockEnabled);
      EEPROM.commit();
      client.publish(topic_child_lock, "OFF", true);
    }
  } else if (topicStr == topic_set_memory1_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      memory1Height = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      saveMemoryHeightToEEPROM(EEPROM_MEMORY1_SLOT, memory1Height);
      publishMemoryHeights(true);
    }
  } else if (topicStr == topic_set_memory2_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      memory2Height = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      saveMemoryHeightToEEPROM(EEPROM_MEMORY2_SLOT, memory2Height);
      publishMemoryHeights(true);
    }
  } else if (topicStr == topic_set_memory3_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      memory3Height = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      saveMemoryHeightToEEPROM(EEPROM_MEMORY3_SLOT, memory3Height);
      publishMemoryHeights(true);
    }
  } else if (topicStr == topic_set_memory4_height) {
    float newHeight = message.toFloat();
    float dangerMinCm = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
    float dangerMaxCm = ENCODER_TO_CM(DANGER_MAX_HEIGHT);
    if (newHeight >= dangerMinCm && newHeight <= dangerMaxCm) {
      memory4Height = constrain(CM_TO_ENCODER(newHeight), (uint16_t)DANGER_MIN_HEIGHT, (uint16_t)DANGER_MAX_HEIGHT);
      saveMemoryHeightToEEPROM(EEPROM_MEMORY4_SLOT, memory4Height);
      publishMemoryHeights(true);
    }
  } else if (topicStr == topic_memory1_recall) {
    mqttLog("MEM1 recall via MQTT"); startMemoryMove(memory1Height, "MEM1");
  } else if (topicStr == topic_memory2_recall) {
    mqttLog("MEM2 recall via MQTT"); startMemoryMove(memory2Height, "MEM2");
  } else if (topicStr == topic_memory3_recall) {
    mqttLog("MEM3 recall via MQTT"); startMemoryMove(memory3Height, "MEM3");
  } else if (topicStr == topic_memory4_recall) {
    mqttLog("MEM4 recall via MQTT"); startMemoryMove(memory4Height, "MEM4");
  } else if (topicStr == topic_restart) {
    if (message == "RESTART" || message == "restart" || message == "1" || message == "true") {
      mqttLog("Restarting ESP32..."); delay(100); ESP.restart();
    }
  } else if (topicStr == topic_command) {
    if (message == "up") {
      user_cmd = Command::UP; mqtt_command_active = true;
      LED_WRITE(LED_ON); mqttLog("MQTT: UP");
    } else if (message == "down") {
      user_cmd = Command::DOWN; mqtt_command_active = true;
      LED_WRITE(LED_ON); mqttLog("MQTT: DOWN");
    } else if (message == "reset") {
      user_cmd = Command::RESET; mqtt_command_active = true;
      LED_WRITE(LED_ON); mqttLog("MQTT: RESET");
    } else if (message == "stop") {
      if (targetHeight > 0 && mqtt_command_active) {
        targetHeight = (targetHeight > height) ? height + MOVE_OFFSET : height - HYSTERESIS;
      } else { targetHeight = 0; }
      user_cmd = Command::NONE; mqtt_command_active = false; LED_WRITE(!LED_ON);
      mqttLog("MQTT: STOP");
    } else if (message.startsWith("height-cm ")) {
      int parsedCm = message.substring(10).toInt();
      if (parsedCm >= minHeightCm && parsedCm <= maxHeightCm) {
        targetHeight = CM_TO_ENCODER(parsedCm); mqtt_command_active = true; LED_WRITE(LED_ON);
        mqttLog(("MQTT: Moving to " + String(parsedCm) + "cm").c_str());
      }
    } else if (message.startsWith("height-percent ")) {
      int percent = message.substring(15).toInt();
      if (percent >= 0 && percent <= 100) {
        int actualCm = minHeightCm + ((percent * (maxHeightCm - minHeightCm)) / 100);
        targetHeight = CM_TO_ENCODER(actualCm); mqtt_command_active = true; LED_WRITE(LED_ON);
        mqttLog(("MQTT: Moving to " + String(percent) + "%").c_str());
      }
    } else if (message.startsWith("height ")) {
      int parsedHeight = message.substring(7).toInt();
      if (parsedHeight >= DANGER_MIN_HEIGHT && parsedHeight <= DANGER_MAX_HEIGHT) {
        targetHeight = parsedHeight; mqtt_command_active = true; LED_WRITE(LED_ON);
      }
    }
  }
}

// ============================================================
// WiFi / MQTT connection
// ============================================================
void connectWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void connectMQTT() {
  while (!client.connected()) {
    bool ok;
    if (strlen(mqtt_user) > 0) {
      ok = client.connect(mqtt_client_id, mqtt_user, mqtt_pass,
                          topic_availability, 1, true, "offline");
    } else {
      ok = client.connect(mqtt_client_id, topic_availability, 1, true, "offline");
    }
    if (ok) {
      client.subscribe(topic_command);
      client.subscribe(topic_set_min_height);
      client.subscribe(topic_set_max_height);
      client.subscribe(topic_set_child_lock);
      client.subscribe(topic_set_memory1_height);
      client.subscribe(topic_set_memory2_height);
      client.subscribe(topic_set_memory3_height);
      client.subscribe(topic_set_memory4_height);
      client.subscribe(topic_memory1_recall);
      client.subscribe(topic_memory2_recall);
      client.subscribe(topic_memory3_recall);
      client.subscribe(topic_memory4_recall);
      client.subscribe(topic_restart);
    } else {
      delay(1000);
    }
  }
}

// ============================================================
// HA Discovery
// ============================================================
void publishHomeAssistantDiscovery() {
  if (!client.connected()) return;

  String dev  = "\"device\":{\"identifiers\":[\"" + String(device_id) + "\"],"
                "\"name\":\"" + String(device_name) + "\","
                "\"manufacturer\":\"IKEA/Custom\","
                "\"model\":\"Bekant ESP32-C3\","
                "\"sw_version\":\"1.1\"}";
  String avail = "\"availability\":[{\"topic\":\"" + String(topic_availability) + "\"}]";

  // Cover
  String cover = "{\"name\":\"Bekant Desk\","
    "\"unique_id\":\"" + String(device_id) + "_cover\","
    "\"command_topic\":\"" + String(topic_command) + "\","
    "\"position_topic\":\"" + String(topic_height_percent) + "\","
    "\"set_position_topic\":\"" + String(topic_command) + "\","
    "\"state_topic\":\"" + String(topic_status) + "\","
    "\"payload_open\":\"up\",\"payload_close\":\"down\",\"payload_stop\":\"stop\","
    "\"position_open\":100,\"position_closed\":0,"
    "\"state_open\":\"open\",\"state_closed\":\"closed\","
    "\"state_opening\":\"opening\",\"state_closing\":\"closing\","
    "\"state_stopped\":\"stopped\","
    "\"position_template\":\"{{ value | int }}\","
    "\"set_position_template\":\"height-percent {{ position | int }}\","
    "\"device_class\":\"blind\",\"icon\":\"mdi:desk\","
    + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/cover/" + device_id + "/config").c_str(), cover.c_str(), true);

  // Height sensor
  String ht = "{\"name\":\"Height\","
    "\"unique_id\":\"" + String(device_id) + "_height_cm\","
    "\"state_topic\":\"" + String(topic_height_cm) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"state_class\":\"measurement\","
    "\"icon\":\"mdi:ruler\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/sensor/" + device_id + "_height/config").c_str(), ht.c_str(), true);

  // Status sensor
  String st = "{\"name\":\"Status\","
    "\"unique_id\":\"" + String(device_id) + "_status\","
    "\"state_topic\":\"" + String(topic_status) + "\","
    "\"icon\":\"mdi:information-outline\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/sensor/" + device_id + "_status/config").c_str(), st.c_str(), true);

  // WiFi RSSI
  String rssi = "{\"name\":\"WiFi Signal\","
    "\"unique_id\":\"" + String(device_id) + "_rssi\","
    "\"state_topic\":\"" + String(topic_wifi_rssi) + "\","
    "\"unit_of_measurement\":\"dBm\","
    "\"device_class\":\"signal_strength\","
    "\"state_class\":\"measurement\","
    "\"entity_category\":\"diagnostic\","
    "\"enabled_by_default\":false,"
    "\"icon\":\"mdi:wifi\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/sensor/" + device_id + "_rssi/config").c_str(), rssi.c_str(), true);

  // Child lock switch
  String cl = "{\"name\":\"Child Lock\","
    "\"unique_id\":\"" + String(device_id) + "_child_lock\","
    "\"state_topic\":\"" + String(topic_child_lock) + "\","
    "\"command_topic\":\"" + String(topic_set_child_lock) + "\","
    "\"payload_on\":\"ON\",\"payload_off\":\"OFF\","
    "\"state_on\":\"ON\",\"state_off\":\"OFF\","
    "\"icon\":\"mdi:lock\","
    "\"entity_category\":\"config\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/switch/" + device_id + "_child_lock/config").c_str(), cl.c_str(), true);

  // Recalibrate button
  String recal = "{\"name\":\"Recalibrate\","
    "\"unique_id\":\"" + String(device_id) + "_recalibrate\","
    "\"command_topic\":\"" + String(topic_command) + "\","
    "\"payload_press\":\"reset\","
    "\"icon\":\"mdi:refresh\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/button/" + device_id + "_recalibrate/config").c_str(), recal.c_str(), true);

  // Restart button
  String restart_cfg = "{\"name\":\"Restart\","
    "\"unique_id\":\"" + String(device_id) + "_restart\","
    "\"command_topic\":\"" + String(topic_restart) + "\","
    "\"payload_press\":\"RESTART\","
    "\"icon\":\"mdi:restart\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/button/" + device_id + "_restart/config").c_str(), restart_cfg.c_str(), true);

  // Memory recall buttons
  const char* memLabels[]  = {"Memory 1 Recall","Memory 2 Recall","Memory 3 Recall","Memory 4 Recall"};
  const char* memTopics[]  = {topic_memory1_recall, topic_memory2_recall, topic_memory3_recall, topic_memory4_recall};
  const char* memIcons[]   = {"mdi:numeric-1-circle","mdi:numeric-2-circle","mdi:numeric-3-circle","mdi:numeric-4-circle"};
  const char* memIds[]     = {"_memory1_recall","_memory2_recall","_memory3_recall","_memory4_recall"};
  for (int i = 0; i < 4; i++) {
    String m = "{\"name\":\"" + String(memLabels[i]) + "\","
      "\"unique_id\":\"" + String(device_id) + String(memIds[i]) + "\","
      "\"command_topic\":\"" + String(memTopics[i]) + "\","
      "\"payload_press\":\"PRESS\","
      "\"icon\":\"" + String(memIcons[i]) + "\"," + avail + "," + dev + "}";
    client.publish((String(ha_discovery_prefix) + "/button/" + device_id + String(memIds[i]) + "/config").c_str(), m.c_str(), true);
  }

  // Min/Max height number inputs
  String minH = "{\"name\":\"Min Height\","
    "\"unique_id\":\"" + String(device_id) + "_min_height\","
    "\"state_topic\":\"" + String(topic_min_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_min_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,\"max\":120,\"step\":1,\"mode\":\"box\","
    "\"icon\":\"mdi:arrow-collapse-down\","
    "\"entity_category\":\"config\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/number/" + device_id + "_min_height/config").c_str(), minH.c_str(), true);

  String maxH = "{\"name\":\"Max Height\","
    "\"unique_id\":\"" + String(device_id) + "_max_height\","
    "\"state_topic\":\"" + String(topic_max_height_cm) + "\","
    "\"command_topic\":\"" + String(topic_set_max_height) + "\","
    "\"unit_of_measurement\":\"cm\","
    "\"min\":65,\"max\":120,\"step\":1,\"mode\":\"box\","
    "\"icon\":\"mdi:arrow-collapse-up\","
    "\"entity_category\":\"config\"," + avail + "," + dev + "}";
  client.publish((String(ha_discovery_prefix) + "/number/" + device_id + "_max_height/config").c_str(), maxH.c_str(), true);

  // Memory height number inputs
  const char* mhLabels[] = {"Memory 1 Height","Memory 2 Height","Memory 3 Height","Memory 4 Height"};
  const char* mhState[]  = {topic_memory1_height_cm, topic_memory2_height_cm, topic_memory3_height_cm, topic_memory4_height_cm};
  const char* mhCmd[]    = {topic_set_memory1_height, topic_set_memory2_height, topic_set_memory3_height, topic_set_memory4_height};
  const char* mhIcons[]  = {"mdi:numeric-1-box","mdi:numeric-2-box","mdi:numeric-3-box","mdi:numeric-4-box"};
  const char* mhIds[]    = {"_memory1_height","_memory2_height","_memory3_height","_memory4_height"};
  for (int i = 0; i < 4; i++) {
    String m = "{\"name\":\"" + String(mhLabels[i]) + "\","
      "\"unique_id\":\"" + String(device_id) + String(mhIds[i]) + "\","
      "\"state_topic\":\"" + String(mhState[i]) + "\","
      "\"command_topic\":\"" + String(mhCmd[i]) + "\","
      "\"unit_of_measurement\":\"cm\","
      "\"min\":65,\"max\":120,\"step\":0.1,\"mode\":\"box\","
      "\"icon\":\"" + String(mhIcons[i]) + "\","
      "\"entity_category\":\"config\"," + avail + "," + dev + "}";
    client.publish((String(ha_discovery_prefix) + "/number/" + device_id + String(mhIds[i]) + "/config").c_str(), m.c_str(), true);
  }

  mqttLog("HA discovery published");
}

// ============================================================
// State publishing
// ============================================================
void publishState() {
  if (!client.connected()) return;
  client.publish(topic_height, String(height).c_str());
  float heightCm = ENCODER_TO_CM(height);
  client.publish(topic_height_cm, String(heightCm, 1).c_str());
  int pct = (int)(((heightCm - minHeightCm) / (maxHeightCm - minHeightCm)) * 100.0 + 0.5);
  pct = constrain(pct, 0, 100);
  client.publish(topic_height_percent, String(pct).c_str());

  String s;
  switch (state) {
    case State::UP: case State::STARTING:              s = "opening"; break;
    case State::DOWN: case State::STARTING_RECAL:
    case State::RECAL: case State::END_RECAL:          s = "closing"; break;
    default:                                            s = "stopped"; break;
  }
  client.publish(topic_status, s.c_str());
}

void publishMemoryHeights(bool force) {
  if (!client.connected()) return;
  if (force || memory1PublishPending) {
    client.publish(topic_memory1_height_cm, String(ENCODER_TO_CM(memory1Height), 1).c_str(), true);
    memory1PublishPending = false;
  }
  if (force || memory2PublishPending) {
    client.publish(topic_memory2_height_cm, String(ENCODER_TO_CM(memory2Height), 1).c_str(), true);
    memory2PublishPending = false;
  }
  if (force || memory3PublishPending) {
    client.publish(topic_memory3_height_cm, String(ENCODER_TO_CM(memory3Height), 1).c_str(), true);
    memory3PublishPending = false;
  }
  if (force || memory4PublishPending) {
    client.publish(topic_memory4_height_cm, String(ENCODER_TO_CM(memory4Height), 1).c_str(), true);
    memory4PublishPending = false;
  }
}

// ============================================================
// Button poll task (Core 0)
// ============================================================
void buttonPollTask(void* parameter) {
  bool up_p = false, up_tmp = false, last_up = false;
  bool dn_p = false, dn_tmp = false, last_dn = false;
  bool m1_p = false, m1_tmp = false, last_m1 = false;
  bool m2_p = false, m2_tmp = false, last_m2 = false;
  bool both  = false, last_both = false;
  int up_cnt = 0, up_rcnt = 0;
  int dn_cnt = 0, dn_rcnt = 0;
  int m1_cnt = 0, m1_rcnt = 0;
  int m2_cnt = 0, m2_rcnt = 0;

  unsigned long m1_start = 0, m2_start = 0;
  bool m1_long_sent = false, m2_long_sent = false;
  unsigned long both_start = 0;
  bool both_sent = false;
  const unsigned long LONG_MS = 5000;

  while (true) {
    up_tmp = (digitalRead(UP_BTN)   == LOW);
    dn_tmp = (digitalRead(DOWN_BTN) == LOW);
    m1_tmp = (digitalRead(MEM1_BTN) == LOW);
    m2_tmp = (digitalRead(MEM2_BTN) == LOW);

    // Debounce UP
    if (up_tmp) { if (++up_cnt >= BTN_DEBOUNCE_MS) { up_p = true;  up_cnt = up_rcnt = 0; } }
    else         { if (++up_rcnt >= BTN_DEBOUNCE_MS) { up_p = false; up_cnt = up_rcnt = 0; } }
    // Debounce DOWN
    if (dn_tmp) { if (++dn_cnt >= BTN_DEBOUNCE_MS) { dn_p = true;  dn_cnt = dn_rcnt = 0; } }
    else         { if (++dn_rcnt >= BTN_DEBOUNCE_MS) { dn_p = false; dn_cnt = dn_rcnt = 0; } }
    // Debounce MEM1
    if (m1_tmp) { if (++m1_cnt >= BTN_DEBOUNCE_MS) { m1_p = true;  m1_cnt = m1_rcnt = 0; } }
    else         { if (++m1_rcnt >= BTN_DEBOUNCE_MS) { m1_p = false; m1_cnt = m1_rcnt = 0; } }
    // Debounce MEM2
    if (m2_tmp) { if (++m2_cnt >= BTN_DEBOUNCE_MS) { m2_p = true;  m2_cnt = m2_rcnt = 0; } }
    else         { if (++m2_rcnt >= BTN_DEBOUNCE_MS) { m2_p = false; m2_cnt = m2_rcnt = 0; } }

    button_up_pressed   = up_p;
    button_down_pressed = dn_p;
    button_mem1_pressed = m1_p;
    button_mem2_pressed = m2_p;

    both = (up_p && dn_p);
    ButtonCommand cmd;

    if (both && !last_both)  { both_start = millis(); both_sent = false; }
    if (both && !both_sent && (millis() - both_start >= RESET_DURATION)) {
      cmd = ButtonCommand::BOTH_PRESS; xQueueSend(buttonCommandQueue, &cmd, 0); both_sent = true;
    }
    if (!both && last_both) {
      both_sent = false; cmd = ButtonCommand::BOTH_RELEASE; xQueueSend(buttonCommandQueue, &cmd, 0);
    }

    if (!both && !last_both) {
      if (up_p  && !last_up) { cmd = ButtonCommand::UP_PRESS;   xQueueSend(buttonCommandQueue, &cmd, 0); }
      if (!up_p && last_up)  { cmd = ButtonCommand::UP_RELEASE; xQueueSend(buttonCommandQueue, &cmd, 0); }
      if (dn_p  && !last_dn) { cmd = ButtonCommand::DOWN_PRESS;   xQueueSend(buttonCommandQueue, &cmd, 0); }
      if (!dn_p && last_dn)  { cmd = ButtonCommand::DOWN_RELEASE; xQueueSend(buttonCommandQueue, &cmd, 0); }

      if (m1_p && !last_m1) { m1_start = millis(); m1_long_sent = false; }
      if (m1_p && !m1_long_sent && (millis() - m1_start >= LONG_MS)) {
        cmd = ButtonCommand::MEM1_LONG_PRESS; xQueueSend(buttonCommandQueue, &cmd, 0); m1_long_sent = true;
      }
      if (!m1_p && last_m1 && !m1_long_sent) {
        cmd = ButtonCommand::MEM1_SHORT_PRESS; xQueueSend(buttonCommandQueue, &cmd, 0);
      }

      if (m2_p && !last_m2) { m2_start = millis(); m2_long_sent = false; }
      if (m2_p && !m2_long_sent && (millis() - m2_start >= LONG_MS)) {
        cmd = ButtonCommand::MEM2_LONG_PRESS; xQueueSend(buttonCommandQueue, &cmd, 0); m2_long_sent = true;
      }
      if (!m2_p && last_m2 && !m2_long_sent) {
        cmd = ButtonCommand::MEM2_SHORT_PRESS; xQueueSend(buttonCommandQueue, &cmd, 0);
      }
    }

    last_up = up_p; last_dn = dn_p; last_m1 = m1_p; last_m2 = m2_p; last_both = both;
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ============================================================
// WiFi/MQTT task (Core 0)
// ============================================================
void wifiMqttTask(void* parameter) {
  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  client.setBufferSize(2048);
  connectMQTT();

  client.publish(topic_availability, "online", true);
  delay(500);
  publishHomeAssistantDiscovery();
  client.publish(topic_min_height_cm,  String(minHeightCm).c_str(),  true);
  client.publish(topic_max_height_cm,  String(maxHeightCm).c_str(),  true);
  client.publish(topic_child_lock,     childLockEnabled ? "ON" : "OFF", true);
  publishMemoryHeights(true);
  publishState();

  unsigned long lastPublish     = 0;
  unsigned long lastDiagnostic  = 0;
  State lastPublishedState      = State::OFF;

  while (true) {
    if (!client.connected()) {
      connectMQTT();
      client.publish(topic_availability, "online", true);
      delay(500);
      publishHomeAssistantDiscovery();
      client.publish(topic_min_height_cm,  String(minHeightCm).c_str(),  true);
      client.publish(topic_max_height_cm,  String(maxHeightCm).c_str(),  true);
      client.publish(topic_child_lock,     childLockEnabled ? "ON" : "OFF", true);
      publishMemoryHeights(true);
      publishState();
    }
    client.loop();

    // Drain log queue
    char* logMsg;
    while (xQueueReceive(mqttLogQueue, &logMsg, 0) == pdTRUE) {
      if (client.connected() && logMsg) { client.publish(topic_log, logMsg); }
      free(logMsg);
    }

    unsigned long now = millis();
    if (state != lastPublishedState || now - lastPublish > 30000) {
      publishState(); lastPublishedState = state; lastPublish = now;
    }
    publishMemoryHeights();

    if (now - lastDiagnostic > 30000) {
      if (client.connected()) client.publish(topic_wifi_rssi, String(WiFi.RSSI()).c_str());
      lastDiagnostic = now;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================
// EEPROM helpers
// ============================================================
void loadSettingsFromEEPROM() {
  uint16_t storedMin  = EEPROM.readUShort(EEPROM_MIN_HEIGHT_SLOT);
  uint16_t storedMax  = EEPROM.readUShort(EEPROM_MAX_HEIGHT_SLOT);
  bool     storedCL   = EEPROM.readBool(EEPROM_CHILD_LOCK_SLOT);
  uint16_t s1 = EEPROM.readUShort(EEPROM_MEMORY1_SLOT);
  uint16_t s2 = EEPROM.readUShort(EEPROM_MEMORY2_SLOT);
  uint16_t s3 = EEPROM.readUShort(EEPROM_MEMORY3_SLOT);
  uint16_t s4 = EEPROM.readUShort(EEPROM_MEMORY4_SLOT);

  float dMin = ENCODER_TO_CM(DANGER_MIN_HEIGHT);
  float dMax = ENCODER_TO_CM(DANGER_MAX_HEIGHT);

  if (storedMin >= dMin && storedMin <= dMax && storedMin > 0)              minHeightCm = storedMin;
  if (storedMax >= dMin && storedMax <= dMax && storedMax > minHeightCm)    maxHeightCm = storedMax;
  childLockEnabled = storedCL;

  memory1Height = (s1 >= DANGER_MIN_HEIGHT && s1 <= DANGER_MAX_HEIGHT) ? s1 : DANGER_MIN_HEIGHT;
  memory2Height = (s2 >= DANGER_MIN_HEIGHT && s2 <= DANGER_MAX_HEIGHT) ? s2 : DANGER_MAX_HEIGHT;
  memory3Height = (s3 >= DANGER_MIN_HEIGHT && s3 <= DANGER_MAX_HEIGHT) ? s3 : DANGER_MIN_HEIGHT;
  memory4Height = (s4 >= DANGER_MIN_HEIGHT && s4 <= DANGER_MAX_HEIGHT) ? s4 : DANGER_MAX_HEIGHT;
}

void saveHeightLimitToEEPROM(uint16_t slot, uint16_t value) {
  EEPROM.writeUShort(slot, value); EEPROM.commit();
}

void saveMemoryHeightToEEPROM(uint16_t slot, uint16_t value) {
  EEPROM.writeUShort(slot, value); EEPROM.commit();
}

void startMemoryMove(uint16_t memoryHeight, const char* label) {
  if (memoryHeight < DANGER_MIN_HEIGHT || memoryHeight > DANGER_MAX_HEIGHT) {
    mqttLog((String(label) + " out of range (" + String(memoryHeight) + ")").c_str()); return;
  }
  targetHeight = memoryHeight; mqtt_command_active = true; LED_WRITE(LED_ON);
  mqttLog((String(label) + " -> " + String(ENCODER_TO_CM(memoryHeight), 1) + "cm").c_str());
}

// ============================================================
// setup()
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // SerialLIN must be initialized before Lin::begin() is called
  SerialLIN.begin(19200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(UP_BTN,   INPUT_PULLUP);
  pinMode(DOWN_BTN, INPUT_PULLUP);
  pinMode(MEM1_BTN, INPUT_PULLUP);
  pinMode(MEM2_BTN, INPUT_PULLUP);
  // No LED pin setup needed (LED == -1)

  EEPROM.begin(512);
  loadSettingsFromEEPROM();

  mqttLogQueue      = xQueueCreate(MQTT_LOG_QUEUE_SIZE,      sizeof(char*));
  buttonCommandQueue = xQueueCreate(BUTTON_COMMAND_QUEUE_SIZE, sizeof(ButtonCommand));

  xTaskCreatePinnedToCore(buttonPollTask, "ButtonPoll", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(wifiMqttTask,  "WiFiMQTT",   8192, NULL, 1, NULL, 0);

  mqttLog("Starting LIN initialization...");
  lin.begin(19200);
  linInit();
  mqttLog("LIN initialization complete");
}

// ============================================================
// loop() - runs on Core 1, handles LIN protocol
// ============================================================
void loop() {
  uint8_t empty[3] = {0, 0, 0};
  uint8_t node_a[3] = {0, 0, 0};
  uint8_t node_b[3] = {0, 0, 0};
  uint8_t cmd[3]    = {0, 0, 0};
  uint8_t res = 0;

  lin.send(17, empty, 3);
  delay_until(5);

  res = lin.recv(8, node_a, 3);
  static uint8_t pid8Fail = 0;
  if (res != 4) {
    delay_until(2); res = lin.recv(8, node_a, 3);
    if (res != 4) { if (++pid8Fail >= LIN_COMM_FAILURE_THRESHOLD) { mqttLog("LIN Motor A failed - restarting"); delay(1000); ESP.restart(); } }
    else pid8Fail = 0;
  } else pid8Fail = 0;
  delay_until(5);

  res = lin.recv(9, node_b, 3);
  static uint8_t pid9Fail = 0;
  if (res != 4) {
    delay_until(2); res = lin.recv(9, node_b, 3);
    if (res != 4) { if (++pid9Fail >= LIN_COMM_FAILURE_THRESHOLD) { mqttLog("LIN Motor B failed - restarting"); delay(1000); ESP.restart(); } }
    else pid9Fail = 0;
  } else pid9Fail = 0;
  delay_until(5);

  for (uint8_t i = 0; i < 6; i++) { delay_until(5); lin.send(16, 0, 0); }
  delay_until(5);
  lin.send(1, 0, 0);

  uint16_t enc_a      = node_a[0] | (node_a[1] << 8);
  uint16_t enc_b      = node_b[0] | (node_b[1] << 8);
  uint16_t enc_min    = min(enc_a, enc_b);
  uint16_t enc_max    = max(enc_a, enc_b);
  uint16_t enc_target = (enc_a + enc_b) / 2;
  height = (enc_a + enc_b) / 2;

  switch (state) {
    case State::OFF:           cmd[2] = LIN_CMD_IDLE;           break;
    case State::STARTING:      cmd[2] = LIN_CMD_PREMOVE;        break;
    case State::UP:            enc_target = min(enc_a,enc_b); cmd[2] = LIN_CMD_RAISE;  lastState = State::UP;   break;
    case State::DOWN:          enc_target = max(enc_a,enc_b); cmd[2] = LIN_CMD_LOWER;  lastState = State::DOWN; break;
    case State::STOPPING1:
    case State::STOPPING2:
    case State::STOPPING3:     enc_target = targetHeight; cmd[2] = LIN_CMD_FINE;        break;
    case State::STOPPING4:
      enc_target = (lastState == State::UP) ? min(enc_a,enc_b) : max(enc_a,enc_b);
      cmd[2] = LIN_CMD_FINISH; break;
    case State::STARTING_RECAL: cmd[2] = LIN_CMD_PREMOVE;       break;
    case State::RECAL:          enc_target = 0; cmd[2] = LIN_CMD_RECALIBRATE;     break;
    case State::END_RECAL:      enc_target = 99; cmd[2] = LIN_CMD_RECALIBRATE_END; break;
  }
  cmd[0] = enc_target & 0xFF;
  cmd[1] = enc_target >> 8;
  delay_until(5);
  lin.send(18, cmd, 3);

  // Auto-move to target
  if (targetHeight > 0 && mqtt_command_active) {
    if (abs((int)height - (int)targetHeight) <= HYSTERESIS) {
      user_cmd = Command::NONE; mqtt_command_active = false; LED_WRITE(!LED_ON);
    } else if (targetHeight > height + HYSTERESIS) {
      user_cmd = Command::UP;
    } else if (targetHeight < height - HYSTERESIS) {
      user_cmd = Command::DOWN;
    }
  }

  bool up_pressed   = button_up_pressed;
  bool down_pressed = button_down_pressed;

  // Process button queue
  ButtonCommand buttonCommand;
  while (xQueueReceive(buttonCommandQueue, &buttonCommand, 0) == pdTRUE) {
    if (childLockEnabled) continue;
    switch (buttonCommand) {
      case ButtonCommand::UP_PRESS:
        if (mqtt_command_active) {
          targetHeight = (targetHeight > height) ? height + MOVE_OFFSET : height - HYSTERESIS;
          mqtt_command_active = false;
        }
        break;
      case ButtonCommand::DOWN_PRESS:
        if (mqtt_command_active) {
          targetHeight = (targetHeight > height) ? height + MOVE_OFFSET : height - HYSTERESIS;
          mqtt_command_active = false;
        }
        break;
      case ButtonCommand::MEM1_SHORT_PRESS: startMemoryMove(memory1Height, "MEM1"); break;
      case ButtonCommand::MEM1_LONG_PRESS:
        memory1Height = (uint16_t)constrain((int)height, DANGER_MIN_HEIGHT, DANGER_MAX_HEIGHT);
        saveMemoryHeightToEEPROM(EEPROM_MEMORY1_SLOT, memory1Height);
        memory1PublishPending = true;
        break;
      case ButtonCommand::MEM2_SHORT_PRESS: startMemoryMove(memory2Height, "MEM2"); break;
      case ButtonCommand::MEM2_LONG_PRESS:
        memory2Height = (uint16_t)constrain((int)height, DANGER_MIN_HEIGHT, DANGER_MAX_HEIGHT);
        saveMemoryHeightToEEPROM(EEPROM_MEMORY2_SLOT, memory2Height);
        memory2PublishPending = true;
        break;
      case ButtonCommand::BOTH_PRESS:
        both_buttons_pressed = true; reset_press_start = millis(); break;
      case ButtonCommand::BOTH_RELEASE:
        if (both_buttons_pressed) { both_buttons_pressed = false; reset(false); } break;
      default: break;
    }
  }

  if (!childLockEnabled && !mqtt_command_active) {
    if      (up_pressed && !down_pressed)  { user_cmd = Command::UP;   LED_WRITE(LED_ON);  }
    else if (down_pressed && !up_pressed)  { user_cmd = Command::DOWN; LED_WRITE(LED_ON);  }
    else if (!up_pressed && !down_pressed) { user_cmd = Command::NONE; LED_WRITE(!LED_ON); }
  }

  if (both_buttons_pressed && (millis() - reset_press_start >= RESET_DURATION)) reset(true);

  bool isIdle = (node_a[2] == 0 || node_a[2] == 37 || node_a[2] == 96) &&
                (node_b[2] == 0 || node_b[2] == 37 || node_b[2] == 96);

  if (childLockEnabled && (state == State::STARTING || state == State::UP || state == State::DOWN ||
                           state == State::STARTING_RECAL || state == State::RECAL)) {
    user_cmd = Command::NONE; mqtt_command_active = false;
    if (state != State::STOPPING1 && state != State::STOPPING2 &&
        state != State::STOPPING3 && state != State::STOPPING4) {
      if      (state == State::UP   || lastState == State::UP)   targetHeight = height + MOVE_OFFSET;
      else if (state == State::DOWN || lastState == State::DOWN) targetHeight = height - HYSTERESIS;
      else                                                        targetHeight = height;
      state = State::STOPPING1;
    }
  }

  switch (state) {
    case State::OFF:
      if      (user_cmd == Command::RESET)             { state = State::STARTING_RECAL; mqttLog("Starting recalibration..."); }
      else if (user_cmd != Command::NONE && isIdle)    { state = State::STARTING; }
      break;
    case State::STARTING:
      switch (user_cmd) {
        case Command::NONE:  state = State::OFF; mqtt_command_active = false; targetHeight = 0; break;
        case Command::UP:
          if (height >= DANGER_MAX_HEIGHT) { state = State::OFF; mqtt_command_active = false; targetHeight = 0; }
          else state = State::UP; break;
        case Command::DOWN:
          if (height <= DANGER_MIN_HEIGHT) { state = State::OFF; mqtt_command_active = false; targetHeight = 0; }
          else state = State::DOWN; break;
        case Command::RESET: state = State::STARTING_RECAL; break;
      }
      break;
    case State::UP:
      if (user_cmd != Command::UP || enc_max >= DANGER_MAX_HEIGHT) state = State::STOPPING1; break;
    case State::DOWN:
      if (user_cmd != Command::DOWN || enc_min <= DANGER_MIN_HEIGHT) state = State::STOPPING1; break;
    case State::STOPPING1: state = State::STOPPING2; break;
    case State::STOPPING2: state = State::STOPPING3; break;
    case State::STOPPING3: state = State::STOPPING4; break;
    case State::STOPPING4:
      if (isIdle) { state = State::OFF; mqtt_command_active = false; targetHeight = 0; } break;
    case State::STARTING_RECAL: state = State::RECAL; mqttLog("Recal: driving to bottom..."); break;
    case State::RECAL:
      if (enc_max <= 99 && node_a[2] == 1 && node_b[2] == 1) {
        state = State::END_RECAL; mqttLog("Recal: reached bottom");
      }
      if (user_cmd != Command::RESET) {
        state = State::OFF; mqtt_command_active = false; targetHeight = 0; mqttLog("Recal cancelled");
      }
      break;
    case State::END_RECAL:
      state = State::OFF; targetHeight = enc_max; mqtt_command_active = false; mqttLog("Recal complete!"); break;
    default:
      state = State::OFF; mqtt_command_active = false; targetHeight = 0; break;
  }

  previous_state = state;
  delay_until(50);
}
