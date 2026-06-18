/*
 * ESP32 Tank — L298N + 2x JGB37 encoder + ThingsBoard
 *
 * Thư viện cần cài (Arduino Library Manager):
 *   - PubSubClient by Nick O'Leary
 *   - ArduinoJson by Benoit Blanchon (v7)
 *
 * Encoder: ngắt RISING (giống code xe cân bằng) — không cần ESP32Encoder
 * Board: ESP32 Dev Module
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <time.h>

#include "config.h"
#include "motor_driver.h"
#include "odometry.h"
#include "mission.h"

TankMotors gMotors;
Odometry gOdom;
MissionController gMission;

WiFiClient gWifiClient;
PubSubClient gMqtt(gWifiClient);

float gCountsPerCm = DEFAULT_COUNTS_PER_CM;
long gTurn90Counts = DEFAULT_TURN_90_COUNTS;
uint32_t gScan360Ms = DEFAULT_SCAN_360_MS;
int gMapSizeX = MAP_SIZE_CM;
int gMapSizeY = MAP_SIZE_CM;

unsigned long gLastTelemetry = 0;
unsigned long gLastTimeSync = 0;
unsigned long gLastMqttAttempt = 0;
bool gTimeValid = false;
bool gGasAboveThreshold = false;
int gGasThreshold = GAS_ALERT_THRESHOLD;
bool gMqttConfigured = false;

static const unsigned long MQTT_RECONNECT_MS = 5000UL;

// -------------------------------------------------------------------
const char* modeToText(ControlMode m) {
  return (m == MODE_AUTO) ? "auto" : "manual";
}

const char* autoStateToText(AutoState st) {
  switch (st) {
    case AUTO_DISABLED: return "disabled";
    case AUTO_WAITING_TIME: return "waiting_time";
    case AUTO_MOVING_TO_TARGET: return "moving_to_target";
    case AUTO_AT_TARGET: return "at_target";
    case AUTO_RETURNING: return "returning";
    case AUTO_FINISHED: return "finished";
    default: return "unknown";
  }
}

uint32_t nowEpoch() {
  time_t t = time(nullptr);
  if (t > 1700000000) {
    gTimeValid = true;
    return (uint32_t)t;
  }
  return 0;
}

void setBuzzer(bool on) {
  digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

void updateBuzzer(int gasRaw) {
  setBuzzer(gasRaw > gGasThreshold);
}

// -------------------------------------------------------------------
void syncTimeNtp(bool force) {
  unsigned long nowMs = millis();
  if (!force && (nowMs - gLastTimeSync < TIME_SYNC_INTERVAL_MS)) return;
  if (WiFi.status() != WL_CONNECTED) return;

  configTime(7 * 3600, 0, "pool.ntp.org", "time.google.com");

  struct tm tmInfo;
  if (getLocalTime(&tmInfo, 3000)) {
    gTimeValid = true;
    gLastTimeSync = nowMs;
    Serial.printf("[NTP] epoch=%lu\n", (unsigned long)nowEpoch());
  }
}

void saveSharedConfigToFS() {
  File f = LittleFS.open(SHARED_CONFIG_FILE, "w");
  if (!f) return;

  AutoConfig cfg = gMission.autoConfig();
  StaticJsonDocument<192> doc;
  doc["mode"] = modeToText(gMission.mode());
  doc["x_auto"] = cfg.xAuto;
  doc["y_auto"] = cfg.yAuto;
  doc["run_at_epoch"] = cfg.runAtEpoch;
  doc["gas_threshold"] = gGasThreshold;
  serializeJson(doc, f);
  f.close();
}

void loadSharedConfigFromFS() {
  if (!LittleFS.exists(SHARED_CONFIG_FILE)) return;

  File f = LittleFS.open(SHARED_CONFIG_FILE, "r");
  if (!f) return;

  StaticJsonDocument<192> doc;
  if (deserializeJson(doc, f)) {
    f.close();
    return;
  }
  f.close();

  if (doc.containsKey("gas_threshold")) {
    gGasThreshold = doc["gas_threshold"] | gGasThreshold;
  }

  AutoConfig cfg = gMission.autoConfig();
  cfg.xAuto = constrain(doc["x_auto"] | 0, 0, gMapSizeX);
  cfg.yAuto = constrain(doc["y_auto"] | 0, 0, gMapSizeY);
  cfg.runAtEpoch = doc["run_at_epoch"] | 0;
  gMission.setAutoConfig(cfg);

  const char* m = doc["mode"] | "manual";
  gMission.setMode(strcmp(m, "auto") == 0 ? MODE_AUTO : MODE_MANUAL);
  Serial.printf("[FS] mode=%s x_auto=%d y_auto=%d run_at=%lu gas_thr=%d\n",
                m, cfg.xAuto, cfg.yAuto, (unsigned long)cfg.runAtEpoch, gGasThreshold);
}

void clearSharedConfigFS() {
  if (LittleFS.exists(SHARED_CONFIG_FILE)) LittleFS.remove(SHARED_CONFIG_FILE);
}

void requestSharedAttributes() {
  if (!gMqtt.connected()) return;

  StaticJsonDocument<256> doc;
  doc["sharedKeys"] = TB_SHARED_KEYS;
  char buf[256];
  serializeJson(doc, buf);
  gMqtt.publish("v1/devices/me/attributes/request/1", buf);
  Serial.println("[MQTT] request shared attributes");
}

// -------------------------------------------------------------------
void publishArrivedTelemetry(int gasRaw) {
  if (!gMqtt.connected()) return;

  const Pose p = gMission.pose();
  const bool isManual = (gMission.mode() == MODE_MANUAL);

  StaticJsonDocument<192> doc;
  doc["status"] = isManual ? "manual_arrived" : "arrived";
  doc["mode"] = modeToText(gMission.mode());
  doc["x"] = p.x;
  doc["y"] = p.y;
  doc["gas"] = gasRaw;

  char buf[192];
  serializeJson(doc, buf);
  gMqtt.publish("v1/devices/me/telemetry", buf);
  Serial.printf("[MQTT] arrived: status=%s x=%d y=%d gas=%d\n",
                isManual ? "manual_arrived" : "arrived", p.x, p.y, gasRaw);
}

void publishCalibrationAttributes() {
  if (!gMqtt.connected()) return;

  StaticJsonDocument<256> doc;
  doc["counts_per_cm"] = gCountsPerCm;
  doc["turn_90_counts"] = gTurn90Counts;
  doc["scan_360_ms"] = gScan360Ms;
  doc["pwm_duty_left"] = PWM_DUTY_LEFT;
  doc["pwm_duty_right"] = PWM_DUTY_RIGHT;
  doc["map_size_x"] = gMapSizeX;
  doc["map_size_y"] = gMapSizeY;

  char buf[256];
  serializeJson(doc, buf);
  gMqtt.publish("v1/devices/me/attributes", buf);
  Serial.printf("[MQTT] client attributes: counts/cm=%.1f turn90=%ld PWM %d/%d\n",
                gCountsPerCm, gTurn90Counts, PWM_DUTY_LEFT, PWM_DUTY_RIGHT);
}

void publishTelemetry(bool force) {
  if (!gMqtt.connected()) return;
  unsigned long nowMs = millis();
  if (!force && (nowMs - gLastTelemetry < TELEMETRY_INTERVAL_MS)) return;
  gLastTelemetry = nowMs;

  int gasRaw = analogRead(PIN_GAS);
  bool gasAbove = (gasRaw > gGasThreshold);
  gGasAboveThreshold = gasAbove;
  updateBuzzer(gasRaw);

  Pose p = gMission.pose();
  StaticJsonDocument<192> doc;
  doc["gas"] = gasRaw;
  doc["x"] = p.x;
  doc["y"] = p.y;
  doc["mode"] = modeToText(gMission.mode());

  char buf[192];
  serializeJson(doc, buf);
  gMqtt.publish("v1/devices/me/telemetry", buf);
}

// -------------------------------------------------------------------
static bool jsonBool(JsonVariant v, bool defaultVal = false) {
  if (v.isNull()) return defaultVal;
  if (v.is<bool>()) return v.as<bool>();
  if (v.is<int>()) return v.as<int>() != 0;
  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    return s && (strcmp(s, "true") == 0 || strcmp(s, "1") == 0);
  }
  return defaultVal;
}

void applyAttributeJson(JsonVariant data) {
  if (data.isNull()) {
    Serial.println("[ATTR] (null payload)");
    return;
  }

  Serial.println("[ATTR] apply:");
  serializeJson(data, Serial);
  Serial.println();

  if (jsonBool(data["stop"])) {
    gMission.stopAll();
    gMission.cancelAuto();
    return;
  }

  bool configChanged = false;

  if (data.containsKey("gas_threshold")) {
    gGasThreshold = data["gas_threshold"] | gGasThreshold;
    configChanged = true;
    Serial.printf("[ATTR] gas_threshold=%d\n", gGasThreshold);
  }

  AutoConfig cfg = gMission.autoConfig();
  if (data.containsKey("x_auto")) {
    cfg.xAuto = constrain(data["x_auto"].as<int>(), 0, gMapSizeX);
    configChanged = true;
  }
  if (data.containsKey("y_auto")) {
    cfg.yAuto = constrain(data["y_auto"].as<int>(), 0, gMapSizeY);
    configChanged = true;
  }
  if (data.containsKey("run_at_epoch")) {
    cfg.runAtEpoch = data["run_at_epoch"] | 0;
    configChanged = true;
  }
  gMission.setAutoConfig(cfg);

  if (data.containsKey("mode")) {
    const char* m = data["mode"] | "manual";
    gMission.setMode(strcmp(m, "auto") == 0 ? MODE_AUTO : MODE_MANUAL);
    configChanged = true;
    Serial.printf("[ATTR] mode=%s\n", modeToText(gMission.mode()));
  }

  if (configChanged) saveSharedConfigToFS();

  if (gMission.mode() == MODE_MANUAL) {
    if (data.containsKey("x") && data.containsKey("y")) {
      const int targetX = data["x"].as<int>();
      const int targetY = data["y"].as<int>();
      const Pose p = gMission.pose();
      gMission.startManualMoveTo(targetX, targetY);
      Serial.printf("[MANUAL] (%d,%d) -> (%d,%d)\n", p.x, p.y, targetX, targetY);
    }
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  Serial.println();
  Serial.println("======== LENH TU THINGSBOARD ========");
  Serial.printf("Topic : %s\n", topic);
  Serial.printf("Bytes : %u\n", length);
  Serial.print("Raw   : ");
  for (unsigned int i = 0; i < length; i++) {
    const char c = (char)payload[i];
    Serial.print(c >= 32 ? c : '.');
  }
  Serial.println();
  Serial.println("====================================");

  StaticJsonDocument<1024> doc;
  const DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[MQTT] JSON loi: %s\n", err.c_str());
    return;
  }

  JsonVariant data = doc.as<JsonVariant>();
  if (doc.containsKey("shared")) {
    Serial.println("[MQTT] (goi shared attributes)");
    data = doc["shared"];
  }
  if (strncmp(topic, "v1/devices/me/attributes/response/", 35) == 0) {
    Serial.println("[MQTT] (phan hoi shared attributes)");
    if (doc.containsKey("shared")) data = doc["shared"];
  }
  applyAttributeJson(data);
}

static bool gMqttSubscribed = false;

void mqttOnConnected() {
  if (gMqttSubscribed) return;

  if (!gMqtt.subscribe("v1/devices/me/attributes")) {
    Serial.println("[MQTT] FAIL subscribe v1/devices/me/attributes");
    return;
  }
  if (!gMqtt.subscribe("v1/devices/me/attributes/response/+")) {
    Serial.println("[MQTT] FAIL subscribe attributes/response/+");
    return;
  }

  gMqttSubscribed = true;
  Serial.println("[MQTT] OK subscribe v1/devices/me/attributes");
  Serial.println("[MQTT] Cho lenh Shared attributes tu ThingsBoard...");
  publishCalibrationAttributes();
  requestSharedAttributes();
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.printf("[WiFi] Connecting %s ", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(WiFi.localIP());
    syncTimeNtp(true);
  } else {
    Serial.println(" failed");
  }
}

void setupMqttClient() {
  if (gMqttConfigured) return;
  gMqtt.setServer(TB_SERVER, TB_PORT);
  gMqtt.setCallback(onMqttMessage);
  gMqtt.setKeepAlive(60);
  gMqtt.setSocketTimeout(15);
  gMqtt.setBufferSize(2048);
  gMqttConfigured = true;
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (gMqtt.connected()) return;

  const unsigned long nowMs = millis();
  if (nowMs - gLastMqttAttempt < MQTT_RECONNECT_MS) return;
  gLastMqttAttempt = nowMs;

  setupMqttClient();

  char clientId[28];
  snprintf(clientId, sizeof(clientId), "ESP32_%06X", (unsigned)(ESP.getEfuseMac() & 0xFFFFFFUL));

  Serial.printf("[MQTT] Connecting %s:%d id=%s ...\n", TB_SERVER, TB_PORT, clientId);
  if (gMqtt.connect(clientId, TB_TOKEN, nullptr)) {
    Serial.println("[MQTT] Connected");
    mqttOnConnected();
    gLastTelemetry = 0;
  } else {
    Serial.printf("[MQTT] connect fail rc=%d\n", gMqtt.state());
  }
}

// -------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Tank (Arduino IDE) ===");

  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);
  pinMode(PIN_GAS, INPUT);

  gMotors.begin();
  gOdom.begin();
  gMission.begin(&gMotors, &gOdom);
  gMission.setCalibration(gCountsPerCm, gTurn90Counts, gScan360Ms);
  gMission.setMapSize(gMapSizeX, gMapSizeY);
  gMission.stopAll();

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS mount failed");
  } else {
    loadSharedConfigFromFS();
  }

  connectWiFi();
  setupMqttClient();
  connectMQTT();
  Serial.printf("[CAL] PWM L/R=%d/%d counts/cm=%.1f turnL=%d turnR=%d turn90=%ld\n",
                PWM_DUTY_LEFT, PWM_DUTY_RIGHT, gCountsPerCm,
                TURN_PWM_LEFT, TURN_PWM_RIGHT, gTurn90Counts);
  Serial.println("=== Ready ===");
}

void loop() {
  const unsigned long nowMs = millis();
  const int gasRaw = analogRead(PIN_GAS);

  if (WiFi.status() != WL_CONNECTED) connectWiFi();

  gMqtt.loop();

  static bool wasMqttConnected = false;
  const bool mqttUp = gMqtt.connected();
  if (wasMqttConnected && !mqttUp) {
    Serial.printf("[MQTT] disconnected state=%d\n", gMqtt.state());
    gMqttSubscribed = false;
  }
  wasMqttConnected = mqttUp;

  if (!mqttUp) connectMQTT();

  syncTimeNtp(false);

  gMission.processManualEncMove(nowMs);
  gMission.processAuto(nowMs, gTimeValid, nowEpoch());

  static AutoState lastAutoState = AUTO_DISABLED;
  const AutoState st = gMission.autoState();
  if (st == AUTO_AT_TARGET && lastAutoState == AUTO_MOVING_TO_TARGET) {
    publishArrivedTelemetry(gasRaw);
  }
  if (st == AUTO_WAITING_TIME && lastAutoState == AUTO_RETURNING) {
    Serial.println("[AUTO] ve (0,0) xong");
  }
  lastAutoState = st;

  static bool lastManualEncMove = false;
  const bool manualMove = gMission.manualEncMoveActive();
  if (lastManualEncMove && !manualMove && gMission.mode() == MODE_MANUAL) {
    publishArrivedTelemetry(gasRaw);
    Serial.printf("[MANUAL] arrived (%d,%d)\n", gMission.pose().x, gMission.pose().y);
  }
  lastManualEncMove = manualMove;

  updateBuzzer(gasRaw);

  publishTelemetry(false);
}
