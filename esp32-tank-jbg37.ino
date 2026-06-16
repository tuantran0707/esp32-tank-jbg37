/*
 * ESP32 Tank — TB6612FNG + 2x JGB37 encoder + ThingsBoard
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
  setBuzzer(gasRaw > GAS_ALERT_THRESHOLD);
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

void saveScheduleToFS() {
  AutoConfig cfg = gMission.autoConfig();
  File f = LittleFS.open(SCHEDULE_FILE, "w");
  if (!f) return;

  StaticJsonDocument<128> doc;
  doc["enabled"] = cfg.enabled;
  doc["target_x"] = cfg.targetX;
  doc["target_y"] = cfg.targetY;
  doc["run_at_epoch"] = cfg.runAtEpoch;
  serializeJson(doc, f);
  f.close();
}

void loadScheduleFromFS() {
  if (!LittleFS.exists(SCHEDULE_FILE)) return;

  File f = LittleFS.open(SCHEDULE_FILE, "r");
  if (!f) return;

  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, f)) {
    f.close();
    return;
  }
  f.close();

  AutoConfig cfg;
  cfg.enabled = doc["enabled"] | false;
  cfg.targetX = constrain(doc["target_x"] | 0, 0, gMapSizeX);
  cfg.targetY = constrain(doc["target_y"] | 0, 0, gMapSizeY);
  cfg.runAtEpoch = doc["run_at_epoch"] | 0;
  gMission.setAutoConfig(cfg);

  if (cfg.enabled && cfg.runAtEpoch > 0) {
    gMission.setMode(MODE_AUTO);
    Serial.println("[FS] Resume auto schedule");
  }
}

void clearScheduleFS() {
  if (LittleFS.exists(SCHEDULE_FILE)) LittleFS.remove(SCHEDULE_FILE);
}

// -------------------------------------------------------------------
void publishEvent(const char* eventName, int gasRaw, int angle) {
  if (!gMqtt.connected()) return;

  StaticJsonDocument<256> doc;
  doc["event"] = eventName;
  doc["mode"] = modeToText(gMission.mode());
  doc["auto_state"] = autoStateToText(gMission.autoState());
  doc["epoch"] = nowEpoch();
  if (gasRaw >= 0) doc["gas_raw"] = gasRaw;
  if (angle >= 0) doc["angle_deg"] = angle;

  AutoConfig cfg = gMission.autoConfig();
  Pose p = gMission.pose();
  doc["target_x"] = cfg.targetX;
  doc["target_y"] = cfg.targetY;
  doc["map_size_x"] = gMission.mapSizeX();
  doc["map_size_y"] = gMission.mapSizeY();
  doc["est_x"] = p.x;
  doc["est_y"] = p.y;
  doc["enc_l"] = gOdom.getLeftCount();
  doc["enc_r"] = gOdom.getRightCount();

  char buf[256];
  serializeJson(doc, buf);
  gMqtt.publish("v1/devices/me/telemetry", buf);
}

void publishTelemetry(bool force) {
  if (!gMqtt.connected()) return;
  unsigned long nowMs = millis();
  if (!force && (nowMs - gLastTelemetry < TELEMETRY_INTERVAL_MS)) return;
  gLastTelemetry = nowMs;

  int gasRaw = analogRead(PIN_GAS);
  bool gasAbove = (gasRaw > GAS_ALERT_THRESHOLD);
  if (gasAbove && !gGasAboveThreshold) publishEvent("gas_over_threshold", gasRaw, -1);
  gGasAboveThreshold = gasAbove;
  updateBuzzer(gasRaw);

  Pose p = gMission.pose();
  StaticJsonDocument<192> doc;
  doc["gas_raw"] = gasRaw;
  doc["est_x"] = p.x;
  doc["est_y"] = p.y;
  doc["map_size_x"] = gMission.mapSizeX();
  doc["map_size_y"] = gMission.mapSizeY();
  doc["enc_l"] = gOdom.getLeftCount();
  doc["enc_r"] = gOdom.getRightCount();
  doc["mode"] = modeToText(gMission.mode());
  if (gasAbove) doc["detect"] = true;

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

  if (data.containsKey("mode")) {
    const char* m = data["mode"] | "manual";
    gMission.setMode(strcmp(m, "auto") == 0 ? MODE_AUTO : MODE_MANUAL);
    Serial.printf("[ATTR] mode=%s\n", modeToText(gMission.mode()));
  }

  if (jsonBool(data["stop"])) {
    gMission.stopAll();
    gMission.cancelAuto();
    clearScheduleFS();
    return;
  }

  if (data.containsKey("counts_per_cm")) {
    gCountsPerCm = data["counts_per_cm"].as<float>();
    gMission.setCalibration(gCountsPerCm, gTurn90Counts, gScan360Ms);
  }
  if (data.containsKey("turn_90_counts")) {
    gTurn90Counts = data["turn_90_counts"] | gTurn90Counts;
    gMission.setCalibration(gCountsPerCm, gTurn90Counts, gScan360Ms);
  }
  if (data.containsKey("scan_360_ms")) {
    gScan360Ms = data["scan_360_ms"] | gScan360Ms;
    gMission.setCalibration(gCountsPerCm, gTurn90Counts, gScan360Ms);
  }

  if (data.containsKey("map_size")) {
    const int sz = data["map_size"] | gMapSizeX;
    gMapSizeX = gMapSizeY = sz;
    gMission.setMapSize(gMapSizeX, gMapSizeY);
    Serial.printf("[ATTR] map_size=%d x %d\n", gMapSizeX, gMapSizeY);
  }
  if (data.containsKey("map_size_x")) {
    gMapSizeX = data["map_size_x"] | gMapSizeX;
    gMission.setMapSize(gMapSizeX, gMapSizeY);
  }
  if (data.containsKey("map_size_y")) {
    gMapSizeY = data["map_size_y"] | gMapSizeY;
    gMission.setMapSize(gMapSizeX, gMapSizeY);
  }

  if (gMission.mode() == MODE_MANUAL && (data.containsKey("x") || data.containsKey("y"))) {
    const int duration = data["duration"] | 0;

    if (duration == 0) {
      const Pose p = gMission.pose();
      const int targetX = data.containsKey("x") ? data["x"].as<int>() : p.x;
      const int targetY = data.containsKey("y") ? data["y"].as<int>() : p.y;
      gMission.startManualMoveTo(targetX, targetY);
      Serial.printf("[MANUAL] di toi (%d,%d) tu (%d,%d)\n",
                    targetX, targetY, p.x, p.y);
    } else {
      const int x = data["x"] | 0;
      const int y = data["y"] | 0;
      gMission.driveXY(x, y);
      gMission.setManualDurationMs(duration, millis());
      Serial.printf("[MANUAL] joystick x=%d y=%d dur=%d ms\n", x, y, duration);
    }
  }

  AutoConfig cfg = gMission.autoConfig();
  bool scheduleChanged = false;

  if (data.containsKey("auto_enabled")) {
    cfg.enabled = jsonBool(data["auto_enabled"]);
    if (!cfg.enabled) {
      gMission.cancelAuto();
      clearScheduleFS();
    }
    scheduleChanged = true;
  }
  if (data.containsKey("target_x")) {
    cfg.targetX = constrain(data["target_x"].as<int>(), 0, gMapSizeX);
    scheduleChanged = true;
  }
  if (data.containsKey("target_y")) {
    cfg.targetY = constrain(data["target_y"].as<int>(), 0, gMapSizeY);
    scheduleChanged = true;
  }
  if (data.containsKey("run_at_epoch")) {
    cfg.runAtEpoch = data["run_at_epoch"] | 0;
    scheduleChanged = true;
  }
  gMission.setAutoConfig(cfg);

  if (scheduleChanged && cfg.enabled) {
    saveScheduleToFS();
  }

  if (data.containsKey("auto_start_now")) {
    static bool lastAutoStartNow = false;
    const bool wantStart = jsonBool(data["auto_start_now"]);
    if (wantStart && !lastAutoStartNow) {
      if (gMission.mode() != MODE_AUTO) {
        Serial.println("[ATTR] auto_start_now ignored — set mode=auto truoc");
      } else {
        cfg = gMission.autoConfig();
        cfg.enabled = true;
        cfg.runAtEpoch = 0;
        gMission.setAutoConfig(cfg);
        saveScheduleToFS();
        gMission.startAutoNow();
        Serial.printf("[ATTR] AUTO START → target (%d,%d)\n", cfg.targetX, cfg.targetY);
        publishEvent("auto_started", analogRead(PIN_GAS), -1);
      }
    }
    lastAutoStartNow = wantStart;
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
  applyAttributeJson(data);
}

static bool gMqttSubscribed = false;

void mqttOnConnected() {
  if (gMqttSubscribed) return;

  if (!gMqtt.subscribe("v1/devices/me/attributes")) {
    Serial.println("[MQTT] FAIL subscribe v1/devices/me/attributes");
    return;
  }

  gMqttSubscribed = true;
  Serial.println("[MQTT] OK subscribe v1/devices/me/attributes");
  Serial.println("[MQTT] Cho lenh Shared attributes tu ThingsBoard...");
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
    loadScheduleFromFS();
  }

  connectWiFi();
  setupMqttClient();
  connectMQTT();
  Serial.printf("[CAL] counts/cm=%.1f trimF=%d trimR=%d turnL=%d turnR=%d turn90=%ld\n",
                gCountsPerCm, STRAIGHT_FWD_RIGHT_TRIM, STRAIGHT_REV_LEFT_TRIM,
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

  gMission.processManualTimeout(nowMs);
  gMission.processManualEncMove(nowMs);
  gMission.processAuto(nowMs, gTimeValid, nowEpoch());

  static AutoState lastAutoState = AUTO_DISABLED;
  const AutoState st = gMission.autoState();
  if (st == AUTO_FINISHED && lastAutoState == AUTO_MOVING_TO_TARGET) {
    clearScheduleFS();
    publishEvent("arrived_target", gasRaw, -1);
    publishEvent("gas_at_target", gasRaw, -1);
    publishTelemetry(true);
  }
  lastAutoState = st;

  if (gMission.mode() == MODE_MANUAL) {
    updateBuzzer(gasRaw);
  }

  publishTelemetry(false);
}
