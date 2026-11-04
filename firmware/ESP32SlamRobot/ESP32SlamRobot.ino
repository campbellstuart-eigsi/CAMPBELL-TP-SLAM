// ============================================================
// ESP32SlamRobot - DRAFT, NOT YET RUN ON THE ROBOT
// Built from the course sketch ESP32RobotControl.ino.
//
// Kept:    the Wi-Fi AP and every course route (/forward ... /scan, /pose, /ping).
// Added:   ARM/STOP, one motor owner task, /drive, /scan2, /status,
//          motor stop on Wi-Fi client loss, optional LiDAR stop zones,
//          a task watchdog on the motor task (a hung task reboots the ESP32,
//          and setup() stops the motors first).
//
// Put ld06.h, ld06.cpp and ld06crc.h in this sketch folder (as in the course
// sketch) and install the MecanumCar_v2, AsyncTCP and ESPAsyncWebServer libraries.
//
// FIRST TEST: wheels off the ground, with a hand on the power switch.
// ============================================================
#include <WiFi.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ld06.h"
#include "MecanumCar_v2.h"

// ------------------------------------------------------------
// SETTINGS TO CHECK BEFORE FLASHING
// ------------------------------------------------------------
// Give every robot its own name and password: all course robots otherwise share
// "RobotESP32", and anyone joined to the network can send drive commands.
const char* ROBOT_SSID = "RobotESP32-SLAM";
const char* ROBOT_PASSWORD = "change-this-password";
const int   WIFI_CHANNEL = 6;

const int      MAX_PWM = 80;           // 100 % in /drive. 80 is the library's default speed.
const uint32_t MAX_MOVE_MS = 1000;     // Longest lease of any move. The course sketch used 2000.
const uint32_t SETTLE_MS = 300;        // Scan counts as still this long after motors stop.
const uint32_t LIDAR_STALE_MS = 500;   // No scan for this long: moves stop and are refused.
const bool     I2C_NACK_IS_FAULT = false;  // Set true once /status shows i2c_nacks stays 0.
const uint32_t TASK_WDT_MS = 1000;     // Motor task or loop() silent this long: reboot.

// Keep false until the LiDAR front direction and body mask have been measured.
const bool     GUARD_ENABLED = false;
const int      LIDAR_FRONT_DEG = 0;    // Robot-front direction measured in LiDAR degrees.
const uint16_t GUARD_MIN_MM = 150;     // Ignore closer points (the robot's own body).
const uint16_t GUARD_STOP_MM = 250;    // Stop when an obstacle is nearer than this.
const uint8_t  GUARD_MIN_INTENSITY = 20;
const uint8_t  GUARD_MIN_POINTS = 3;

// ------------------------------------------------------------
// HARDWARE
// ------------------------------------------------------------
#define RX_LIDAR 33
#define TX_LIDAR -1
#define LD06_PWM_PIN 32
#define LIDAR_BAUD 230400
LD06 lidar(Serial2, LD06_PWM_PIN);

mecanumCar car(19, 21);  // Bit-banged I2C: SDA 19, SCL 21. Only motorStep() uses it after setup.

const uint8_t MOTOR_I2C_WRITE = 0x30 << 1;
// {reverse register, forward register}, same as Motor_Upper_L() etc. in MecanumCar_v2.cpp.
enum { FL, FR, RL, RR };
const uint8_t WHEEL_REG[4][2] = {{3, 4}, {1, 2}, {7, 8}, {5, 6}};

AsyncWebServer server(80);

// ------------------------------------------------------------
// SHARED STATE
// ------------------------------------------------------------
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE scanMux = portMUX_INITIALIZER_UNLOCKED;

bool armed = false;                // Always false at boot.
bool fault = false;
bool moveActive = false;
int16_t moveWheel[4] = {0, 0, 0, 0};  // Signed PWM, wanted by the current move.
int moveVx = 0, moveVy = 0;        // Percent, used by the stop zones.
uint32_t moveStartMs = 0;
uint32_t moveLeaseMs = 0;
uint32_t lastMotionEndMs = 0;
uint32_t lastCommandSeq = 0;

uint32_t i2cNacks = 0;
uint32_t guardTrips = 0;
uint32_t bootId = 0;

// Declared here, not before requestMove(): the Arduino builder puts its generated
// function prototypes (replyMove(..., MoveResult)) above the first function.
enum MoveResult { MOVE_OK, MOVE_DISARMED, MOVE_NO_LIDAR, MOVE_BLOCKED, MOVE_STALE };

uint32_t scanSeq = 0;
uint32_t scanEndMs = 0;
uint32_t oversizeScans = 0;
uint16_t scanPoints = 0;
uint16_t lidarSpeedDps = 0;
uint8_t scanStill = 0;
uint16_t sectorMinMm[8];           // Nearest valid point per 45 degree sector, robot frame.
uint8_t sectorNear[8];             // Points inside the stop distance per sector.

// ROBOT POSE: stored and echoed only. The laptop writes it with /pose.
int16_t posex = 0;
int16_t posey = 0;
uint16_t posez = 0;

// ------------------------------------------------------------
// MOTOR OUTPUT: runs only inside setup() and motorStep()
// ------------------------------------------------------------
bool writeRegister(uint8_t reg, uint8_t value) {
  // Same bytes as mecanumCar::Writebyte(), but the ACK bits are kept.
  bool nack = false;
  car.IIC_Start();
  car.IIC_SendByte(MOTOR_I2C_WRITE);
  nack |= car.IIC_RecvACK();
  car.IIC_SendByte(reg);
  nack |= car.IIC_RecvACK();
  car.IIC_SendByte(value);
  nack |= car.IIC_RecvACK();
  car.IIC_Stop();
  return !nack;
}

bool writeWheels(const int16_t wheel[4]) {
  bool ok = true;
  for (int i = 0; i < 4; ++i) {
    const int pwm = constrain(abs((int)wheel[i]), 0, MAX_PWM);
    const bool forwardDirection = wheel[i] > 0;
    // Clear the unused direction first so both inputs are never driven together.
    ok &= writeRegister(WHEEL_REG[i][forwardDirection ? 0 : 1], 0);
    ok &= writeRegister(WHEEL_REG[i][forwardDirection ? 1 : 0], pwm);
  }
  return ok;
}

// Wheel signs match Advance, L_Move and Turn_Left in MecanumCar_v2.cpp:
// +vx forward, +vy left, +wz counter-clockwise. Inputs are percent.
void mixWheels(int vx, int vy, int wz, int16_t wheel[4]) {
  long mix[4] = {(long)vx - vy - wz, (long)vx + vy + wz,
                 (long)vx + vy - wz, (long)vx - vy + wz};
  long largest = 100;
  for (int i = 0; i < 4; ++i) largest = max(largest, labs(mix[i]));
  for (int i = 0; i < 4; ++i) wheel[i] = (int16_t)(mix[i] * MAX_PWM / largest);
}

int travelSector(int vx, int vy) {
  float degrees = atan2((float)vy, (float)vx) * 180.0f / PI;
  if (degrees < 0) degrees += 360.0f;
  return ((int)((degrees + 22.5f) / 45.0f)) % 8;
}

// Call with stateMux held.
bool pathBlockedLocked(int vx, int vy) {
  if (!GUARD_ENABLED || (vx == 0 && vy == 0)) return false;
  const int sector = travelSector(vx, vy);
  for (int offset = -1; offset <= 1; ++offset) {
    if (sectorNear[(sector + offset + 8) % 8] >= GUARD_MIN_POINTS) return true;
  }
  return false;
}

void endMoveLocked(uint32_t now) {
  if (moveActive) lastMotionEndMs = now;
  moveActive = false;
  for (int i = 0; i < 4; ++i) moveWheel[i] = 0;
}

// One 10 ms motor cycle. This is the only code that decides motor output.
void motorStep() {
  static int16_t applied[4] = {1, 1, 1, 1};  // Forces a zero write on the first cycle.
  static uint32_t lastRefreshMs = 0;
  const uint32_t now = millis();
  int16_t target[4];

  portENTER_CRITICAL(&stateMux);
  bool stopNow = !armed || fault;
  if (moveActive) {
    if ((uint32_t)(now - moveStartMs) >= moveLeaseMs) stopNow = true;
    if ((uint32_t)(now - scanEndMs) >= LIDAR_STALE_MS) stopNow = true;
    if (pathBlockedLocked(moveVx, moveVy)) {
      stopNow = true;
      guardTrips++;
    }
  }
  if (stopNow) endMoveLocked(now);
  for (int i = 0; i < 4; ++i) target[i] = moveWheel[i];
  portEXIT_CRITICAL(&stateMux);

  bool changed = false;
  bool allZero = true;
  for (int i = 0; i < 4; ++i) {
    changed |= (target[i] != applied[i]);
    allZero &= (target[i] == 0);
  }
  // The board holds its last PWM values, so a stop is repeated in case one was lost.
  const bool refreshStop = allZero && (uint32_t)(now - lastRefreshMs) >= 200;
  if (!changed && !refreshStop) return;

  const bool ok = writeWheels(target);
  lastRefreshMs = now;
  for (int i = 0; i < 4; ++i) applied[i] = target[i];
  if (!ok) {
    portENTER_CRITICAL(&stateMux);
    i2cNacks++;
    if (I2C_NACK_IS_FAULT) {
      fault = true;
      armed = false;
    }
    portEXIT_CRITICAL(&stateMux);
  }
}

void motorTask(void*) {
  esp_task_wdt_add(nullptr);  // A hang here (e.g. inside the I2C writes) reboots the ESP32.
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    esp_task_wdt_reset();
    motorStep();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

// ------------------------------------------------------------
// MOVE REQUESTS (never touch the motor bus)
// ------------------------------------------------------------

void disarmAndStop() {
  portENTER_CRITICAL(&stateMux);
  armed = false;
  endMoveLocked(millis());
  portEXIT_CRITICAL(&stateMux);
}

MoveResult requestMove(int vx, int vy, int wz, int timeMs, uint32_t seq) {
  const uint32_t now = millis();
  int16_t wheel[4];
  mixWheels(vx, vy, wz, wheel);
  MoveResult result = MOVE_OK;

  portENTER_CRITICAL(&stateMux);
  if (seq != 0 && seq <= lastCommandSeq) {
    result = MOVE_STALE;
  } else if (!armed || fault) {
    result = MOVE_DISARMED;
  } else if ((uint32_t)(now - scanEndMs) >= LIDAR_STALE_MS || scanSeq == 0) {
    result = MOVE_NO_LIDAR;
  } else if (pathBlockedLocked(vx, vy)) {
    result = MOVE_BLOCKED;
  } else {
    if (seq != 0) lastCommandSeq = seq;
    if (timeMs <= 0) {
      endMoveLocked(now);
    } else {
      moveLeaseMs = (uint32_t)constrain(timeMs, 1, (int)MAX_MOVE_MS);
      moveStartMs = now;
      moveVx = vx;
      moveVy = vy;
      for (int i = 0; i < 4; ++i) moveWheel[i] = wheel[i];
      moveActive = true;
    }
  }
  portEXIT_CRITICAL(&stateMux);
  return result;
}

// ------------------------------------------------------------
// HTTP HELPERS
// ------------------------------------------------------------
int getArg(AsyncWebServerRequest* req, const char* name, int def) {
  return req->hasParam(name) ? req->getParam(name)->value().toInt() : def;
}

void addCORS(AsyncWebServerResponse* resp) {
  resp->addHeader("Access-Control-Allow-Origin", "*");
  resp->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  resp->addHeader("Access-Control-Allow-Headers", "*");
}

void replyText(AsyncWebServerRequest* req, int code, const char* text) {
  auto response = req->beginResponse(code, "text/plain", text);
  addCORS(response);
  req->send(response);
}

void replyMove(AsyncWebServerRequest* req, MoveResult result) {
  switch (result) {
    case MOVE_OK:       replyText(req, 200, "OK"); break;
    case MOVE_DISARMED: replyText(req, 423, "DISARMED"); break;
    case MOVE_NO_LIDAR: replyText(req, 503, "NO_LIDAR"); break;
    case MOVE_BLOCKED:  replyText(req, 409, "BLOCKED"); break;
    case MOVE_STALE:    replyText(req, 409, "STALE_SEQ"); break;
  }
}

void addLegacyMove(const char* path, int vx, int vy, int wz, int divisor) {
  // Same request and reply as the course routes. Rotation keeps the course's time/2.
  server.on(path, HTTP_GET, [vx, vy, wz, divisor](AsyncWebServerRequest* r) {
    replyMove(r, requestMove(vx, vy, wz, getArg(r, "time", 300) / divisor, 0));
  });
}

// ------------------------------------------------------------
// LIDAR SCANS
// ------------------------------------------------------------
const uint16_t MAX_SCAN_POINTS = 600;  // Course value 460 dropped scans below about 9.8 Hz.
const size_t MAX_SCAN_BYTES = 8 + 5 * MAX_SCAN_POINTS;
const size_t SCAN2_HEADER_BYTES = 16;
uint8_t buildBuffer[MAX_SCAN_BYTES];   // loop() only
uint8_t publishedScan[MAX_SCAN_BYTES];
size_t publishedSize = 0;
uint8_t sendBuffer[SCAN2_HEADER_BYTES + MAX_SCAN_BYTES];  // Web task only

void publishScan() {
  const uint16_t n = lidar.getNbPointsInScan();
  if (n == 0) return;
  if (n > MAX_SCAN_POINTS) {
    portENTER_CRITICAL(&stateMux);
    oversizeScans++;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  uint16_t minMm[8];
  uint8_t nearCount[8];
  for (int i = 0; i < 8; ++i) {
    minMm[i] = 65535;
    nearCount[i] = 0;
  }

  buildBuffer[6] = n & 0xff;
  buildBuffer[7] = n >> 8;
  size_t off = 8;
  for (uint16_t i = 0; i < n; ++i) {
    const auto* pt = lidar.getPoints(i);
    const uint16_t angle = (uint16_t)(pt->angle * 100);
    buildBuffer[off++] = angle & 0xff;
    buildBuffer[off++] = angle >> 8;
    buildBuffer[off++] = pt->distance & 0xff;
    buildBuffer[off++] = pt->distance >> 8;
    buildBuffer[off++] = pt->intensity;

    if (pt->distance >= GUARD_MIN_MM && pt->intensity >= GUARD_MIN_INTENSITY) {
      float robotDegrees = pt->angle - LIDAR_FRONT_DEG;
      while (robotDegrees < 0) robotDegrees += 360.0f;
      while (robotDegrees >= 360.0f) robotDegrees -= 360.0f;
      const int sector = ((int)((robotDegrees + 22.5f) / 45.0f)) % 8;
      if (pt->distance < minMm[sector]) minMm[sector] = pt->distance;
      if (pt->distance <= GUARD_STOP_MM && nearCount[sector] < 255) nearCount[sector]++;
    }
  }

  const uint32_t now = millis();
  const uint32_t scanTimeMs = (uint32_t)n * 2 / 9;  // 4500 samples per second.
  const uint32_t startedMs = now - scanTimeMs;
  const uint16_t speed = lidar.getSpeed();

  portENTER_CRITICAL(&scanMux);
  memcpy(publishedScan + 6, buildBuffer + 6, off - 6);
  publishedSize = off;
  portEXIT_CRITICAL(&scanMux);

  portENTER_CRITICAL(&stateMux);
  scanSeq++;
  scanEndMs = now;
  scanPoints = n;
  lidarSpeedDps = speed;
  scanStill = (!moveActive && (int32_t)(startedMs - lastMotionEndMs) >= (int32_t)SETTLE_MS) ? 1 : 0;
  for (int i = 0; i < 8; ++i) {
    sectorMinMm[i] = minMm[i];
    sectorNear[i] = nearCount[i];
  }
  portEXIT_CRITICAL(&stateMux);
}

size_t copyLegacyScan(uint8_t* out) {
  portENTER_CRITICAL(&scanMux);
  const size_t size = publishedSize;
  memcpy(out, publishedScan, size);
  out[0] = posex & 0xff; out[1] = (uint16_t)posex >> 8;
  out[2] = posey & 0xff; out[3] = (uint16_t)posey >> 8;
  out[4] = posez & 0xff; out[5] = posez >> 8;
  portEXIT_CRITICAL(&scanMux);
  return size;
}

void sendBinary(AsyncWebServerRequest* req, const uint8_t* data, size_t size) {
  auto response = req->beginResponseStream("application/octet-stream");
  response->write(data, size);
  addCORS(response);
  req->send(response);
}

void sendScanBinary(AsyncWebServerRequest* req) {
  const size_t size = copyLegacyScan(sendBuffer);
  if (!size) { replyText(req, 503, "NO_VALID_SCAN"); return; }
  sendBinary(req, sendBuffer, size);
}

// /scan2 = 16-byte header + the same bytes as /scan. All values are little-endian.
// 0 'S'  1 '2'  2 version=1  3 flags (bit0 still)  4 seq u32  8 end_ms u32
// 12 lidar_deg_per_s u16  14 reserved u16
void sendScan2(AsyncWebServerRequest* req) {
  const size_t size = copyLegacyScan(sendBuffer + SCAN2_HEADER_BYTES);
  if (!size) { replyText(req, 503, "NO_VALID_SCAN"); return; }
  portENTER_CRITICAL(&stateMux);
  const uint32_t seq = scanSeq;
  const uint32_t endMs = scanEndMs;
  const uint16_t speed = lidarSpeedDps;
  const uint8_t still = scanStill;
  portEXIT_CRITICAL(&stateMux);

  sendBuffer[0] = 'S';
  sendBuffer[1] = '2';
  sendBuffer[2] = 1;
  sendBuffer[3] = still;
  for (int i = 0; i < 4; ++i) {
    sendBuffer[4 + i] = (seq >> (8 * i)) & 0xff;
    sendBuffer[8 + i] = (endMs >> (8 * i)) & 0xff;
  }
  sendBuffer[12] = speed & 0xff;
  sendBuffer[13] = speed >> 8;
  sendBuffer[14] = 0;
  sendBuffer[15] = 0;
  sendBinary(req, sendBuffer, SCAN2_HEADER_BYTES + size);
}

void sendStatus(AsyncWebServerRequest* req) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&stateMux);
  const bool armedNow = armed, faultNow = fault, movingNow = moveActive;
  const uint32_t remaining = moveActive ? moveLeaseMs - min(moveLeaseMs, now - moveStartMs) : 0;
  const uint32_t seq = scanSeq, age = now - scanEndMs, cmdSeq = lastCommandSeq;
  const uint32_t nacks = i2cNacks, trips = guardTrips, oversize = oversizeScans;
  const uint16_t points = scanPoints, speed = lidarSpeedDps;
  uint16_t sectors[8];
  for (int i = 0; i < 8; ++i) sectors[i] = sectorMinMm[i];
  portEXIT_CRITICAL(&stateMux);

  char json[560];
  snprintf(json, sizeof(json),
           "{\"boot_id\":%lu,\"uptime_ms\":%lu,\"reset_reason\":%d,\"armed\":%d,\"fault\":%d,"
           "\"moving\":%d,\"lease_ms\":%lu,\"cmd_seq\":%lu,\"scan_seq\":%lu,\"scan_age_ms\":%lu,"
           "\"scan_points\":%u,\"lidar_dps\":%u,\"crc_fails\":%u,\"oversize_scans\":%lu,"
           "\"i2c_nacks\":%lu,\"guard_on\":%d,\"guard_trips\":%lu,"
           "\"sector_mm\":[%u,%u,%u,%u,%u,%u,%u,%u],\"clients\":%d,\"free_heap\":%lu}",
           (unsigned long)bootId, (unsigned long)now, (int)esp_reset_reason(), armedNow, faultNow,
           movingNow, (unsigned long)remaining, (unsigned long)cmdSeq, (unsigned long)seq,
           (unsigned long)age, points, speed, lidar.getChecksumFailCount(),
           (unsigned long)oversize, (unsigned long)nacks, GUARD_ENABLED, (unsigned long)trips,
           sectors[0], sectors[1], sectors[2], sectors[3], sectors[4], sectors[5], sectors[6],
           sectors[7], WiFi.softAPgetStationNum(), (unsigned long)ESP.getFreeHeap());
  auto response = req->beginResponse(200, "application/json", json);
  addCORS(response);
  req->send(response);
}

// ------------------------------------------------------------
// SETUP
// ------------------------------------------------------------
void setup() {
  // First action: the board keeps its last PWM values across an ESP32 reset.
  car.Stop();

  Serial.begin(115200);
  delay(200);
  bootId = esp_random();
  for (int i = 0; i < 8; ++i) {
    sectorMinMm[i] = 65535;
    sectorNear[i] = 0;
  }

  // Task watchdog: TASK_WDT_MS instead of the default 5 s, and a panic (reboot)
  // on expiry. Idle tasks are not watched; only the motor task and loop() are.
  const esp_task_wdt_config_t wdtConfig = {TASK_WDT_MS, 0, true};
  if (esp_task_wdt_reconfigure(&wdtConfig) == ESP_ERR_INVALID_STATE) esp_task_wdt_init(&wdtConfig);

  // Priority above loop(), so a stalled loop() cannot delay a lease expiry.
  xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 3, nullptr, 1);

  // LIDAR UART
  Serial2.setRxBufferSize(2048);  // About 115 ms of LD06 data.
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(2);
  lidar.init();  // Sets the LiDAR PWM input on GPIO32 HIGH.

  // WIFI AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ROBOT_SSID, ROBOT_PASSWORD, WIFI_CHANNEL);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { disarmAndStop(); },
               ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  delay(200);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // COURSE ROUTES
  addLegacyMove("/forward", 100, 0, 0, 1);
  addLegacyMove("/backward", -100, 0, 0, 1);
  addLegacyMove("/left", 0, 100, 0, 1);
  addLegacyMove("/right", 0, -100, 0, 1);
  addLegacyMove("/rotateleft", 0, 0, 100, 2);
  addLegacyMove("/rotateright", 0, 0, -100, 2);

  server.on("/scan", HTTP_GET, [](AsyncWebServerRequest* r) { sendScanBinary(r); });

  server.on("/pose", HTTP_GET, [](AsyncWebServerRequest* r) {
    const int xPose = constrain(getArg(r, "x", 0), -32768, 32767);
    const int yPose = constrain(getArg(r, "y", 0), -32768, 32767);
    const int angle = getArg(r, "z", 0);
    portENTER_CRITICAL(&scanMux);
    posex = xPose;
    posey = yPose;
    posez = ((angle % 36000) + 36000) % 36000;
    portEXIT_CRITICAL(&scanMux);
    replyText(r, 200, "OK");
  });

  server.on("/ping", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send(200, "text/plain", "OK");
  });

  // Stops the motors and disarms. Moving again needs a deliberate /arm.
  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest* r) {
    disarmAndStop();
    replyText(r, 200, "STOPPED");
  });

  // ADDED ROUTES
  server.on("/arm", HTTP_GET, [](AsyncWebServerRequest* r) {
    portENTER_CRITICAL(&stateMux);
    endMoveLocked(millis());
    fault = false;
    armed = true;
    lastCommandSeq = 0;
    portEXIT_CRITICAL(&stateMux);
    replyText(r, 200, "ARMED");
  });

  // /drive?vx=&vy=&wz=&time=&seq=  percent of MAX_PWM, signs as in mixWheels().
  server.on("/drive", HTTP_GET, [](AsyncWebServerRequest* r) {
    const int vx = constrain(getArg(r, "vx", 0), -100, 100);
    const int vy = constrain(getArg(r, "vy", 0), -100, 100);
    const int wz = constrain(getArg(r, "wz", 0), -100, 100);
    replyMove(r, requestMove(vx, vy, wz, getArg(r, "time", 0),
                             (uint32_t)max(getArg(r, "seq", 0), 0)));
  });

  server.on("/scan2", HTTP_GET, [](AsyncWebServerRequest* r) { sendScan2(r); });
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest* r) { sendStatus(r); });

  server.begin();
  enableLoopWDT();  // Restarts the ESP32 if loop() hangs.
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  if (lidar.readScan()) publishScan();
  delay(1);  // Lets lower-priority tasks run. The 2048-byte UART buffer covers this.
}
