// ============================================================
// ESP32SlamRobot - COMPILE-VERIFIED, HARDWARE-UNTESTED (never run on the robot)
// Built from the course sketch ESP32RobotControl.ino. Build and upload: README.md.
//
// Kept:    the Wi-Fi AP and every course route (/forward ... /scan, /pose, /ping).
// Added:   ARM/STOP, one motor owner task, /drive, /scan2, /status,
//          motor stop on Wi-Fi client loss, optional LiDAR stop zones,
//          a task watchdog on the motor task (a hung task reboots the ESP32,
//          and setup() stops the motors first), Wi-Fi name and password in the
//          untracked wifi_secrets.h, the command seq kept across /arm,
//          boot diagnostics ([BOOT] checkpoints, reset reason, reset-loop count)
//          and a 300 ms motor-task stall check (the shared task watchdog keeps
//          the core's 5 s, which AsyncTCP needs).
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

// Declared before the first function: the Arduino builder puts its generated
// function prototypes (replyMove(..., MoveResult)) above the first function.
enum MoveResult { MOVE_OK, MOVE_DISARMED, MOVE_NO_LIDAR, MOVE_BLOCKED, MOVE_STALE };

// ------------------------------------------------------------
// SETTINGS TO CHECK BEFORE FLASHING
// ------------------------------------------------------------
// Give every robot its own name and password: all course robots otherwise share
// "RobotESP32", and anyone joined to the network can send drive commands.
// ROBOT_SSID and ROBOT_PASSWORD live in wifi_secrets.h, which is never committed:
// copy wifi_secrets.example.h to wifi_secrets.h and edit it (README.md).
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#error "wifi_secrets.h is missing: copy wifi_secrets.example.h to wifi_secrets.h and set ROBOT_SSID and ROBOT_PASSWORD (README.md)."
#endif

constexpr bool sameText(const char* a, const char* b) {
  return *a == *b && (*a == '\0' || sameText(a + 1, b + 1));
}
static_assert(!sameText(ROBOT_PASSWORD, "change-this-password"),
              "wifi_secrets.h still has the example password: choose your own.");
static_assert(sizeof(ROBOT_PASSWORD) - 1 >= 8 && sizeof(ROBOT_PASSWORD) - 1 <= 63,
              "ROBOT_PASSWORD must be 8 to 63 characters (WPA2).");
const int   WIFI_CHANNEL = 6;

const int      MAX_PWM = 80;           // 100 % in /drive. 80 is the library's default speed.
const uint32_t MAX_MOVE_MS = 1000;     // Longest lease of any move. The course sketch used 2000.
const uint32_t SETTLE_MS = 300;        // Scan counts as still this long after motors stop.
const uint32_t LIDAR_STALE_MS = 500;   // No scan for this long: moves stop and are refused.
const bool     I2C_NACK_IS_FAULT = false;  // Set true once /status shows i2c_nacks stays 0.
// Motor task silent this long: loop() reboots the ESP32 (setup() then stops the
// motors first). This is the fast motor check. The shared task watchdog keeps the
// core's 5 s: AsyncTCP's own task sleeps up to 5000 / 4 = 1250 ms between watchdog
// resets, so a shorter shared timeout (the earlier 1000 ms) reboots it in a loop.
const uint32_t MOTOR_STALL_MS = 300;
const uint32_t LIDAR_FIRST_SCAN_WARN_MS = 3000;  // boot log warns if no scan by then
// DIAGNOSTIC BUILD: 1 = the motors can never be armed (/arm answers 423
// MOTORS_LOCKED), so no route can move a wheel. Zero-PWM writes still run.
// Set to 0 only after the S1 cut-off and stopping checks have passed.
#define MOTORS_LOCKED 1

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

// ------------------------------------------------------------
// BOOT DIAGNOSTICS (serial log; survive a software reset in RTC memory)
// ------------------------------------------------------------
const uint32_t DIAG_MAGIC = 0x534C414D;  // "SLAM": RTC values below are valid
RTC_NOINIT_ATTR uint32_t diagMagic;
RTC_NOINIT_ATTR uint32_t diagBootCount;   // boots since power-on
RTC_NOINIT_ATTR uint32_t diagCheckpoint;  // last checkpoint reached by the previous boot
RTC_NOINIT_ATTR uint32_t diagOwnRestart;  // 1 = the previous boot rebooted itself (motor stall)
volatile uint32_t motorHeartbeatMs = 0;   // written by the motor task every cycle
bool motorStopAcked = false;              // the motor board ACKed the zero-PWM write at boot
bool motorTaskOk = false;                 // /arm is refused without the motor task
bool wifiStarted = false;

const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:   return "POWERON (power applied)";
    case ESP_RST_EXT:       return "EXT (reset pin)";
    case ESP_RST_SW:        return "SW (esp_restart)";
    case ESP_RST_PANIC:     return "PANIC (crash or abort: read the backtrace above)";
    case ESP_RST_INT_WDT:   return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (task watchdog: a task did not reset it)";
    case ESP_RST_WDT:       return "WDT (other watchdog)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply voltage dropped: check battery and motors)";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

const char* CHECKPOINT_NAMES[] = {
  "none", "motors zeroed", "serial up", "motor task started", "LiDAR UART started",
  "Wi-Fi AP started", "HTTP server started", "setup complete", "running"};

// One line per startup step, flushed, and remembered across a software reset.
void checkpoint(uint32_t step, const char* detail) {
  diagCheckpoint = step;
  Serial.printf("[BOOT %lu ms] %lu %s: %s\n", (unsigned long)millis(), (unsigned long)step,
                CHECKPOINT_NAMES[step], detail);
  Serial.flush();
}

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
  // Backstop: the shared task watchdog (5 s). The fast check is MOTOR_STALL_MS in loop().
  esp_task_wdt_add(nullptr);
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    esp_task_wdt_reset();
    motorHeartbeatMs = millis();   // loop() reboots if this stops for MOTOR_STALL_MS
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

  char json[640];
  snprintf(json, sizeof(json),
           "{\"boot_id\":%lu,\"uptime_ms\":%lu,\"reset_reason\":%d,\"boot_count\":%lu,"
           "\"motor_ack\":%d,\"motors_locked\":%d,\"armed\":%d,\"fault\":%d,"
           "\"moving\":%d,\"lease_ms\":%lu,\"cmd_seq\":%lu,\"scan_seq\":%lu,\"scan_age_ms\":%lu,"
           "\"scan_points\":%u,\"lidar_dps\":%u,\"crc_fails\":%u,\"oversize_scans\":%lu,"
           "\"i2c_nacks\":%lu,\"guard_on\":%d,\"guard_trips\":%lu,"
           "\"sector_mm\":[%u,%u,%u,%u,%u,%u,%u,%u],\"clients\":%d,\"free_heap\":%lu}",
           (unsigned long)bootId, (unsigned long)now, (int)esp_reset_reason(),
           (unsigned long)diagBootCount, motorStopAcked, MOTORS_LOCKED, armedNow, faultNow,
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
  const int16_t zero[4] = {0, 0, 0, 0};
  motorStopAcked = writeWheels(zero);  // the same zeros again, with the ACK bits checked

  Serial.begin(115200);
  delay(200);
  bootId = esp_random();
  for (int i = 0; i < 8; ++i) {
    sectorMinMm[i] = 65535;
    sectorNear[i] = 0;
  }

  // Why the previous boot ended. The ROM line "rst:0xc (SW_CPU_RESET)" is the same
  // for a panic, a task watchdog, a brownout and esp_restart(); this tells them apart.
  const esp_reset_reason_t reason = esp_reset_reason();
  const bool rtcValid = diagMagic == DIAG_MAGIC && reason != ESP_RST_POWERON;
  const uint32_t previousCheckpoint = rtcValid ? diagCheckpoint : 0;
  const bool previousOwnRestart = rtcValid && diagOwnRestart == 1;
  if (!rtcValid) {
    diagMagic = DIAG_MAGIC;
    diagBootCount = 0;
  }
  diagBootCount++;
  diagCheckpoint = 0;
  diagOwnRestart = 0;
  Serial.println();
  Serial.println("=== ESP32SlamRobot: COMPILE-VERIFIED, HARDWARE-UNTESTED ===");
  Serial.printf("[BOOT] reset reason: %s\n", resetReasonName(reason));
  if (rtcValid) {
    Serial.printf("[BOOT] boot %lu since power-on; the previous boot reached checkpoint %lu (%s)\n",
                  (unsigned long)diagBootCount, (unsigned long)previousCheckpoint,
                  CHECKPOINT_NAMES[previousCheckpoint < 9 ? previousCheckpoint : 0]);
  }
  if (previousOwnRestart) Serial.println("[BOOT] the previous boot rebooted itself: motor task stalled");
  if (diagBootCount >= 3) {
    Serial.println("[BOOT] WARNING: 3+ boots without a power cycle = RESET LOOP. Copy the lines "
                   "printed just before 'rst:' (panic, task_wdt or Brownout).");
  }
  checkpoint(1, motorStopAcked ? "zero PWM written; the motor board ACKed"
                               : "zero PWM written; the motor board did NOT ACK: the motors may "
                                 "not be stopped - cut motor power");
  checkpoint(2, "Serial 115200 baud");

  // Priority above loop(), so a stalled loop() cannot delay a lease expiry.
  motorTaskOk =
      xTaskCreatePinnedToCore(motorTask, "motor", 4096, nullptr, 3, nullptr, 1) == pdPASS;
  checkpoint(3, motorTaskOk ? "10 ms motor task on core 1; stall check 300 ms; task watchdog 5 s"
                            : "FAILED to create the motor task: staying DISARMED");

  // LIDAR UART
  Serial2.setRxBufferSize(2048);  // About 115 ms of LD06 data.
  Serial2.begin(LIDAR_BAUD, SERIAL_8N1, RX_LIDAR, TX_LIDAR);
  Serial2.setTimeout(2);
  lidar.init();  // Sets the LiDAR PWM input on GPIO32 HIGH (keeps RX on GPIO33).
  checkpoint(4, "Serial2 230400 baud, RX GPIO33, PWM GPIO32 HIGH; first scan reported below");

  // WIFI AP
  wifiStarted = WiFi.mode(WIFI_AP) && WiFi.softAP(ROBOT_SSID, ROBOT_PASSWORD, WIFI_CHANNEL);
  WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t) { disarmAndStop(); },
               ARDUINO_EVENT_WIFI_AP_STADISCONNECTED);
  delay(200);
  if (wifiStarted) {
    char detail[96];
    snprintf(detail, sizeof(detail), "SSID \"%s\", IP %s, channel %d", ROBOT_SSID,
             WiFi.softAPIP().toString().c_str(), WIFI_CHANNEL);
    checkpoint(5, detail);
  } else {
    checkpoint(5, "FAILED: no access point; the robot stays DISARMED (no HTTP control)");
  }
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
#if MOTORS_LOCKED
    replyText(r, 423, "MOTORS_LOCKED");  // diagnostic build: movement disabled
    return;
#endif
    if (!motorTaskOk) {
      replyText(r, 503, "MOTOR_TASK_FAILED");  // nothing would stop a lease: never arm
      return;
    }
    portENTER_CRITICAL(&stateMux);
    endMoveLocked(millis());
    fault = false;
    armed = true;
    // lastCommandSeq is NOT reset: a /drive sent before a STOP and delayed in
    // the network must stay STALE after the next ARM. It restarts only at boot.
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
  checkpoint(6, "port 80 (starts the async_tcp task)");
  enableLoopWDT();  // Restarts the ESP32 if loop() hangs (shared task watchdog, 5 s).
  checkpoint(7, MOTORS_LOCKED ? "robot DISARMED; MOTORS_LOCKED build: /arm is refused"
                              : "robot DISARMED; waiting for /arm");
}

// ------------------------------------------------------------
// LOOP
// ------------------------------------------------------------
void loop() {
  // Fast motor check: read the heartbeat BEFORE the clock, so the age never underflows.
  const uint32_t beat = motorHeartbeatMs;
  const uint32_t now = millis();
  if (beat != 0 && now - beat > MOTOR_STALL_MS) {
    Serial.printf("[FATAL] motor task silent for %lu ms: rebooting (setup() zeroes the motors "
                  "first)\n", (unsigned long)(now - beat));
    diagOwnRestart = 1;
    Serial.flush();
    esp_restart();
  }

  if (lidar.readScan()) publishScan();

  static bool lidarReported = false;
  static bool lidarWarned = false;
  if (!lidarReported && scanSeq > 0) {
    lidarReported = true;
    char detail[64];
    snprintf(detail, sizeof(detail), "first LiDAR scan: %u points, %u deg/s", scanPoints,
             lidarSpeedDps);
    checkpoint(8, detail);
  } else if (!lidarReported && !lidarWarned && now > LIDAR_FIRST_SCAN_WARN_MS) {
    lidarWarned = true;
    Serial.println("[LIDAR] WARNING: no valid scan after 3 s. Check LD06 power, its TX wire "
                   "to GPIO33 and the motor spinning. /drive answers NO_LIDAR until scans arrive.");
  }
  delay(1);  // Lets lower-priority tasks run. The 2048-byte UART buffer covers this.
}
