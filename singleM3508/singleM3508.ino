#include <SPI.h>
#include "mcp2515.h"
#include "RPi_Pico_TimerInterrupt.h"
#include "RoboMaster.h"

// ---------------- Pin config ----------------
#define MOSI_PIN D10
#define CS_PIN   D3
#define SCK_PIN  D8
#define MISO_PIN D9

// ---------------- Gear ratio ----------------
// IMPORTANT: The M3508's internal 19:1 planetary gearbox is ALREADY
// baked into the library (see M3508Traits::innerGearRatio = 19.0 in
// RoboMaster.h). The library computes total reduction as:
//     gearRatio * Traits::innerGearRatio
// So we only pass the EXTERNAL ratio here (the cycloidal drive).
// Effective total reduction = 20.0 (external) * 19.0 (internal) = 380:1.
#define EXTERNAL_GEAR_RATIO 20.0f

// ---------------- Step sizes ----------------
#define ANGLE_STEP_COARSE_DEG   45.0f
#define ANGLE_STEP_FINE_DEG     5.0f
#define SPEED_STEP_COARSE_RPM   5.0f
#define SPEED_STEP_FINE_RPM     0.5f
#define CURRENT_STEP_COARSE_A   0.5f
#define CURRENT_STEP_FINE_A     0.05f

MCP2515 mcp(CS_PIN, 8000000, &SPI);
struct can_frame sendMsg[2] = {}, readMsg = {};
RPI_PICO_Timer recvTimer(0);
RPI_PICO_Timer sendTimer(1);
RPI_PICO_Timer serialTimer(2);
M3508 m3508;

struct {
  uint32_t recv   = 1000000 / FEEDBACK_500HZ;
  uint32_t send   = 1000000 / FEEDBACK_500HZ;
  uint32_t serial = 1000000 / 20;
} dt;

struct {
  volatile bool recvMotor = false;
  volatile bool sendMotor = false;
  volatile bool serialIO  = false;
} flag;

bool fineMode = false;
uint32_t printCount = 0;

void initSPI();
void initMCP();
void initTimer();
void setMotorParam();
void recvMotor();
void sendMotor();
void serialIO();
void printHeader();
void printHelp();
const char* modeName(MODE m);
bool recvMotorFlag(struct repeating_timer *t);
bool sendMotorFlag(struct repeating_timer *t);
bool serialIOFlag(struct repeating_timer *t);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  initSPI();
  initMCP();
  setMotorParam();
  initTimer();
  delay(200);
  printHelp();
  printHeader();
}

void loop() {
  if (flag.recvMotor) recvMotor();
  if (flag.sendMotor) sendMotor();
  if (flag.serialIO)  serialIO();
}

void initSPI() {
  SPI.setMISO(MISO_PIN);
  SPI.setSCK(SCK_PIN);
  SPI.setMOSI(MOSI_PIN);
  SPI.begin();
}

void initMCP() {
  mcp.reset();
  mcp.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp.setNormalMode();
}

void initTimer() {
  recvTimer.attachInterruptInterval(dt.recv,     recvMotorFlag);
  sendTimer.attachInterruptInterval(dt.send,     sendMotorFlag);
  serialTimer.attachInterruptInterval(dt.serial, serialIOFlag);
}

void setMotorParam() {
  m3508.mode             = MODE::SLEEP;
  m3508.gearRatio        = EXTERNAL_GEAR_RATIO;   // external only; internal 19:1 auto
  m3508.direction        = DIRECTION::FWD;

  // Starting-point gains with correct gear ratio.
  // Max output-shaft speed ~= 9000 motor RPM / 380 ~= 23 RPM.
  // So realistic speed commands are in the 0-20 RPM range.
  // Format: { Kp, Ki, Kd, integralLimit, outputLimit }
  m3508.pidParam.angle   = { 3.0, 3.0, 0.0, 3600.0, 8.0 };  // out = target RPM
  m3508.pidParam.speed   = { 1.5, 0.0, 0.0, 450.0,  20.0 };  // out = target current A
  m3508.pidParam.current = { 0.0, 0.0, 0.0,   20.0, 20.0 };  // passthrough
  m3508.setPidInterval(8, 4, 2);
  m3508.init(1);
}

void recvMotor() {
  MCP2515::ERROR status = mcp.readMessage(&readMsg);
  if (status == MCP2515::ERROR_OK) m3508.refresh(micros(), sendMsg, readMsg);
  flag.recvMotor = false;
}

void sendMotor() {
  for (uint8_t i = 0; i < 2; i++) {
    if (sendMsg[i].can_id != 0x00) {
      mcp.sendMessage(&sendMsg[i]);
      sendMsg[i].can_id = 0x00;
    }
  }
  flag.sendMotor = false;
}

void serialIO() {
  while (Serial.available()) {
    char c = Serial.read();

    float angleStep   = fineMode ? ANGLE_STEP_FINE_DEG   : ANGLE_STEP_COARSE_DEG;
    float speedStep   = fineMode ? SPEED_STEP_FINE_RPM   : SPEED_STEP_COARSE_RPM;
    float currentStep = fineMode ? CURRENT_STEP_FINE_A   : CURRENT_STEP_COARSE_A;

    switch (c) {
      case '0':
        m3508.mode = MODE::SLEEP;
        m3508.target.angle = 0.0;
        m3508.target.speed = 0.0;
        m3508.target.current = 0.0;
        m3508.resetIntegral();
        Serial.println("# MODE: SLEEP (all targets zeroed)");
        break;
      case '1':
        m3508.mode = MODE::ANGLE;
        Serial.println("# MODE: ANGLE");
        break;
      case '2':
        m3508.mode = MODE::SPEED;
        Serial.println("# MODE: SPEED");
        break;
      case '3':
        m3508.mode = MODE::CURRENT;
        Serial.println("# MODE: CURRENT");
        break;

      case ' ':
        m3508.mode = MODE::SLEEP;
        m3508.target.angle = 0.0;
        m3508.target.speed = 0.0;
        m3508.target.current = 0.0;
        m3508.resetIntegral();
        Serial.println("# *** STOP ***");
        break;

      case 'f':
      case 'F':
        fineMode = !fineMode;
        Serial.print("# step mode: ");
        Serial.println(fineMode ? "FINE" : "COARSE");
        break;

      // Reset angle reference: current output position becomes "0 deg"
      case 'r':
      case 'R':
        m3508.resetAngle(0.0);
        m3508.target.angle = 0.0;
        Serial.println("# angle zeroed at current position");
        break;

      case 'q': m3508.target.angle += angleStep; break;
      case 'a': m3508.target.angle  = 0.0;       break;
      case 'z': m3508.target.angle -= angleStep; break;

      case 'w': m3508.target.speed += speedStep; break;
      case 's': m3508.target.speed  = 0.0;       break;
      case 'x': m3508.target.speed -= speedStep; break;

      case 'e': m3508.target.current += currentStep; break;
      case 'd': m3508.target.current  = 0.0;         break;
      case 'c': m3508.target.current -= currentStep; break;

      case 'h':
      case 'H':
      case '?':
        printHelp();
        printHeader();
        break;

      default: break;
    }
  }

  if ((printCount % 20) == 0) printHeader();
  printCount++;

  Serial.printf("%-6s %s | tgt: a=%8.2f s=%7.2f i=%6.2f | act: a=%8.2f s=%7.2f i=%6.2f\n",
    modeName(m3508.mode),
    fineMode ? "F" : "C",
    m3508.target.angle,  m3508.target.speed,  m3508.target.current,
    m3508.getAngle(),    m3508.getSpeed(),    m3508.getCurrent());

  flag.serialIO = false;
}

const char* modeName(MODE m) {
  switch (m) {
    case MODE::SLEEP:   return "SLEEP";
    case MODE::ANGLE:   return "ANGLE";
    case MODE::SPEED:   return "SPEED";
    case MODE::CURRENT: return "CURRNT";
    default:            return "???";
  }
}

void printHeader() {
  Serial.println("#");
  Serial.println("# mode   step | target                              | actual");
  Serial.println("#             |   angle[deg]  speed[RPM]  cur[A]    |   angle[deg]  speed[RPM]  cur[A]");
}

void printHelp() {
  Serial.println();
  Serial.println("# ======================================================");
  Serial.println("# M3508 test controller");
  Serial.println("# ------------------------------------------------------");
  Serial.println("# External gear ratio: 20.0 (cycloidal)");
  Serial.println("# Internal gear ratio: 19.0 (M3508 planetary, auto)");
  Serial.println("# Total reduction:     380:1");
  Serial.println("# All angle/speed values are at the OUTPUT shaft.");
  Serial.println("# Max output speed ~= 23 RPM (motor free-run / 380).");
  Serial.println("# ------------------------------------------------------");
  Serial.println("# MODES");
  Serial.println("#   0 = SLEEP (safe, motor off)");
  Serial.println("#   1 = ANGLE (position control)");
  Serial.println("#   2 = SPEED (velocity control)");
  Serial.println("#   3 = CURRENT (torque / direct current)");
  Serial.println("#   SPACE = panic stop (SLEEP + zero targets)");
  Serial.println("# ");
  Serial.println("# STEP MODE");
  Serial.println("#   f = toggle COARSE <-> FINE step size");
  Serial.println("#       angle:  45 deg   / 5 deg");
  Serial.println("#       speed:  5 RPM    / 0.5 RPM");
  Serial.println("#       cur:    0.5 A    / 0.05 A");
  Serial.println("# ");
  Serial.println("# TARGETS");
  Serial.println("#   q / a / z  : angle   +step / zero / -step");
  Serial.println("#   w / s / x  : speed   +step / zero / -step");
  Serial.println("#   e / d / c  : current +step / zero / -step");
  Serial.println("# ");
  Serial.println("#   r          : reset angle (current position = 0 deg)");
  Serial.println("#   h / ?      : show this help");
  Serial.println("# ======================================================");
  Serial.println();
}

bool recvMotorFlag(struct repeating_timer *t) {
  flag.recvMotor = true;
  return true;
}

bool sendMotorFlag(struct repeating_timer *t) {
  flag.sendMotor = true;
  return true;
}

bool serialIOFlag(struct repeating_timer *t) {
  flag.serialIO = true;
  return true;
}