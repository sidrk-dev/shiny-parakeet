#include <Arduino.h>
#include <Wire.h>

/**
 * SINGLE ENCODER CONTINUOUS READER
 * XIAO RP2350 + PCA9548A + AS5600
 */

#define MUX_ADDR      0x70
#define AS5600_ADDR   0x36

#define SDA1_PIN      6
#define SCL1_PIN      7

// Pick the mux channel your encoder is connected to
#define ENCODER_CHANNEL 0

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;

  Wire1.beginTransmission(MUX_ADDR);
  Wire1.write(1 << channel);
  uint8_t error = Wire1.endTransmission();

  if (error != 0) {
    Serial.print("MUX error: ");
    Serial.println(error);
  }
}

uint16_t readAS5600RawAngle() {
  // Use standard stop-start sequence (true) instead of repeated start (false) for better stability on noisy lines
  Wire1.beginTransmission(AS5600_ADDR);
  Wire1.write(0x0C);                 // RAW ANGLE register
  uint8_t error = Wire1.endTransmission(true); 
  
  if (error != 0) {
    // 2: NACK on address, 3: NACK on data, 4: other error
    return 0xE000 | error; 
  }

  if (Wire1.requestFrom(AS5600_ADDR, 2) != 2) {
    return 0xFFFF;
  }

  uint16_t raw = (Wire1.read() << 8) | Wire1.read();
  raw &= 0x0FFF;                     // 12-bit value
  return raw;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Wire1.setSDA(SDA1_PIN);
  Wire1.setSCL(SCL1_PIN);
  Wire1.begin();
  Wire1.setClock(400000); // Reduced to 100kHz for stability

  Serial.println("Encoder Test Started (100kHz)");

  // Verify mux exists
  Wire1.beginTransmission(MUX_ADDR);
  if (Wire1.endTransmission() != 0) {
    Serial.println("PCA9548A not found at 0x70");
    while (1) delay(100);
  }
}

void loop() {
  // Re-select channel every time to ensure mux state isn't lost
  selectMuxChannel(ENCODER_CHANNEL);
  
  uint16_t raw = readAS5600RawAngle();

  if (raw == 0xFFFF) {
    Serial.println("Read fail: Timeout/Incomplete");
  } else if ((raw & 0xE000) == 0xE000) {
    Serial.print("Read fail: I2C Error ");
    Serial.println(raw & 0x000F);
  } else {
    float degrees = (raw * 360.0f) / 4096.0f;
    Serial.print("Angle: ");
    Serial.print(degrees, 2);
    Serial.print(" | Raw: ");
    Serial.println(raw);
  }

  delay(50); // Slower update for debugging
}