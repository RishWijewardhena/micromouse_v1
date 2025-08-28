#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ==== TCA9548A I2C MUX ====
#define TCA_ADDR 0x70
static inline void tcaSelect(uint8_t ch) {
  if (ch > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

// ==== OLED (SPI, SSD1306 128x64) ====
#define OLED_MOSI  23
#define OLED_CLK   18
#define OLED_DC    19
#define OLED_CS    5
#define OLED_RESET 4
Adafruit_SSD1306 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS);

// ==== ToF sensors (VL53L0X) on mux ch 2=L, 1=C, 0=R ====
Adafruit_VL53L0X tofL, tofC, tofR;

// ==== MPU6050 (on mux ch 3) ====
Adafruit_MPU6050 mpu;

// ==== Motor pins (your mapping) ====
#define IN1_PIN    14  // Left direction 1
#define IN2_PIN    27  // Left direction 2
#define ENA_PIN    26  // Left PWM (analogWrite)
#define IN3_PIN    25  // Right direction 1
#define IN4_PIN    33  // Right direction 2
#define ENB_PIN    32  // Right PWM (analogWrite)

// ==== Encoder pins (input-only on ESP32) ====
#define ENC_L_A 35
#define ENC_L_B 34
#define ENC_R_A 39
#define ENC_R_B 36

// ==== Shared state (volatile for ISR/task safety) ====
volatile long ticksL = 0;
volatile long ticksR = 0;
volatile uint8_t lastStateL = 0;
volatile uint8_t lastStateR = 0;

volatile int distL = -1, distC = -1, distR = -1; // mm
volatile float yawDeg = 0.0f;

// Update flags
volatile bool sensorsUpdated = false;

// ==== Timing helpers ====
unsigned long lastYawMs = 0;

// ==== Forward declarations ====
void IRAM_ATTR isrEncL();
void IRAM_ATTR isrEncR();
void SensorTask(void *param);
void DisplayTask(void *param);

// ===== Motor control via analogWrite (0–255) =====
static inline void setLeft(int spd) {
  if (spd >= 0) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW); }
  else          { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); spd = -spd; }
  spd = constrain(spd, 0, 255);
  analogWrite(ENA_PIN, spd);
}

static inline void setRight(int spd) {
  if (spd >= 0) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW); }
  else          { digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); spd = -spd; }
  spd = constrain(spd, 0, 255);
  analogWrite(ENB_PIN, spd);
}

void drive(int left, int right) { setLeft(left); setRight(right); }
void forward(int spd)   { drive(spd, spd); }
void backward(int spd)  { drive(-spd, -spd); }
void turnLeft(int spd)  { drive(-spd, spd); }
void turnRight(int spd) { drive(spd, -spd); }
void stopMotors()       { drive(0, 0); }

// ==== Quadrature decoder (compact state table) ====
static inline int8_t quadDelta(uint8_t prev, uint8_t curr) {
  // Map 2-bit states to 0..3
  // Valid transitions: 00->01->11->10->00 (CW) and reverse for CCW
  int8_t diff = (int8_t)((prev & 0x3) << 2 | (curr & 0x3));
  // Lookup table: -1, +1, etc. 16 entries
  // Index = (prev<<2) | curr
  static const int8_t lut[16] = {
    0,  +1,  -1,  0,
    -1, 0,   0,  +1,
    +1, 0,   0,  -1,
    0,  -1,  +1,  0
  };
  return lut[diff & 0xF];
}

void IRAM_ATTR isrEncL() {
  uint8_t a = digitalRead(ENC_L_A);
  uint8_t b = digitalRead(ENC_L_B);
  uint8_t curr = (a << 1) | b;
  int8_t d = quadDelta(lastStateL, curr);
  lastStateL = curr;
  ticksL += d;
}

void IRAM_ATTR isrEncR() {
  uint8_t a = digitalRead(ENC_R_A);
  uint8_t b = digitalRead(ENC_R_B);
  uint8_t curr = (a << 1) | b;
  int8_t d = quadDelta(lastStateR, curr);
  lastStateR = curr;
  ticksR += d;
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\n=== ESP32 Robot: ToF + MPU + Encoders + OLED ===");

  // I2C @ 100 kHz (all sensors on this bus + mux)
  Wire.begin();              // ESP32 defaults SDA=21, SCL=22
  Wire.setClock(100000);     // 100 kHz as requested

  // Motors
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  // PWM pins: analogWrite is supported by ESP32 Arduino core
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  stopMotors();

  // Encoders
  pinMode(ENC_L_A, INPUT);
  pinMode(ENC_L_B, INPUT);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);
  lastStateL = ((uint8_t)digitalRead(ENC_L_A) << 1) | (uint8_t)digitalRead(ENC_L_B);
  lastStateR = ((uint8_t)digitalRead(ENC_R_A) << 1) | (uint8_t)digitalRead(ENC_R_B);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrEncR, CHANGE);

  // OLED (hardware SPI)
  SPI.begin(OLED_CLK, -1, OLED_MOSI);
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1) { delay(1000); }
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Initializing..."));
  display.display();

  // ToF sensors (Left ch2, Center ch1, Right ch0)
  tcaSelect(2); if (!tofL.begin()) Serial.println("Left ToF FAILED"); else Serial.println("Left ToF OK");
  tcaSelect(1); if (!tofC.begin()) Serial.println("Center ToF FAILED"); else Serial.println("Center ToF OK");
  tcaSelect(0); if (!tofR.begin()) Serial.println("Right ToF FAILED"); else Serial.println("Right ToF OK");

  // MPU6050 on mux ch3
  tcaSelect(3);
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("MPU6050 not found!");
    display.println(F("MPU FAIL"));
    display.display();
    while (1) { delay(1000); }
  }
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  lastYawMs = millis();
  Serial.println("MPU6050 OK");

  // Start tasks
  xTaskCreatePinnedToCore(SensorTask,  "SensorTask",  8192, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 6144, nullptr, 1, nullptr, 1);

  Serial.println("Setup complete.");
}


void loop() {
  static uint8_t state = 0; // 0=fwd, 1=back, 2=left, 3=right
  static unsigned long actionStart = 0;
  static bool actionRunning = false;

  // Local copies of encoders & yaw
  long tl, tr;
  float y;
  noInterrupts();
  tl = ticksL;
  tr = ticksR;
  y = yawDeg;
  interrupts();

  if (!actionRunning) {
    // Start movement for current state
    switch (state) {
      case 0: forward(180);  break;
      case 1: backward(180); break;
      case 2: turnLeft(160); break;
      case 3: turnRight(160); break;
    }
    actionStart = millis();
    actionRunning = true;
  } else {
    // Stop after 1 second, move to next state
    if (millis() - actionStart > 1000) {
      stopMotors();
      actionRunning = false;
      state = (state + 1) % 4; // 0→1→2→3→0
      delay(300); // pause between moves
    }
  }

  // Print status every 100ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    Serial.printf("ticksL:%ld ticksR:%ld | Yaw: %.1f | State:%d\n",
                  tl, tr, y, state);
    lastPrint = millis();
  }

  delay(2);
}

// ====== Sensor Task: reads ToF + integrates yaw from gyro ======
void SensorTask(void *param) {
  VL53L0X_RangingMeasurementData_t m;
  unsigned long prevMs = millis();

  for (;;) {
    // --- Left ToF (ch 2)
    tcaSelect(2);
    if (tofL.rangingTest(&m, false), m.RangeStatus != 4) distL = m.RangeMilliMeter; else distL = -1;

    // --- Center ToF (ch 1)
    tcaSelect(1);
    if (tofC.rangingTest(&m, false), m.RangeStatus != 4) distC = m.RangeMilliMeter; else distC = -1;

    // --- Right ToF (ch 0)
    tcaSelect(0);
    if (tofR.rangingTest(&m, false), m.RangeStatus != 4) distR = m.RangeMilliMeter; else distR = -1;

    // --- MPU yaw integration (ch 3)
    tcaSelect(3);
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    unsigned long nowMs = millis();
    float dt = (nowMs - lastYawMs) / 1000.0f;
    lastYawMs = nowMs;

    // g.gyro.z in rad/s → deg/s
    float yawRateDeg = g.gyro.z * (180.0f / PI);
    float y = yawDeg + yawRateDeg * dt;

    // wrap to [-180, 180]
    if (y > 180.0f) y -= 360.0f;
    if (y < -180.0f) y += 360.0f;
    yawDeg = y;

    sensorsUpdated = true;

    vTaskDelay(pdMS_TO_TICKS(10)); // ~100 Hz loop, non-blocking
  }
}

// ====== Display Task: ~20 FPS, non-blocking ======
void DisplayTask(void *param) {
  int lastDL = -999, lastDC = -999, lastDR = -999;
  long lastTL = LONG_MIN, lastTR = LONG_MIN;
  float lastYaw = 9999.0f;
  unsigned long lastRefresh = 0;

  for (;;) {
    // limit to ~20 fps
    if (millis() - lastRefresh < 50) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    lastRefresh = millis();

    // Snapshot shared variables
    int l, c, r; long tl, tr; float y;
    noInterrupts();
    l = distL; c = distC; r = distR;
    tl = ticksL; tr = ticksR;
    y = yawDeg;
    interrupts();

    bool changed =
      (l != lastDL) || (c != lastDC) || (r != lastDR) ||
      (tl != lastTL) || (tr != lastTR) || (fabsf(y - lastYaw) > 0.5f);

    if (changed) {
      lastDL = l; lastDC = c; lastDR = r;
      lastTL = tl; lastTR = tr; lastYaw = y;

      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println(F("ToF (mm)  Yaw & Enc"));

      display.print(F("L: ")); display.println((l >= 0) ? String(l) : F("---"));
      display.print(F("C: ")); display.println((c >= 0) ? String(c) : F("---"));
      display.print(F("R: ")); display.println((r >= 0) ? String(r) : F("---"));

      display.print(F("Yaw: "));
      display.println(String(y, 1));

      display.print(F("EncL: "));
      display.println(String(tl));
      display.print(F("EncR: "));
      display.println(String(tr));

      display.display();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
