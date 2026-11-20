#include "Wire.h"
#include "math.h"
#include <stdint.h>
#include <string.h>
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// Both clocks are 400 kHz: the library otherwise drops the bus to 100 kHz after each transfer.
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 400000UL, 400000UL);

// Plant register map: keep in sync with chip/chip.chip.c.
#define PLANT_ADDR   0x42
#define REG_Q_TRUE   0x30 // R, 16  ground truth -- SCORING ONLY, never in the control path
#define REG_TORQUE   0x50 // W, 12
#define REG_IMU_DATA 0x70 // R, 24  accel x,y,z then gyro x,y,z

struct Quat { float w, x, y, z; };
struct Vec3 { float x, y, z; };

// --- Reciprocal square root ---
// Normalises the accelerometer direction once per loop. On the FPU-less AVR the
// bit trick is faster (730 vs 979 cycles); on any FPU target, use 1.0f/sqrtf.
#if defined(__AVR__)
static inline float invSqrt(float x) {
    // Quake III initial guess plus one Newton step: ~1.75e-3 worst-case error, fine
    // for a direction, not for q_est. memcpy avoids undefined type punning.
  float halfx = 0.5f * x;
  uint32_t i;
  memcpy(&i, &x, sizeof i);
  i = 0x5f3759dfUL - (i >> 1);
  float y;
  memcpy(&y, &i, sizeof y);
  return y * (1.5f - halfx * y * y);
}
#else
static inline float invSqrt(float x) {
  return 1.0f / sqrtf(x);
}
#endif

Quat quat_mul(const Quat& q, const Quat& p) {
  return {
    q.w*p.w - q.x*p.x - q.y*p.y - q.z*p.z,
    q.w*p.x + q.x*p.w + q.y*p.z - q.z*p.y,
    q.w*p.y - q.x*p.z + q.y*p.w + q.z*p.x,
    q.w*p.z + q.x*p.y - q.y*p.x + q.z*p.w
  };
}

Quat quat_conj(const Quat& q) {
  return { q.w, -q.x, -q.y, -q.z };
}

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

Quat quat_from_euler(float roll, float pitch, float yaw) {
  float cr = cos(roll * 0.5f);
  float sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f);
  float sp = sin(pitch * 0.5f);
  float cy = cos(yaw * 0.5f);
  float sy = sin(yaw * 0.5f);
  return {
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy
  };
}

const float Kp_base[3] = {18.0f, 36.0f, 54.0f}; 
const float Kd_base[3] = {4.2f,  8.4f,  12.6f}; 

// State Estimation Globals
Quat q_est = {1.0f, 0.0f, 0.0f, 0.0f}; 
uint32_t last_micros = 0;
uint32_t last_frame_millis = 0;
uint32_t last_telemetry_millis = 0;
uint32_t max_dt_us = 0;
float Kp_imu = 2.5f; 

// Mahony integral term: the gyro-bias estimate, rad/s. Ki_imu = Kp_imu^2 / 4
// critically damps the tilt error. Yaw bias is unobservable without a magnetometer.
Vec3 gyro_bias = {0.0f, 0.0f, 0.0f};
float Ki_imu = 1.5625f;

// --- OLED streaming ---
// A full display() stalls the loop ~25 ms, so the frame goes out in two chunks per loop.
const uint16_t OLED_BYTES = SCREEN_WIDTH * SCREEN_HEIGHT / 8;
const uint8_t OLED_CHUNK = 31;    // 32-byte AVR Wire buffer, less the 0x40 prefix
const uint8_t OLED_CHUNKS_PER_LOOP = 2;
uint16_t oled_sent = OLED_BYTES;  // == OLED_BYTES: no frame in flight

void oled_start_frame() {
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x00);  // command stream
  Wire.write(SSD1306_PAGEADDR);
  Wire.write(0);
  Wire.write(SCREEN_HEIGHT / 8 - 1);
  Wire.write(SSD1306_COLUMNADDR);
  Wire.write(0);
  Wire.write(SCREEN_WIDTH - 1);
  Wire.endTransmission();
  oled_sent = 0;
}

void oled_send_chunk() {
  uint16_t n = OLED_BYTES - oled_sent;
  if (n > OLED_CHUNK) n = OLED_CHUNK;
  Wire.beginTransmission(OLED_ADDR);
  Wire.write(0x40);  // data stream
  Wire.write(display.getBuffer() + oled_sent, n);
  Wire.endTransmission();
  oled_sent += n;
}

void render_horizon() {
  float roll  = atan2(2.0f * (q_est.w * q_est.x + q_est.y * q_est.z), 1.0f - 2.0f * (q_est.x * q_est.x + q_est.y * q_est.y));
    // Clamp: rounding can push the argument past +/-1, and asin would return NaN.
  float pitch = asin(clampf(2.0f * (q_est.w * q_est.y - q_est.z * q_est.x), -1.0f, 1.0f));

  display.clearDisplay();
  display.drawFastHLine(64 - 15, 32, 10, SSD1306_WHITE); 
  display.drawFastHLine(64 + 5,  32, 10, SSD1306_WHITE); 
  display.drawPixel(64, 32, SSD1306_WHITE);              

  float pixels_per_rad = 40.0f; 
  float pitch_offset = pitch * pixels_per_rad;
  
  int r = 100; 
  int x0 = 64 - r * cos(roll);
  int y0 = 32 + pitch_offset + r * sin(roll);
  int x1 = 64 + r * cos(roll);
  int y1 = 32 + pitch_offset - r * sin(roll);

  display.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
}

// Fixed-point printing: Serial.print(float) costs ~8,800 cycles per value on the AVR.
void print_fixed4(float v) {
  static const int16_t place[4] = {1000, 100, 10, 1};
  int16_t n = lround(v * 10000.0f);
  char s[8], *p = s;
  if (n < 0) { *p++ = '-'; n = -n; }
  *p++ = n >= 10000 ? '1' : '0';
  if (n >= 10000) n -= 10000;
  *p++ = '.';
  for (uint8_t k = 0; k < 4; k++) {  // digits by subtraction: no 16-bit divides
    char c = '0';
    while (n >= place[k]) { n -= place[k]; c++; }
    *p++ = c;
  }
  *p = 0;
  Serial.print(s);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  
  pinMode(A0, INPUT); 
  pinMode(A1, INPUT); 
  pinMode(A2, INPUT); 
  
  display.clearDisplay();
  display.display();
  delay(100); 
  
  last_micros = micros();
  last_frame_millis = millis();
  last_telemetry_millis = millis();
}

void loop() {
  uint32_t now = micros();

    // --- 1. SENSOR READING ---
    // One transaction for both sensors, so accel and gyro come from the same plant tick.
  Vec3 accel, gyro;
  Wire.beginTransmission(PLANT_ADDR);
  Wire.write(REG_IMU_DATA);
  Wire.endTransmission(false);
  if (Wire.requestFrom(PLANT_ADDR, 24) == 24) {
    Wire.readBytes((uint8_t*)&accel, 12);
    Wire.readBytes((uint8_t*)&gyro, 12);
  } else {
        // Short read: skip this cycle and keep last_micros, so the next good sample
        // integrates across the gap.
    return;
  }

  uint32_t dt_us = now - last_micros;
  last_micros = now;
  if (dt_us > max_dt_us) max_dt_us = dt_us;
  float dt = dt_us / 1000000.0f;

    // --- 2. MAHONY SENSOR FUSION ---
    // The filter corrects `gyro` in place; the PD law needs the raw rate, so keep a copy.
  const Vec3 gyro_raw = gyro;

  float accel_sq = accel.x*accel.x + accel.y*accel.y + accel.z*accel.z;
  if (accel_sq > 0.0f) {
    // Fast normalization using multiplication
    float a_norm_inv = invSqrt(accel_sq);
    accel.x *= a_norm_inv; 
    accel.y *= a_norm_inv; 
    accel.z *= a_norm_inv;
    
    Vec3 g_est = {
      2.0f * (q_est.x * q_est.z - q_est.w * q_est.y),
      2.0f * (q_est.w * q_est.x + q_est.y * q_est.z),
      q_est.w * q_est.w - q_est.x * q_est.x - q_est.y * q_est.y + q_est.z * q_est.z
    };

    Vec3 error = {
      accel.y * g_est.z - accel.z * g_est.y,
      accel.z * g_est.x - accel.x * g_est.z,
      accel.x * g_est.y - accel.y * g_est.x
    };

    float ki_dt = Ki_imu * dt;
    gyro_bias.x -= ki_dt * error.x;
    gyro_bias.y -= ki_dt * error.y;
    gyro_bias.z -= ki_dt * error.z;

    gyro.x += Kp_imu * error.x;
    gyro.y += Kp_imu * error.y;
    gyro.z += Kp_imu * error.z;
  }
  gyro.x -= gyro_bias.x;
  gyro.y -= gyro_bias.y;
  gyro.z -= gyro_bias.z;

  // Integrate
  Quat q_dot = {
    -0.5f * (q_est.x * gyro.x + q_est.y * gyro.y + q_est.z * gyro.z),
     0.5f * (q_est.w * gyro.x + q_est.y * gyro.z - q_est.z * gyro.y),
     0.5f * (q_est.w * gyro.y - q_est.x * gyro.z + q_est.z * gyro.x),
     0.5f * (q_est.w * gyro.z + q_est.x * gyro.y - q_est.y * gyro.x)
  };

  q_est.w += q_dot.w * dt;
  q_est.x += q_dot.x * dt;
  q_est.y += q_dot.y * dt;
  q_est.z += q_dot.z * dt;

    // Normalise with 1/sqrtf: one Newton step would leave a ~1.7e-3 bias that compounds.
  float q_norm_inv = 1.0f / sqrtf(q_est.w*q_est.w + q_est.x*q_est.x + q_est.y*q_est.y + q_est.z*q_est.z);
  q_est.w *= q_norm_inv; 
  q_est.x *= q_norm_inv; 
  q_est.y *= q_norm_inv; 
  q_est.z *= q_norm_inv;

  // --- 3. PILOT INPUT ---
  float joy_roll =  ((analogRead(A0) - 512) / 512.0f) * (PI / 4.0f);
  float joy_pitch = ((analogRead(A1) - 512) / 512.0f) * (PI / 4.0f);
  Quat q_cmd = quat_from_euler(joy_roll, joy_pitch, 0.0f);

  float gain_multiplier = 0.1f + (analogRead(A2) / 1023.0f) * 2.9f;

  // --- 4. PD CONTROL LAW ---
  Quat q_est_inv = quat_conj(q_est);
  Quat q_err = quat_mul(q_est_inv, q_cmd);
  // q and -q are the same attitude: take the short way round.
  float sign_w = (q_err.w >= 0.0f) ? 1.0f : -1.0f;

    // Derivative term: raw gyro minus the bias estimate. The filter-corrected rate
    // would feed accelerometer noise into the torque.
  Vec3 tau;
  tau.x = (Kp_base[0] * gain_multiplier) * sign_w * q_err.x - (Kd_base[0] * gain_multiplier) * (gyro_raw.x - gyro_bias.x);
  tau.y = (Kp_base[1] * gain_multiplier) * sign_w * q_err.y - (Kd_base[1] * gain_multiplier) * (gyro_raw.y - gyro_bias.y);
  tau.z = (Kp_base[2] * gain_multiplier) * sign_w * q_err.z - (Kd_base[2] * gain_multiplier) * (gyro_raw.z - gyro_bias.z);

  Wire.beginTransmission(PLANT_ADDR);
  Wire.write(REG_TORQUE);
  Wire.write((uint8_t*)&tau, 12);
  Wire.endTransmission();

  // --- 5. UI: stream the framebuffer, new frame every >= 33 ms ---
  bool rendered = false;
  if (oled_sent < OLED_BYTES) {
    for (uint8_t i = 0; i < OLED_CHUNKS_PER_LOOP && oled_sent < OLED_BYTES; i++) oled_send_chunk();
  } else if (millis() - last_frame_millis >= 33) {
    last_frame_millis = millis();
    render_horizon();
    oled_start_frame();
    rendered = true;
  }

  // --- 6. TELEMETRY, ~30 Hz, never in the same iteration as a render ---
  if (!rendered && millis() - last_telemetry_millis >= 33) {
    last_telemetry_millis = millis();

        // --- GROUND TRUTH: SCORING ONLY ---
        // Read after the torque command, into a local, so the control path cannot use it.
    Quat q_true = {1.0f, 0.0f, 0.0f, 0.0f};
    bool truth_ok = false;
    Wire.beginTransmission(PLANT_ADDR);
    Wire.write(REG_Q_TRUE);
    Wire.endTransmission(false);
    if (Wire.requestFrom(PLANT_ADDR, 16) == 16) {
      Wire.readBytes((uint8_t*)&q_true, 16);
      truth_ok = true;
    }

        // Columns: q_est.w,q_est.x,q_est.y,q_est.z,q_true.w,q_true.x,q_true.y,q_true.z,max_dt_us
        // max_dt_us is the worst loop period since the previous line.
    print_fixed4(q_est.w); Serial.print(',');
    print_fixed4(q_est.x); Serial.print(',');
    print_fixed4(q_est.y); Serial.print(',');
    print_fixed4(q_est.z); Serial.print(',');
    if (truth_ok) {
      print_fixed4(q_true.w); Serial.print(',');
      print_fixed4(q_true.x); Serial.print(',');
      print_fixed4(q_true.y); Serial.print(',');
      print_fixed4(q_true.z);
    } else {
      Serial.print("nan,nan,nan,nan");
    }
    Serial.print(',');
    Serial.println(max_dt_us);
    max_dt_us = 0;
  }
}
