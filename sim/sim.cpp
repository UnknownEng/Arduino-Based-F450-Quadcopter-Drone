// Software-in-the-loop simulator for flight_controller.ino.
//
// The real, unmodified flight controller code is compiled into this program. It talks to a fake Arduino
// (arduino_sim.h) whose clock, radio receiver, MPU-6050 and ESC pins are connected to a physics model of
// an F450 quadcopter. Each scenario flies the quad and checks the result.
//
// Usage: sim <scenario> [csv_file]      scenarios: hover failsafe_cut failsafe_descent gyro_fail
//                                                  ground_windup rx_glitch
#include "arduino_sim.h"
// The Arduino IDE generates these prototypes automatically; a normal compiler needs them written out.
void set_gyro_registers();
void gyro_signalen();
void calculate_pid();
int convert_receiver_channel(byte function);
#include "../flight_controller/flight_controller.ino"
#undef int
#undef double

#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include <algorithm>

//==================================================================================================
// Fake Arduino state
//==================================================================================================
PortDRegister PORTD;
volatile uint8_t PINB = 0;
uint8_t DDRD, DDRB, PCICR, PCMSK0, TWBR;
TwoWire Wire;
EEPROMClass EEPROM;

namespace sim {

// Start the clock 25 s before the 32-bit micros() counter wraps, so every flight crosses the wrap.
const uint64_t START_TIME_US = 4294967296ULL - 25000000ULL;
uint64_t now_us = START_TIME_US;
bool in_isr = false;

//------------------------------ F450 model parameters ---------------------------------------------
// These can be changed from the command line (e.g. "sim robust mass=1.5 tmax=7") to test other builds.
double MASS = 1.2;                  // kg
const double G = 9.81;
const double ARM = 0.159;           // m, motor arm projected on the roll/pitch axes (225 mm * sin 45)
double I_ROLL = 0.0125, I_PITCH = 0.0125, I_YAW = 0.023;  // kg m^2
double T_MAX = 8.0;                 // N per motor at 2000 us
const double K_YAW = 0.016;         // m, reaction torque per newton of thrust
double MOTOR_TAU = 0.04;            // s, motor/prop spin-up time constant
double NOISE_MULT = 1.0;            // vibration / sensor noise multiplier (1 = balanced props)
double GUST_MULT = 1.0;             // wind gust multiplier
double BATTERY_V = 11.8;            // battery voltage seen by the flight controller
const double DRAG = 0.35;           // N per m/s, translational drag
const double ROT_DAMP = 0.004;      // N m per rad/s, rotational air damping
const double PHYS_DT_US = 200;      // physics step

//------------------------------ State -------------------------------------------------------------
double roll = 0, pitch = 0, yaw = 0;        // deg. roll + = right side down, pitch + = nose up, yaw + = nose right
double p = 0, q = 0, r = 0;                 // deg/s body rates, same signs
double px = 0, py = 0, pz = 0;              // m, world position (x forward, y left, z up)
double vx = 0, vy = 0, vz = 0;
double ax_w = 0, ay_w = 0, az_w = 0;        // last world acceleration
double motor[5];                            // 0..1 spin level, index 1-4 = ESC 1-4
bool on_ground = true;
double ground_roll = 0, ground_pitch = 0;   // tilt of the ground the quad sits on
bool flipped = false;
double gust_roll = 0, gust_pitch = 0, gust_yaw = 0;
double gust_strength = 0;                   // N m (standard deviation)
uint64_t last_phys = START_TIME_US;

//------------------------------ ESC pulse measurement --------------------------------------------
uint64_t pulse_rise[5];
int esc_pulse[5] = {0, 1000, 1000, 1000, 1000};   // last measured pulse width per ESC, us
uint64_t last_pulse_time[5];
int max_pulse_seen_disarmed = 0;

//------------------------------ Radio receiver ----------------------------------------------------
int stick[5] = {0, 1500, 1500, 1000, 1500};       // us: 1 roll, 2 pitch, 3 throttle, 4 yaw
int frame_width[5] = {0, 1500, 1500, 1000, 1500};
bool rx_on = true;
bool rx_drop_frame = false;                       // drop the next frame only
bool rx_glitches = false;                         // inject short noise spikes
bool dropping_this_frame = false;
uint64_t frame_start = 0;

//------------------------------ MPU-6050 ----------------------------------------------------------
std::mt19937 rng(12345);
std::normal_distribution<double> gauss(0.0, 1.0);
const double GYRO_BIAS[4] = {0, 41, -27, 12};     // raw LSB, removed by the gyro calibration
double gyro_f[4], acc_f[4];                       // sensor values after the MPU's 43 Hz low-pass filter
bool i2c_fail = false;
uint8_t mpu_reg_ptr = 0;
uint8_t mpu_regs[256];
uint8_t i2c_buf[16];
int i2c_len = 0, i2c_idx = 0, i2c_written = 0;

bool led = false;
int analog_battery = 893;                         // ~11.8 V on a 3S pack

//------------------------------ Helpers -----------------------------------------------------------
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
double t_s() { return (now_us - START_TIME_US) / 1e6; }
const double D2R = M_PI / 180.0;

// Thrust direction and body axes in the world frame (x forward, y left, z up).
void body_axes(double bx[3], double by[3], double bz[3]) {
  double ph = roll * D2R, th = pitch * D2R;
  bz[0] = -std::sin(th) * std::cos(ph);
  bz[1] = -std::sin(ph);
  bz[2] = std::cos(th) * std::cos(ph);
  double n = std::sqrt(bz[0]*bz[0] + bz[1]*bz[1] + bz[2]*bz[2]);
  for (int i = 0; i < 3; i++) bz[i] /= n;
  double x[3] = {std::cos(th), 0, std::sin(th)};
  double d = x[0]*bz[0] + x[1]*bz[1] + x[2]*bz[2];
  for (int i = 0; i < 3; i++) x[i] -= d * bz[i];
  n = std::sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
  for (int i = 0; i < 3; i++) bx[i] = x[i] / n;
  by[0] = bz[1]*bx[2] - bz[2]*bx[1];
  by[1] = bz[2]*bx[0] - bz[0]*bx[2];
  by[2] = bz[0]*bx[1] - bz[1]*bx[0];
}

double avg_motor() { return (motor[1] + motor[2] + motor[3] + motor[4]) / 4.0; }

void physics_step(double dt) {
  // ESCs: a pulse older than 50 ms means no signal -> the ESC stops the motor.
  for (int i = 1; i <= 4; i++) {
    double cmd = clampd((esc_pulse[i] - 1000) / 1000.0, 0, 1);
    if (now_us - last_pulse_time[i] > 50000) cmd = 0;
    motor[i] += (cmd - motor[i]) * (dt / MOTOR_TAU);
  }
  double F[5];
  for (int i = 1; i <= 4; i++) F[i] = T_MAX * motor[i];
  double thrust = F[1] + F[2] + F[3] + F[4];

  // Gusts: slowly varying random torques.
  double a = dt / 0.3;
  double gs = gust_strength * GUST_MULT;
  gust_roll  += (-gust_roll)  * a + gs * std::sqrt(2 * a) * gauss(rng);
  gust_pitch += (-gust_pitch) * a + gs * std::sqrt(2 * a) * gauss(rng);
  gust_yaw   += (-gust_yaw)   * a + 0.3 * gs * std::sqrt(2 * a) * gauss(rng);

  // Motor layout: 1 front-right CCW, 2 rear-right CW, 3 rear-left CCW, 4 front-left CW.
  double tau_roll  = ARM * (F[3] + F[4] - F[1] - F[2]) + gust_roll;    // left side up -> right side down (+)
  double tau_pitch = ARM * (F[1] + F[4] - F[2] - F[3]) + gust_pitch;   // front up -> nose up (+)
  double tau_yaw   = K_YAW * (F[1] + F[3] - F[2] - F[4]) + gust_yaw;   // CCW props push the frame nose right (+)

  double bx[3], by[3], bz[3];
  body_axes(bx, by, bz);
  double fz_world = thrust * bz[2] - MASS * G;

  if (on_ground && fz_world > 0.0) on_ground = false;                  // lift off

  if (on_ground) {
    roll = ground_roll; pitch = ground_pitch;
    p = q = 0;
    // Yaw on the ground is held by friction unless the yaw torque is large.
    if (std::fabs(tau_yaw) < 0.15) r = 0;
    else { r += (tau_yaw / I_YAW) / D2R * dt; yaw += r * dt; }
    vx = vy = vz = 0; ax_w = ay_w = az_w = 0;
    return;
  }

  p += ((tau_roll  - ROT_DAMP * p * D2R) / I_ROLL)  / D2R * dt;
  q += ((tau_pitch - ROT_DAMP * q * D2R) / I_PITCH) / D2R * dt;
  r += ((tau_yaw   - ROT_DAMP * r * D2R) / I_YAW)   / D2R * dt;
  roll += p * dt; pitch += q * dt; yaw += r * dt;

  ax_w = (thrust * bz[0] - DRAG * vx) / MASS;
  ay_w = (thrust * bz[1] - DRAG * vy) / MASS;
  az_w = (thrust * bz[2] - DRAG * vz) / MASS - G;
  vx += ax_w * dt; vy += ay_w * dt; vz += az_w * dt;
  px += vx * dt; py += vy * dt; pz += vz * dt;

  if (std::fabs(roll) > 70 || std::fabs(pitch) > 70) flipped = true;

  if (pz <= 0) {                                                       // touchdown (or crash)
    pz = 0; on_ground = true;
    ground_roll = 0; ground_pitch = 0; roll = 0; pitch = 0;
    p = q = r = 0; vx = vy = vz = 0;
  }
}

void sensor_step(double dt) {
  // What the MPU-6050 measures: body rates and specific force (acceleration minus gravity).
  double bx[3], by[3], bz[3];
  body_axes(bx, by, bz);
  double s[3] = {ax_w, ay_w, az_w + G};
  double s_x = s[0]*bx[0] + s[1]*bx[1] + s[2]*bx[2];
  double s_y = s[0]*by[0] + s[1]*by[1] + s[2]*by[2];
  double s_z = s[0]*bz[0] + s[1]*bz[1] + s[2]*bz[2];
  // Vibration: each motor/prop shakes the frame at its own rotation frequency (prop imbalance), roughly
  // 100-180 Hz on an F450. Plus a little broadband sensor noise. NOISE_MULT scales the imbalance.
  static double phase[5];
  static const double axis_phase[5][4] = {{0}, {0, 0.0, 1.7, 3.1}, {0, 0.9, 2.6, 4.4}, {0, 2.1, 0.4, 5.2}, {0, 3.3, 4.8, 1.2}};
  double vib_g[4] = {0, 0, 0, 0}, vib_a[4] = {0, 0, 0, 0};
  for (int m = 1; m <= 4; m++) {
    double freq = 180.0 * std::sqrt(std::max(0.0, motor[m]));          // Hz, rpm grows with sqrt(thrust)
    phase[m] += 2 * M_PI * freq * dt;
    for (int ax = 1; ax <= 3; ax++) {
      // rotation frequency + blade-pass (2 blades = 2x rotation, aliases to ~50 Hz at the 250 Hz loop rate)
      double sw = std::sin(phase[m] + axis_phase[m][ax]) + 0.6 * std::sin(2 * phase[m] + 1.3 * axis_phase[m][ax]);
      vib_g[ax] += 80.0 * NOISE_MULT * motor[m] * sw;                    // gyro LSB (65.5 LSB = 1 deg/s)
      vib_a[ax] += 600.0 * NOISE_MULT * motor[m] * sw;                   // accel LSB (4096 LSB = 1 g)
    }
  }
  double raw_g[4], raw_a[4];
  raw_g[1] = p * 65.5 + GYRO_BIAS[1] + vib_g[1] + 3.0 * gauss(rng);
  raw_g[2] = q * 65.5 + GYRO_BIAS[2] + vib_g[2] + 3.0 * gauss(rng);
  raw_g[3] = r * 65.5 + GYRO_BIAS[3] + vib_g[3] + 3.0 * gauss(rng);
  // Axis mapping that setup stored: acc axis 1 = pitch angle, acc axis 2 = -roll angle (see gyro_signalen()).
  raw_a[1] =  s_x / G * 4096 + vib_a[1] + 30.0 * gauss(rng);
  raw_a[2] = -s_y / G * 4096 + vib_a[2] + 30.0 * gauss(rng);
  raw_a[3] =  s_z / G * 4096 + vib_a[3] + 30.0 * gauss(rng);
  double k = dt / (dt + 1.0 / (2 * M_PI * 43.0));                    // MPU digital low-pass filter ~43 Hz
  for (int i = 1; i <= 3; i++) {
    gyro_f[i] += (raw_g[i] - gyro_f[i]) * k;
    acc_f[i]  += (raw_a[i] - acc_f[i]) * k;
  }
}

void receiver_tick() {
  uint64_t phase = now_us - frame_start;
  if (phase >= 20000) {                                                // new 50 Hz frame
    frame_start = now_us; phase = 0;
    for (int i = 1; i <= 4; i++) frame_width[i] = stick[i];
    dropping_this_frame = rx_drop_frame; rx_drop_frame = false;
  }
  uint8_t pins = 0;
  if (rx_on && !dropping_this_frame) {
    for (int c = 1; c <= 4; c++) {
      uint64_t s = (c - 1) * 2500;
      if (phase >= s && phase < s + frame_width[c]) pins |= (1 << (c - 1));
      // Noise spike: a 300 us pulse in the gap after each channel.
      if (rx_glitches && phase >= s + 2200 && phase < s + 2200 + 300) pins |= (1 << (c - 1));
    }
  }
  if (pins != (PINB & 0x0F)) {
    PINB = pins;
    in_isr = true;
    PCINT0_vect_handler();                                             // pin change interrupt
    in_isr = false;
  }
}

void advance(uint64_t us) {
  for (uint64_t i = 0; i < us; i++) {
    now_us++;
    receiver_tick();
    if (now_us - last_phys >= (uint64_t)PHYS_DT_US) {
      double dt = (now_us - last_phys) / 1e6;
      last_phys = now_us;
      physics_step(dt);
      sensor_step(dt);
    }
  }
}

void reset_world() {
  roll = pitch = yaw = p = q = r = 0;
  px = py = pz = vx = vy = vz = 0;
  on_ground = true; flipped = false;
  for (int i = 1; i <= 4; i++) { motor[i] = 0; gyro_f[i % 4] = 0; }
  gyro_f[1] = GYRO_BIAS[1]; gyro_f[2] = GYRO_BIAS[2]; gyro_f[3] = GYRO_BIAS[3];
  acc_f[1] = 0; acc_f[2] = 0; acc_f[3] = 4096;
}

}  // namespace sim

//==================================================================================================
// Fake Arduino API, backed by the simulator
//==================================================================================================
void sim_portd_changed(uint8_t o, uint8_t n) {
  for (int i = 1; i <= 4; i++) {
    uint8_t bit = 1 << (i + 3);                                         // ESC 1-4 = D4-D7
    if (!(o & bit) && (n & bit)) sim::pulse_rise[i] = sim::now_us;
    if ((o & bit) && !(n & bit)) {
      sim::esc_pulse[i] = (int)(sim::now_us - sim::pulse_rise[i]);
      sim::last_pulse_time[i] = sim::now_us;
    }
  }
}

unsigned long micros() { if (!sim::in_isr) sim::advance(1); return (unsigned long)(uint32_t)sim::now_us; }
unsigned long millis() { if (!sim::in_isr) sim::advance(1); return (unsigned long)(uint32_t)(sim::now_us / 1000); }
void delay(unsigned long ms) { sim::advance((uint64_t)ms * 1000); }
void delayMicroseconds(unsigned int us) { sim::advance(us); }
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t v) { if (pin == 12) sim::led = v; }
int digitalRead(uint8_t pin) { return pin == 12 ? sim::led : 0; }
int analogRead(uint8_t) { return sim::analog_battery; }

void TwoWire::begin() {}
void TwoWire::setWireTimeout(uint32_t, bool) {}
void TwoWire::beginTransmission(int) { sim::i2c_written = 0; }
size_t TwoWire::write(uint8_t d) {
  if (sim::i2c_written == 0) sim::mpu_reg_ptr = d;
  else sim::mpu_regs[sim::mpu_reg_ptr++] = d;
  sim::i2c_written++;
  return 1;
}
uint8_t TwoWire::endTransmission() {
  if (sim::i2c_fail) { sim::advance(1000); return 5; }                  // 5 = timeout
  sim::advance(25 * (sim::i2c_written + 1));
  return 0;
}
uint8_t TwoWire::requestFrom(int, int quantity) {
  if (sim::i2c_fail) { sim::advance(1000); sim::i2c_len = 0; sim::i2c_idx = 0; return 0; }
  sim::advance(23 * (quantity + 2));                                    // 400 kHz bus time
  auto put16 = [](int idx, double v) {
    int16_t x = (int16_t)sim::clampd(std::lround(v), -32768, 32767);
    sim::i2c_buf[idx] = (uint8_t)((uint16_t)x >> 8);
    sim::i2c_buf[idx + 1] = (uint8_t)(x & 0xFF);
  };
  if (sim::mpu_reg_ptr == 0x3B) {
    put16(0, sim::acc_f[1]); put16(2, sim::acc_f[2]); put16(4, sim::acc_f[3]);
    put16(6, 1000);                                                      // temperature
    put16(8, sim::gyro_f[1]); put16(10, sim::gyro_f[2]); put16(12, sim::gyro_f[3]);
  } else {
    for (int i = 0; i < quantity; i++) sim::i2c_buf[i] = sim::mpu_regs[(sim::mpu_reg_ptr + i) & 0xFF];
  }
  sim::i2c_len = quantity; sim::i2c_idx = 0;
  return (uint8_t)quantity;
}
int TwoWire::available() { return sim::i2c_len - sim::i2c_idx; }
int TwoWire::read() { return sim::i2c_idx < sim::i2c_len ? sim::i2c_buf[sim::i2c_idx++] : -1; }

// EEPROM as the setup program would have written it: receiver channel 1-4 on inputs 8-11, not reversed,
// endpoints 1000/1500/2000, gyro axes roll=1 pitch=2 yaw=3, MPU-6050 at 0x68, signature "JMB".
uint8_t eeprom_mem[1024];
uint8_t EEPROMClass::read(int a) { return eeprom_mem[a]; }
void EEPROMClass::write(int a, uint8_t v) { eeprom_mem[a] = v; }
void init_eeprom() {
  auto put = [](int addr, int v) { eeprom_mem[addr] = v & 0xFF; eeprom_mem[addr + 1] = v >> 8; };
  for (int ch = 0; ch < 4; ch++) { put(ch * 2, 1500); put(8 + ch * 2, 2000); put(16 + ch * 2, 1000); }
  eeprom_mem[24] = 1; eeprom_mem[25] = 2; eeprom_mem[26] = 3; eeprom_mem[27] = 4;
  eeprom_mem[28] = 1; eeprom_mem[29] = 2; eeprom_mem[30] = 3;
  eeprom_mem[31] = 1; eeprom_mem[32] = 0x68;
  eeprom_mem[33] = 'J'; eeprom_mem[34] = 'M'; eeprom_mem[35] = 'B';
}

//==================================================================================================
// Test harness
//==================================================================================================
using namespace sim;

FILE* csv = nullptr;
FILE* events = nullptr;                          // check results with timestamps (used to make the video)
double last_log = -1;
int failures = 0;

void check(bool ok, const char* what, const std::string& detail = "") {
  std::printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what, detail.empty() ? "" : " - ", detail.c_str());
  if (events) std::fprintf(events, "%.3f\t%s\t%s\t%s\n", t_s(), ok ? "PASS" : "FAIL", what, detail.c_str());
  if (!ok) failures++;
}
std::string fmt(const char* f, double a, double b = 0, double c = 0) {
  char buf[200]; std::snprintf(buf, sizeof buf, f, a, b, c); return buf;
}

void log_row() {
  if (!csv) return;
  double t = t_s();
  if (t - last_log < 0.01) return;
  last_log = t;
  std::fprintf(csv, "%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f\n",
               t, roll, pitch, r, pz, angle_roll, esc_pulse[1], esc_pulse[2], esc_pulse[3], esc_pulse[4],
               stick[1], stick[2], stick[3], stick[4], (int)start, (int)failsafe_active, (int)gyro_failure,
               (int)led, (int)rx_on, angle_pitch, pid_i_mem_roll);
}

// Run the real flight loop until time t_end, calling the scenario script before every loop.
template <class F> void fly_until(double t_end, F script) {
  while (t_s() < t_end) {
    script(t_s());
    loop();
    if (start != 2) max_pulse_seen_disarmed = std::max({max_pulse_seen_disarmed, esc_pulse[1], esc_pulse[2], esc_pulse[3], esc_pulse[4]});
    log_row();
  }
}
void fly_until(double t_end) { fly_until(t_end, [](double) {}); }

bool all_motors_off() { return esc_pulse[1] <= 1000 && esc_pulse[2] <= 1000 && esc_pulse[3] <= 1000 && esc_pulse[4] <= 1000; }

void boot() {
  init_eeprom();
  reset_world();
  stick[3] = 1000; stick[4] = 1500;                // throttle low, yaw centred, as setup() requires
  setup();
  std::printf("  setup() finished at t=%.1f s (gyro calibrated, receiver ok)\n", t_s());
  check(!led, "LED off after start-up");
}

void arm() {
  double t0 = t_s();
  stick[3] = 1000; stick[4] = 1000;
  fly_until(t0 + 0.4);
  stick[4] = 1500;
  fly_until(t0 + 0.8);
}

// Find a throttle that holds altitude, by climbing to ~2 m and holding.
void take_off_and_hover(double hover_stick, double t_climb = 2.0) {
  double t0 = t_s();
  fly_until(t0 + t_climb, [&](double t) { stick[3] = (int)(1000 + (hover_stick + 60 - 1000) * std::min(1.0, (t - t0) / 0.8)); });
  fly_until(t0 + t_climb + 1.0, [&](double) { stick[3] = (int)hover_stick; });
}

struct Stats { double max_abs_roll = 0, max_abs_pitch = 0, sum2 = 0; int n = 0; double sum_roll = 0, sum_pitch = 0, sum_r = 0; };
template <class F> Stats measure(double t_end, F script) {
  Stats s;
  fly_until(t_end, [&](double t) {
    script(t);
    s.max_abs_roll = std::max(s.max_abs_roll, std::fabs(roll));
    s.max_abs_pitch = std::max(s.max_abs_pitch, std::fabs(pitch));
    s.sum2 += roll * roll + pitch * pitch; s.n++;
    s.sum_roll += roll; s.sum_pitch += pitch; s.sum_r += r;
  });
  return s;
}

// Throttle stick that holds altitude for the current build. The flight code scales every ESC pulse by
// (1 + (12.4 V - battery) / 35 V) for battery compensation, so the stick needs to be that much lower.
double HOVER = 1345;
void compute_hover() {
  double pulse = 1000 + 1000 * (MASS * G / 4.0) / T_MAX;
  double batt = BATTERY_V * 100;
  double comp = (batt < 1240 && batt > 800) ? 1.0 + (1240 - batt) / 3500.0 : 1.0;
  HOVER = pulse / comp;
}

//------------------------------ Scenarios ---------------------------------------------------------
void scenario_hover() {
  std::printf("Scenario: normal flight (take-off, hover in gusts, stick steps, landing, disarm)\n");
  boot();
  arm();
  check(start == 2, "arms with throttle low + yaw left, then yaw centre");
  check(esc_pulse[1] >= 1100 && esc_pulse[2] >= 1100, "motors idle at >=1100 us when armed", fmt("ESC1 %.0f us", esc_pulse[1]));
  gust_strength = 0.03;
  take_off_and_hover(HOVER);
  check(!on_ground, "takes off", fmt("altitude %.1f m", pz));
  double t0 = t_s();
  Stats h = measure(t0 + 10, [](double) {});
  double rms = std::sqrt(h.sum2 / h.n);
  check(!flipped && h.max_abs_roll < 8 && h.max_abs_pitch < 8, "stable hover for 10 s in gusts (crosses micros() overflow)",
        fmt("max roll %.1f deg, max pitch %.1f deg, RMS %.2f deg", h.max_abs_roll, h.max_abs_pitch, rms));
  check(t_s() > 25.0, "micros() overflow happened during the flight", fmt("wrap at t=25.0 s, now t=%.1f s", t_s()));

  gust_strength = 0.0;
  t0 = t_s();
  int base[5]; for (int i = 1; i <= 4; i++) base[i] = esc_pulse[i];
  int kick = 0; double t10 = -1;
  fly_until(t0 + 1.0, [&](double t) {
    stick[1] = 1700;
    if (t - t0 < 0.3) for (int i = 1; i <= 4; i++) kick = std::max(kick, std::abs(esc_pulse[i] - base[i]));
    if (t10 < 0 && roll >= 10) t10 = t - t0;
  });
  std::printf("  [INFO] roll stick flick: peak ESC change %d us in the first 0.3 s, reaches 10 deg bank after %.0f ms\n", kick, t10 * 1000);
  check(kick < 250, "no big ESC spike when the roll stick is flicked", fmt("peak change %.0f us", kick));
  check(t10 > 0 && t10 < 0.6, "still responds quickly to the stick (10 deg within 600 ms)", fmt("%.0f ms", t10 * 1000));
  Stats sr = measure(t0 + 2.0, [](double) { stick[1] = 1700; });
  double roll_avg = sr.sum_roll / sr.n;
  check(roll_avg > 9 && roll_avg < 18, "roll stick right -> banks RIGHT to ~13 deg (auto-level limit)", fmt("average roll %+.1f deg", roll_avg));
  fly_until(t0 + 4.0, [](double) { stick[1] = 1500; });
  check(std::fabs(roll) < 3, "returns to level when the roll stick is released", fmt("roll %+.1f deg", roll));

  t0 = t_s();
  fly_until(t0 + 1.0, [](double) { stick[2] = 1300; });
  Stats sp = measure(t0 + 2.0, [](double) { stick[2] = 1300; });
  double pitch_avg = sp.sum_pitch / sp.n;
  check(pitch_avg < -9 && pitch_avg > -18, "pitch stick forward -> nose DOWN ~13 deg", fmt("average pitch %+.1f deg", pitch_avg));
  fly_until(t0 + 4.0, [](double) { stick[2] = 1500; });
  check(std::fabs(pitch) < 3, "returns to level when the pitch stick is released", fmt("pitch %+.1f deg", pitch));

  t0 = t_s();
  fly_until(t0 + 0.7, [](double) { stick[4] = 1700; });
  Stats sy = measure(t0 + 1.5, [](double) { stick[4] = 1700; });
  double yaw_rate = sy.sum_r / sy.n;
  check(yaw_rate > 40 && yaw_rate < 90, "yaw stick right -> nose turns RIGHT at ~64 deg/s", fmt("average yaw rate %+.0f deg/s", yaw_rate));
  fly_until(t0 + 2.5, [](double) { stick[4] = 1500; });

  t0 = t_s();
  fly_until(t0 + 25, [&](double) { stick[3] = on_ground ? 1000 : (int)(HOVER - 40); });   // descend, cut on touchdown
  check(on_ground && !flipped, "lands by lowering the throttle", fmt("altitude %.1f m", pz));
  t0 = t_s();
  fly_until(t0 + 0.5, [](double) { stick[3] = 1000; stick[4] = 2000; });
  fly_until(t0 + 1.0, [](double) { stick[4] = 1500; });
  check(start == 0 && all_motors_off(), "disarms with throttle low + yaw right; motors at 1000 us");
}

void scenario_failsafe_cut() {
  std::printf("Scenario: radio signal lost in flight (default failsafe_throttle = 1000)\n");
  boot();
  arm();
  take_off_and_hover(HOVER);
  double t_off = t_s();
  rx_on = false;
  double t_stop = -1;
  fly_until(t_off + 1.0, [&](double t) { if (t_stop < 0 && all_motors_off()) t_stop = t; });
  check(t_stop > 0, "motors stop after the signal is lost", fmt("%.0f ms after the last pulse", (t_stop - t_off) * 1000));
  check(t_stop > 0 && t_stop - t_off < 0.20, "...within 200 ms");
  check(start == 0, "flight controller is disarmed");
  // Try to arm with no signal: impossible, the sticks can't be read. Then bring the signal back.
  fly_until(t_s() + 2.0);
  check(start == 0 && all_motors_off(), "stays disarmed while there is no signal");
  rx_on = true;
  stick[3] = 1000; stick[4] = 1500;
  fly_until(t_s() + 0.5);
  check(start == 0 && !failsafe_active, "signal back: failsafe clears but it does NOT re-arm by itself");
  arm();
  check(start == 2, "can be re-armed normally with the stick command");
}

void scenario_failsafe_descent() {
  std::printf("Scenario: radio signal lost with failsafe_throttle set below hover (controlled descent)\n");
  failsafe_throttle = (int)HOVER - 60;
  boot();
  arm();
  gust_strength = 0.02;
  double t0 = t_s();
  fly_until(t0 + 3, [&](double t) { stick[3] = (int)(1000 + (HOVER + 80 - 1000) * std::min(1.0, (t - t0) / 1.0)); });   // climb
  fly_until(t0 + 9, [](double) { stick[3] = (int)HOVER; stick[1] = 1600; });   // settle in hover, holding a bit of right roll
  double alt0 = pz;
  rx_on = false;
  double t_off = t_s();
  Stats s = measure(t_off + 4.5, [](double) {});
  check(!flipped && s.max_abs_roll < 12 && std::fabs(roll) < 3, "levels out after signal loss (stick input ignored)",
        fmt("roll now %+.1f deg, max %.1f deg", roll, s.max_abs_roll));
  check(pz < alt0 && start == 2, "descends with motors still running", fmt("altitude %.1f m -> %.1f m", alt0, pz));
  fly_until(t_off + 6.0);
  check(start == 0 && all_motors_off(), "motors stop after failsafe_descent_ms (5 s)");
}

void scenario_gyro_fail() {
  std::printf("Scenario: MPU-6050 stops answering in flight (loose wire)\n");
  boot();
  arm();
  take_off_and_hover(HOVER);
  i2c_fail = true;
  double t_fail = t_s(), t_stop = -1;
  fly_until(t_fail + 1.0, [&](double t) { if (t_stop < 0 && all_motors_off()) t_stop = t; });
  check(t_stop > 0 && t_stop - t_fail < 0.25, "motors stop after repeated I2C failures (no freeze)", fmt("%.0f ms", (t_stop - t_fail) * 1000));
  check(led, "LED on solid");
  i2c_fail = false;
  fly_until(t_s() + 1.0);
  arm();
  check(start == 0 && all_motors_off(), "refuses to re-arm even after the gyro comes back (needs power cycle)");
}

void scenario_ground_windup() {
  std::printf("Scenario: armed on sloped ground (6 deg) for 10 s, then take off\n");
  boot();
  ground_roll = 6;                                 // the quad sits tilted
  arm();
  double max_i = 0;
  fly_until(t_s() + 10, [&](double) { max_i = std::max(max_i, (double)std::fabs(pid_i_mem_roll)); });
  // The reset runs just before calculate_pid(), so at most one loop of I (gain 0.04 x error) remains. Without the fix
  // it would keep growing every loop up to the +/-400 limit.
  check(max_i < 3, "roll I-term can't wind up while armed on the ground (max one loop's worth, limit is 400)", fmt("max |I| = %.1f", max_i));
  double t0 = t_s();
  Stats s = measure(t0 + 4, [&](double t) { stick[3] = (int)(1000 + (HOVER + 60 - 1000) * std::min(1.0, (t - t0) / 0.6)); });
  check(!flipped && !on_ground && std::fabs(roll) < 4, "takes off from the slope and levels out (no flip)",
        fmt("max roll %.1f deg, roll now %+.1f deg", s.max_abs_roll, roll));
}

void scenario_rx_glitch() {
  std::printf("Scenario: noisy receiver signal and one dropped frame\n");
  boot();
  arm();
  take_off_and_hover(HOVER);
  rx_glitches = true;
  Stats s = measure(t_s() + 3, [](double) {});
  check(!flipped && s.max_abs_roll < 8 && start == 2, "flies through 300 us noise spikes (filtered out)", fmt("max roll %.1f deg", s.max_abs_roll));
  rx_glitches = false;
  bool fs = false;
  rx_drop_frame = true;
  fly_until(t_s() + 1, [&](double) { fs |= failsafe_active; });
  check(!fs && start == 2, "one dropped 20 ms frame does not trigger the failsafe");
}

void scenario_robust() {
  compute_hover();
  analog_battery = (int)(BATTERY_V * 100 / 1.2317 - 65);
  I_YAW = I_ROLL * 1.84;
  std::printf("Scenario: robustness  mass %.2f kg, motor thrust %.1f N, motor lag %.0f ms, inertia %.4f, noise x%.1f, gusts x%.1f, battery %.1f V (hover stick %.0f)\n",
              MASS, T_MAX, MOTOR_TAU * 1000, I_ROLL, NOISE_MULT, GUST_MULT, BATTERY_V, HOVER);
  boot();
  arm();
  gust_strength = 0.03;
  take_off_and_hover(HOVER);
  // Hover in gusts: tilt, roll-rate wobble, motor jitter and how often the motors hit their limits.
  double max_ang = 0, p2 = 0, d2 = 0; int n = 0, sat = 0, prev_esc = esc_pulse[1];
  fly_until(t_s() + 8, [&](double) {
    max_ang = std::max({max_ang, std::fabs(roll), std::fabs(pitch)});
    p2 += p * p; d2 += (double)(esc_pulse[1] - prev_esc) * (esc_pulse[1] - prev_esc); prev_esc = esc_pulse[1]; n++;
    for (int i = 1; i <= 4; i++) if (esc_pulse[i] <= 1100 || esc_pulse[i] >= 2000) { sat++; break; }
  });
  double rate_rms = std::sqrt(p2 / n);
  double jitter = std::sqrt(d2 / n);                // loop-to-loop ESC change: high = noisy, hot motors
  double sat_pct = 100.0 * sat / n;
  // Roll stick step in calm air: overshoot and settling.
  gust_strength = 0;
  double t0 = t_s(), peak = 0;
  fly_until(t0 + 2.0, [&](double) { stick[1] = 1700; peak = std::max(peak, roll); });
  double hold = roll;
  double t_rel = t_s(), settle = -1;
  fly_until(t_rel + 3.0, [&](double t) { stick[1] = 1500; if (settle < 0 && std::fabs(roll) < 4 && std::fabs(p) < 10) settle = t - t_rel; });
  double overshoot = peak - hold;
  check(!flipped, "no flip");
  check(max_ang < 10, "hover in gusts stays within 10 deg", fmt("max %.1f deg", max_ang));
  check(rate_rms < 20, "no sustained wobble (roll-rate RMS < 20 deg/s)", fmt("%.1f deg/s", rate_rms));
  check(sat_pct < 5, "motors rarely hit their limits (<5% of the time)", fmt("%.1f%%", sat_pct));
  check(jitter < 40, "motor signal not too noisy (loop-to-loop change < 40 us)", fmt("%.0f us", jitter));
  check(overshoot < 5, "roll step overshoot < 5 deg", fmt("%.1f deg", overshoot));
  check(settle > 0 && settle < 1.5, "levels out within 1.5 s after the stick is released", fmt("%.2f s", settle));
  std::printf("ROBUST|%.2f|%.1f|%.0f|%.4f|%.1f|%.1f|%.1f|%.0f|%.1f|%.1f|%.0f|%.1f|%.1f|%.2f|%s\n",
              MASS, T_MAX, MOTOR_TAU * 1000, I_ROLL, NOISE_MULT, GUST_MULT, BATTERY_V, HOVER,
              max_ang, rate_rms, jitter, sat_pct, overshoot, settle, failures ? "FAIL" : "PASS");
}

int main(int argc, char** argv) {
  // Arguments: <scenario> [csv_file] [events_file] [key=value ...]
  std::vector<std::string> pos;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    size_t eq = a.find('=');
    if (eq == std::string::npos) { pos.push_back(a); continue; }
    std::string k = a.substr(0, eq); double v = std::atof(a.c_str() + eq + 1);
    if (k == "mass") MASS = v; else if (k == "tmax") T_MAX = v; else if (k == "tau") MOTOR_TAU = v / 1000.0;
    else if (k == "inertia") { I_ROLL = v; I_PITCH = v; } else if (k == "noise") NOISE_MULT = v;
    else if (k == "gust") GUST_MULT = v; else if (k == "battery") BATTERY_V = v;
    else { std::printf("unknown parameter %s\n", k.c_str()); return 2; }
  }
  if (pos.size() > 2) events = std::fopen(pos[2].c_str(), "w");
  std::string name = pos.size() > 0 ? pos[0] : "hover";
  if (pos.size() > 1) {
    csv = std::fopen(pos[1].c_str(), "w");
    std::fprintf(csv, "t,roll,pitch,yaw_rate,alt,fc_angle_roll,esc1,esc2,esc3,esc4,rx_roll,rx_pitch,rx_throttle,rx_yaw,armed,failsafe,gyro_failure,led,rx_on,fc_angle_pitch,i_roll\n");
  }
  if (name == "hover") scenario_hover();
  else if (name == "failsafe_cut") scenario_failsafe_cut();
  else if (name == "failsafe_descent") scenario_failsafe_descent();
  else if (name == "gyro_fail") scenario_gyro_fail();
  else if (name == "ground_windup") scenario_ground_windup();
  else if (name == "rx_glitch") scenario_rx_glitch();
  else if (name == "robust") scenario_robust();
  else { std::printf("unknown scenario %s\n", name.c_str()); return 2; }
  check(max_pulse_seen_disarmed <= 1000, "motors never got more than 1000 us while disarmed",
        fmt("max %.0f us", max_pulse_seen_disarmed));
  if (csv) std::fclose(csv);
  if (events) std::fclose(events);
  std::printf("  => %s\n\n", failures ? "FAILED" : "all checks passed");
  return failures ? 1 : 0;
}
