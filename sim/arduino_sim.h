// Fake Arduino Uno layer so the real flight_controller.ino can run on a PC inside a physics simulation.
// Time is virtual: every micros() call advances the simulated clock, and the simulator's physics,
// radio receiver and ESC pulse measurement run inside that clock.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstdlib>

typedef uint8_t byte;
typedef bool boolean;

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define F(s) (s)

#define PCIE0 0
#define PCINT0 0
#define PCINT1 1
#define PCINT2 2
#define PCINT3 3

#define B00000001 1
#define B00000010 2
#define B00000100 4
#define B00001000 8
#define B00001111 15
#define B00110000 48
#define B01111111 127
#define B10111111 191
#define B11011111 223
#define B11101111 239
#define B11110000 240

// PORTD drives the ESC signal pins D4-D7. Every write is reported to the simulator so it can
// measure the real pulse widths the flight code produces.
void sim_portd_changed(uint8_t old_value, uint8_t new_value);
struct PortDRegister {
  uint8_t value = 0;
  operator uint8_t() const { return value; }
  PortDRegister& operator|=(uint8_t m) { uint8_t o = value; value |= m; sim_portd_changed(o, value); return *this; }
  PortDRegister& operator&=(uint8_t m) { uint8_t o = value; value &= m; sim_portd_changed(o, value); return *this; }
  PortDRegister& operator=(uint8_t v)  { uint8_t o = value; value = v;  sim_portd_changed(o, value); return *this; }
};
extern PortDRegister PORTD;
extern volatile uint8_t PINB;                     // Receiver pins D8-D11, driven by the simulated receiver.
extern uint8_t DDRD, DDRB, PCICR, PCMSK0, TWBR;

#define ISR(vector) void vector##_handler()
void PCINT0_vect_handler();

// The simulated "interrupt" runs synchronously inside the clock, so atomic blocks need no locking.
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(x) for (int _atomic_once = 1; _atomic_once; _atomic_once = 0)

unsigned long micros();
unsigned long millis();
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);

struct TwoWire {
  void begin();
  void setWireTimeout(uint32_t timeout_us, bool reset);
  void beginTransmission(int address);
  size_t write(uint8_t data);
  uint8_t endTransmission();
  uint8_t requestFrom(int address, int quantity);
  int available();
  int read();
};
extern TwoWire Wire;

struct EEPROMClass {
  uint8_t read(int address);
  void write(int address, uint8_t value);
};
extern EEPROMClass EEPROM;

// Match the ATmega328P: int is 16-bit and double is 32-bit. This makes overflow and sign-extension
// behave like the real board (for example "Wire.read()<<8|Wire.read()" producing negative numbers).
#define int short
#define double float
