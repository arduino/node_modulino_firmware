#include "Wire.h"
#include <vector>
#include <VL53L1X.h>
#include <Arduino_APDS9960.h>

VL53L1X tof_sensor;
APDS9960 color_sensor(Wire1, -1);

#define Wire Wire1

class Module {
public:
  Module(uint8_t address = 0xFF)
    : address(address) {}
  bool begin() {
    if (address == 0xFF) {
      //find possible matches
    }
  }
  bool read(uint8_t* buf, int howmany) {
    if (address == 0xFF) {
      return false;
    }
    Wire.requestFrom(address, howmany+1);
    auto start = millis();
    while ((Wire.available() == 0) && (millis() - start < 100)) {
      delay(1);
    }
    if (Wire.available() < howmany) {
      return false;
    }
    pinstrap_address = Wire.read();
    for (int i = 0; i < howmany; i++) {
      buf[i] = Wire.read();
    }
    while (Wire.available()) {
      Wire.read();
    }
    return true;
  }
  bool write(uint8_t* buf, int howmany) {
    if (address == 0xFF) {
      return false;
    }
    Wire.beginTransmission(address);
    for (int i = 0; i < howmany; i++) {
      Wire.write(buf[i]);
    }
    Wire.endTransmission();
    return true;
  }
  bool nonDefaultAddress() {
    return (pinstrap_address != address);
  }
private:
  uint8_t address;
  uint8_t pinstrap_address;
};

class Buttons : public Module {
public:
  Buttons(uint8_t address = 0xFF)
    : Module(address) {}
  bool get(bool& a, bool& b, bool& c) {
    uint8_t buf[3];
    auto res = read((uint8_t*)buf, 3);
    a = buf[0];
    b = buf[1];
    c = buf[2];
    auto ret = res && (a != last_a || b != last_b || c != last_c);
    last_a = a;
    last_b = b;
    last_c = c;
    return ret;
  }
private:
  bool last_a, last_b, last_c;
};

class Tone : public Module {
public:
  Tone(uint8_t address = 0xFF)
    : Module(address) {}
  void tone(size_t freq, size_t len_ms) {
    uint8_t buf[8];
    memcpy(&buf[0], &freq, 4);
    memcpy(&buf[4], &len_ms, 4);
    write(buf, 8);
  }
  void noTone() {
    uint8_t buf[8];
    memset(&buf[0], 0, 8);
    write(buf, 8);
  }
};

class Color {
public:
  Color(uint8_t r, uint8_t g, uint8_t b)
    : r(r), g(g), b(b) {}
  operator uint32_t() {
    return (b << 8 | g << 16 | r << 24);
  }
private:
  uint8_t r, g, b;
};

Color RED(255, 0, 0);
Color BLUE(0, 0, 255);
Color GREEN(0, 255, 0);
Color VIOLET(255, 0, 255);

class LEDS : public Module {
public:
  LEDS(uint8_t address = 0xFF)
    : Module(address) {
    memset(data, 0xE0, 40);
  }
  bool begin() {
    show();
  }
  void set(int idx, uint8_t brightness, Color rgb) {
    data[idx] = (uint32_t)rgb | brightness | 0xE0;
  }
  void show() {
    write((uint8_t*)data, 10 * 4);
  }
private:
  uint32_t data[10];
};


class Encoder : public Module {
public:
  Encoder(uint8_t address = 0xFF)
    : Module(address) {}
  int16_t get() {
    uint8_t buf[3];
    auto res = read(buf, 3);
    if (res == false) {
      return 0;
    }
    _pressed = (buf[2] != 0);
    int16_t ret = buf[0] | (buf[1] << 8);
    return ret;
  }
  bool pressed() {
    return _pressed;
  }
private:
  bool _pressed = false;
};

std::vector<Module*> findModules() {

  std::vector<Module*> modules;
  for (int i = 8; i < 127; i++) {
    Wire.beginTransmission(i);
    auto ret = Wire.endTransmission();
    if (ret != 2) {
      Serial.println(String(i) + ":" + String(ret));
    }
  }
  return modules;
}

Buttons button(62);
Tone _tone(30);
LEDS leds(54);
Encoder encoder(58);

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);

  findModules();

  tof_sensor.setBus(&Wire1);
  tof_sensor.init();
  tof_sensor.setDistanceMode(VL53L1X::Short);
  tof_sensor.setMeasurementTimingBudget(50000);
  tof_sensor.startContinuous(50);

  color_sensor.begin();

  leds.begin();
}

int skip = 0;
int pitch = 0;
bool a = false;
bool b = false;
bool c = false;

void loop() {

  if (encoder.pressed()) {
    skip = (skip + 1) % 5;
  }

  pitch = encoder.get() + tof_sensor.read();
  //Serial.println(pitch);
  //Serial.println(encoder.pressed());

  if (color_sensor.colorAvailable()) {
    int r; int g; int b;
    color_sensor.readColor(r,g,b);
    leds.set(4 + skip, 50, Color(r,g,b));
    leds.show();
  }

  if (button.get(a, b, c)) {
    if (a) {
      leds.set(1 + skip, 15, RED);
      _tone.tone(440 + pitch, 1000);
    } else {
      leds.set(1 + skip, 0, RED);
    }
    if (b) {
      leds.set(2 + skip, 15, BLUE);
      _tone.tone(880 + pitch, 1000);
    } else {
      leds.set(2 + skip, 0, BLUE);
    }
    if (c) {
      leds.set(3 + skip, 15, GREEN);
      _tone.tone(1240 + pitch, 1000);
    } else {
      leds.set(3 + skip, 0, GREEN);
    }
    leds.show();
  }
}
