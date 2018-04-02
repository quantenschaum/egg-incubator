#include <avr/wdt.h>
#include <SoftwareServo.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <EEPROM.h>

// stty -F /dev/ttyACM0 115200 cs8 cread clocal -hupcl time 30 && tee incubator.log </dev/ttyACM0

#define WDT_TIMEOUT WDTO_8S // if defined, enable hardware watchdog
#define DHTPIN 3 // data pin of the DHT T/H sensor
#define T_OFFSET 0.9 // temperature sensor offset
#define FAN_PIN 2 // fan tacho signal pin
#define FAN_THRES 500 // fan alarm threshold
#define BEEPER A2 // pin where beeper is attached
#define BRIGHTNESS 10 // display brightness pin
#define HEATER A1 // heater MOSFET pin
#define DELAY 2000 // loop delay in ms
#define TS_ADDR 0 // temperature set point EEPROM address
#define HS_ADDR 4 // humidity set point EEPROM address
#define HC_ADDR 8 // humiditiy control mode EEPROM address
#define TI_RESET 1 // integral reset threshold, set integral to 0 when T error greater than this
#define HI_RESET 5 // integral reset threshold, set integral to 0 when H error greater than this
#define ALARM_T 2 // temperature alarm threshold, alert if T error greater than this
#define ALARM_H 8 // humidity alarm threshold, alert if H error greater than this
#define H_AUTO_THRES 3 // disable vent control in auto mode if H error < this
#define H_AUTO_COUNT 200 // disable for n cycles
#define HWAT 0.25 // holt winters parameters for temperature smoothing
#define HWBT 0.2
#define HWAH 0.7 // holt winters parameters for humidity smoothing
#define HWBH 0.5
#define A 0.005 // long average parameter
#define VENTCLOSED 80 // consider vent closed if under this angle
#define VENTOPENMS  480000L // open vent if closed longer
#define VENTRESETMS 600000L // reset vent after this time (>VENTOPENMS!)

SoftwareServo vent;
DHT dht(DHTPIN, DHT22);
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define RIGHT 16
#define UP 8
#define DOWN 4
#define LEFT 2
#define SELECT 1
#define NO_KEY 0

byte getKey() {
  int key = analogRead(0);
  if (key < 50) {
    return RIGHT;
  } else if (key < 150) {
    return UP;
  } else if (key < 300) {
    return DOWN;
  } else if (key < 500) {
    return LEFT;
  } else if (key < 800) {
    return SELECT;
  } else {
    return NO_KEY;
  }
}

void eeread(int address, int length, void* p) {
  byte* b = (byte*)p;
  for (int i = 0; i < length; i++) {
    *b++ = EEPROM.read(address + i);
  }
}

void eewrite(int address, int length, void* p) {
  byte* b = (byte*)p;
  for (int i = 0; i < length; i++) {
    EEPROM.write(address + i, *b++);
  }
}

void write_byte(int address, byte &value) {
  eewrite(address, sizeof(value), &value);
}

byte read_byte(int address) {
  byte value;
  eeread(address, sizeof(value), &value);
  return value;
}

void write_int(int address, int &value) {
  eewrite(address, sizeof(value), &value);
}

int read_int(int address) {
  int value;
  eeread(address, sizeof(value), &value);
  return value;
}

void write_float(int address, float &value) {
  eewrite(address, sizeof(value), &value);
}

float read_float(int address) {
  float value;
  eeread(address, sizeof(value), &value);
  return value;
}

void heater(boolean on) {
  digitalWrite(HEATER, !on ? LOW : HIGH);
}

boolean heater() {
  return digitalRead(HEATER) == HIGH;
}

volatile int fancount;
void count() {
  ++fancount;
}

void beep(unsigned long f, unsigned long l) {
  pinMode(BEEPER, OUTPUT);
  byte v = 0;
  f = 500000 / f;
  l = (1000 * l) / f;
  for (int i = 0; i < l; ++i) {
    digitalWrite(BEEPER, v = !v);
    delayMicroseconds(f);
  }
  pinMode(BEEPER, INPUT);
}

float Ts, Hs; // set points
byte Hcontrol; // H control mode
byte Ts_changed, Hs_changed; // setpoint changed flags
float ET, dETdt, IETdt; // prop/diff/integ terms for T
float EH, dEHdt, IEHdt; // prop/diff/integ terms for H
float Tavg = NAN, Tvar; // T average
float Havg = NAN, Hvar; // average
float Hpower, Hduty; // heater current power, average duty cycle
unsigned long t0, Hon, talarm, tventclosed;
byte c, displayMode;
byte key, bri = 255, alarm;

void setup() {
#if defined(WDT_TIMEOUT)
  wdt_enable(WDT_TIMEOUT);
#endif
  pinMode(HEATER, OUTPUT);
  heater(0);
  pinMode(BRIGHTNESS, OUTPUT);
  analogWrite(BRIGHTNESS, bri = 255);
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.print("Incubator 0.6");
  lcd.setCursor(0, 1);
  lcd.print(__DATE__);
  dht.begin();
  vent.setMinimumPulse(800);
  vent.setMaximumPulse(2600);
  vent.attach(11);
  //  write_float(TS_ADDR, Ts=37.8);  write_float(HS_ADDR, Hs=55);
  Ts = read_float(TS_ADDR);
  Hs = read_float(HS_ADDR);
  Hcontrol = read_byte(HC_ADDR);
  pinMode(FAN_PIN, INPUT_PULLUP);
  attachInterrupt(0, count, FALLING);
  sei();
  beep(800, 100);
  beep(1000, 100);
  beep(1200, 100);
  beep(1600, 100);
}

boolean ventclosed;

void loop() {
  unsigned long t1 = millis();
  int dt = t1 - t0;

  if (!key) {
    key = getKey();
  }

  if (key) {
    analogWrite(BRIGHTNESS, bri = 255);
  }

  if (t1 - Hon > Hpower * DELAY) {
    heater(0);
  }

  if (Hcontrol && Hcontrol < H_AUTO_COUNT) {
    vent.refresh();
  }

  if (alarm && !(alarm & 8)) {
    beep(1000, 50);
    beep(1414, 50);
  }

  if (dt > DELAY || key) {
    //    beep(2000, 50);
    float T = dht.readTemperature() + T_OFFSET;
    float H = dht.readHumidity();

    if (isnan(T) || T < 10 || T > 60 || isnan(H) || H < 5 || H > 95) {
      heater(0);
      lcd.clear();
      lcd.print("SENSOR ERROR!");
      lcd.setCursor(0, 1);
      lcd.print("T=");
      lcd.print(T);
      lcd.print("C H=");
      lcd.print(H, 1);
      lcd.print("%");
      beep(2000, 1000);
      return;
    }

    float dts = dt * 1e-3;

    int fanrpm = fancount * 60 / dts;
    fancount = 0;

    // temperature Holt-Winters smoothing
    float E0 = ET;
    ET = HWAT * (T - Ts) + (1 - HWAT) * (ET + dETdt * dts); // smoothed T error (deviation from set point)
    dETdt = HWBT * (ET - E0) / dts + (1 - HWBT) * dETdt; // smoothed derivative
    IETdt += ET * dts; // integral of error
    if (abs(ET) > TI_RESET) // reset integral on big deviation
      IETdt = 0;
    float pidT = 1.1765 * (ET + 0.010526 * IETdt + 23.75 * dETdt); // PID value, adjust coefficients to tune
    Hpower = fanrpm > FAN_THRES ? max(0, min(1, -pidT)) : 0;
    heater(Hpower > 0.1);
    Hon = millis();

    // humidity Holt-Winters smoothing
    E0 = EH;
    EH = HWAH * (H - Hs) + (1 - HWAH) * (EH + dEHdt * dts); // smoothed H error (deviation from set point)
    dEHdt = HWBH * (EH - E0) / dts + (1 - HWBH) * dEHdt; // smoothed derivative
    IEHdt += EH * dts; // integral of error
    if (abs(EH) > HI_RESET) // reset integral on big deviation
      IEHdt = 0;
    float pidH = 0.1176 * (EH + 0.09091 * IEHdt + 2.75 * dEHdt); // PID value, adjust coefficients to tune
    vent.write(pidH * 180);

    boolean ventclosed0 = ventclosed;
    ventclosed = vent.read() < VENTCLOSED;
    if (ventclosed && ventclosed != ventclosed0) {
      tventclosed = millis();
    }

    boolean openvent = ventclosed && millis() - tventclosed > VENTOPENMS;
    if (openvent) {
      vent.write(180);
      if (millis() - tventclosed > VENTRESETMS) {
        tventclosed = millis();
      }
    }

    if (Hcontrol > 1) {
      if (abs(EH) > H_AUTO_THRES || openvent) {
        Hcontrol = 2;
      } else {
        if (Hcontrol < H_AUTO_COUNT) {
          ++Hcontrol;
        } else {
          IEHdt = 0;
        }
      }
    }

    // long term averages
    Tavg = A * T + (1 - A) * (isnan(Tavg) ? T : Tavg);
    Tvar = A * pow(T - Tavg, 2) + (1 - A) * (isnan(Tvar) ? 0 : Tvar);
    float Tstd = sqrt(Tvar);

    Havg = A * H + (1 - A) * (isnan(Havg) ? H : Havg);
    Hvar = A * pow(H - Havg, 2) + (1 - A) * (isnan(Hvar) ? 0 : Hvar);
    float Hstd = sqrt(Hvar);

    Hduty = A * Hpower + (1 - A) * Hduty;

    if (Ts_changed) {
      if (Ts_changed-- == 1)
        write_float(TS_ADDR, Ts);
    }

    if (Hs_changed) {
      if (Hs_changed-- == 1)
        write_float(HS_ADDR, Hs);
    }

    if (key & SELECT)
      displayMode = ++displayMode % 8;

    lcd.clear();
    lcd.print("T=");
    lcd.print(Ts + ET);
    lcd.print("C H=");
    lcd.print(Hs + EH, 1);
    lcd.print("%");

    lcd.setCursor(0, 1);
    float uptime;
    char unit;
    switch (displayMode) {
      case 0: // raw values
        lcd.print("T=");
        lcd.print(T);
        lcd.print("C H=");
        lcd.print(H, 1);
        lcd.print("%");
        break;
      case 1: // temperature setpoint
        if (key & (UP | DOWN | LEFT | RIGHT)) {
          Ts = max(20, min(50, Ts + (key & (UP | RIGHT) ? +1 : -1) * (key & (UP | DOWN) ? 0.1 : 1)));
          Ts_changed = 10;
        }
        lcd.print("Ts=");
        lcd.print(Ts);
        lcd.print("C");
        break;
      case 2: // humidity setpoint
        if (key & (UP | DOWN)) {
          Hs = max(10, min(90, Hs + (key & UP ? +1 : -1)));
          Hs_changed = 10;
        }
        if (key & RIGHT) {
          if (Hcontrol < 2) {
            Hcontrol = ++Hcontrol;
          } else {
            Hcontrol = 0;
          }
          IEHdt = 0;
          write_byte(HC_ADDR, Hcontrol);
        }
        lcd.print("Hs=");
        lcd.print(Hs);
        lcd.print("% ");
        lcd.print(Hcontrol == 1 ? "on" : (Hcontrol > 1 ? "auto" : "off"));
        break;
      case 3: // average temperatur
        lcd.print("Ta=");
        lcd.print(Tavg);
        lcd.print("C (");
        lcd.print(Tstd);
        lcd.print(")");
        break;
      case 4: // average humidity
        lcd.print("Ha=");
        lcd.print(Havg);
        lcd.print("% (");
        lcd.print(Hstd);
        lcd.print(")");
        break;
      case 5: // heater duty cycle
        lcd.print("Hd=");
        lcd.print(Hduty);
        lcd.print(" Hp=");
        lcd.print(Hpower);
        break;
      case 6: // air vent
        lcd.print("V=");
        lcd.print(vent.read() / 180.0);
        lcd.print(" F=");
        lcd.print(fanrpm);
        break;
      case 7: // average humidity
        uptime = t1 * 1e-3;
        unit = 's';
        if (uptime > 60) {
          uptime /= 60;
          unit = 'm';
          if (uptime > 60) {
            uptime /= 60;
            unit = 'h';
            if (uptime > 24) {
              uptime /= 24;
              unit = 'd';
            }
          }
        }
        lcd.print("Up=");
        lcd.print(uptime, 1);
        lcd.print(unit);
        break;
      default:;
    }

    if (abs(ET) > ALARM_T) {
      alarm |= 1;
      vent.write(ET < 0 ? 0 : 180);
      if (Hcontrol > 1) {
        Hcontrol = 2;
      }
    } else {
      alarm &= ~1;
    }

    if (Hcontrol && abs(EH) > ALARM_H) {
      alarm |= 2;
    } else {
      alarm &= ~2;
    }

    if (fanrpm < FAN_THRES) {
      alarm |= 4;
    } else {
      alarm &= ~4;
    }

    if (alarm & 7) {
      if (!talarm) {
        talarm = millis();
      }
      // sound on persistent alarm and fan failure
      if (millis() - talarm > 300000L || alarm & 4) {
        alarm &= ~8;
      }
      analogWrite(BRIGHTNESS, bri = 255);
      lcd.setCursor(0, 0);
      if (alarm & 1)
        lcd.print("T ");
      if (alarm & 2)
        lcd.print("H ");
      if (alarm & 4)
        lcd.print("F ");
      lcd.print("ALARM!          ");
      if (!(alarm & 8) && key) {
        alarm |= 8; // alarm acknowledged
        talarm = millis();
      }
      if (!(alarm & 8)) {
        lcd.setCursor(0, 1);
        lcd.print("T=");
        lcd.print(T);
        lcd.print("C H=");
        lcd.print(H, 1);
        lcd.print("%");
      }
    } else {
      alarm = 0;
      talarm = 0;
    }

    if (key) {
      delay(500);
    }

    if (bri) {
      analogWrite(BRIGHTNESS, --bri);
    }
    key = 0;
    t0 = t1;
    ++c;
  }
#if defined(WDT_TIMEOUT)
  wdt_reset();
#endif
}



