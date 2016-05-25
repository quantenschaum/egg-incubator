#include <avr/wdt.h>
#include <SoftwareServo.h>
//#include <Streaming.h>
//#include <OneWire.h>
//#include <DallasTemperature.h>
//#include <MemoryFree.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <EEPROM.h>

// stty -F /dev/ttyACM0 115200 cs8 cread clocal -hupcl time 30 && tee incubator.log </dev/ttyACM0

#define WDT_TIMEOUT WDTO_8S
#define SERIAL_LOGGING
#define DHTPIN 3
//#define ONE_WIRE 3
#define T_OFFSET 0.9
#define FAN_PIN 2
#define FAN_THRES 500
#define BEEPER A2
#define BRIGHTNESS 10
#define HEATER A1
#define DELAY 2000
#define TS_ADDR 0
#define HS_ADDR 4
#define HC_ADDR 8
#define TI_RESET 1
#define HI_RESET 5
#define ALARM_T 2
#define ALARM_H 10
#define H_AUTO_THRES 3
#define H_AUTO_COUNT 150
#define HWAT 0.25 // holt winters parameters for temperature
#define HWBT 0.2
#define HWAH 0.7 // holt winters parameters for humidity
#define HWBH 0.5
#define A 0.005 // long average parameter

SoftwareServo vent;
//OneWire oneWire(ONE_WIRE);
//DallasTemperature sensors(&oneWire);
DHT dht(DHTPIN, DHT22);
//LiquidCrystal lcd(2,3,4,5,6,7);
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

float Ts, Hs;
byte Hcontrol;
byte Ts_changed, Hs_changed;
float ET, dETdt, IETdt;
float EH, dEHdt, IEHdt;
float Tavg = NAN, Tvar;
float Havg = NAN, Hvar;
float Hpower, Hduty;
unsigned long t0, Hon, talarm;
byte c, displayMode;
byte key, bri = 255, alarm;

void setup(void) {
#if defined(WDT_TIMEOUT)
  wdt_enable(WDT_TIMEOUT);
#endif
#if defined(SERIAL_LOGGING)
  Serial.begin(115200);
#endif
  pinMode(HEATER, OUTPUT);
  heater(0);
  pinMode(BRIGHTNESS, OUTPUT);
  analogWrite(BRIGHTNESS, bri = 255);
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.print("Incubator 0.5");
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


void loop(void) {
  unsigned long t1 = millis();
  int dt = t1 - t0;

  if (!key)
    key = getKey();

  if (key)
    analogWrite(BRIGHTNESS, bri = 255);

  if (t1 - Hon > Hpower * DELAY)
    heater(0);

  if (Hcontrol && Hcontrol < H_AUTO_COUNT)
    vent.refresh();

  if (dt > DELAY || key) {
    //    beep(2000, 50);
    float T = dht.readTemperature() + T_OFFSET;
    float H = dht.readHumidity();

    if (isnan(T) || T < 10 || T > 100 || isnan(H) || H < 10 || H > 95) {
      heater(0);
      lcd.clear();
      lcd.print("SENSOR ERROR");
      beep(2000, 1000);
      return;
    }

    float dts = dt * 1e-3;

    int fanrpm = fancount * 60 / dts;
    fancount = 0;

    float E0 = ET;
    ET = HWAT * (T - Ts) + (1 - HWAT) * (ET + dETdt * dts); // double exp smoothing (holt winters)
    dETdt = HWBT * (ET - E0) / dts + (1 - HWBT) * dETdt;
    IETdt += ET * dts;
    if (abs(ET) > TI_RESET)
      IETdt = 0;
    float pidT = 1.1765 * (ET + 0.010526 * IETdt + 23.75 * dETdt);
    Hpower = fanrpm > FAN_THRES ? max(0, min(1, -pidT)) : 0;
    heater(Hpower > 0.1);
    Hon = millis();

    E0 = EH;
    EH = HWAH * (H - Hs) + (1 - HWAH) * (EH + dEHdt * dts); // double exp smoothing (holt winters)
    dEHdt = HWBH * (EH - E0) / dts + (1 - HWBH) * dEHdt;
    IEHdt += EH * dts;
    if (abs(EH) > HI_RESET)
      IEHdt = 0;
    float pidH = 0.1176 * (EH + 0.09091 * IEHdt + 2.75 * dEHdt);
    vent.write(pidH * 180);
    if (Hcontrol > 1) {
      if (abs(EH) > H_AUTO_THRES) {
        Hcontrol = 2;
      } else {
        if (Hcontrol < H_AUTO_COUNT) {
          ++Hcontrol;
        } else {
          IEHdt = 0;
        }
      }
    }

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
        if (key & (UP | DOWN)) {
          Ts = max(20, min(50, Ts + (key & UP ? +1 : -1) * 0.1));
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
          Hcontrol = ++Hcontrol % 3;
          IEHdt = 0;
          write_byte(HC_ADDR, Hcontrol);
        }
        lcd.print("Hs=");
        lcd.print(Hs);
        lcd.print("% ");
        lcd.print(Hcontrol == 1 ? "on" : Hcontrol > 1 ? "auto" : "off");
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
    } else {
      alarm &= ~1;
    }

    if (abs(EH) > ALARM_H) {
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
        for (int i = 0; i < 3; ++i) {
          beep(1000, 333);
          beep(1400, 333);
        }
      }
    } else {
      alarm = 0;
      talarm = 0;
    }

#if defined(SERIAL)
    //    Serial.print("free=");    Serial.println(freeMemory());
    Serial.print("up=");
    Serial.println(t1 / 1000);
    Serial.print("dt=");
    Serial.println(dts, 4);
    Serial.println();

    Serial.print("Ts=");
    Serial.println(Ts, 4);
    Serial.print("Tr=");
    Serial.println(T, 4);
    Serial.print("T=");
    Serial.println(Ts + ET, 4);
    Serial.print("ET=");
    Serial.println(ET, 4);
    Serial.print("dETdt=");
    Serial.println(dETdt, 4);
    Serial.print("IETdt=");
    Serial.println(IETdt, 4);
    Serial.print("Tavg=");
    Serial.println(Tavg, 4);
    Serial.print("Tstd=");
    Serial.println(Tstd, 4);
    Serial.print("pidT=");
    Serial.println(pidT, 4);
    Serial.print("Ht=");
    Serial.println(Hpower, 4);
    Serial.print("Hduty=");
    Serial.println(Hduty, 4);
    Serial.println();

    Serial.print("Hs=");
    Serial.println(Hs, 4);
    Serial.print("Hcontrol=");
    Serial.println(Hcontrol);
    Serial.print("Hr=");
    Serial.println(H, 4);
    Serial.print("H=");
    Serial.println(Hs + EH, 4);
    Serial.print("EH=");
    Serial.println(EH, 4);
    Serial.print("dEHdt=");
    Serial.println(dEHdt, 4);
    Serial.print("IEHdt=");
    Serial.println(IEHdt, 4);
    Serial.print("Havg=");
    Serial.println(Havg, 4);
    Serial.print("Hstd=");
    Serial.println(Hstd, 4);
    Serial.print("pidH=");
    Serial.println(pidH, 4);
    Serial.print("V=");
    Serial.println(vent.read() / 180.0, 4);
    Serial.print("Vdeg=");
    Serial.println(vent.read());
    Serial.print("Fan=");
    Serial.println(fanrpm);

    Serial.println();
#endif

    if (key)
      delay(500);
    if (bri)
      analogWrite(BRIGHTNESS, --bri);
    key = 0;
    t0 = t1;
    ++c;
  }
#if defined(WDT_TIMEOUT)
  wdt_reset();
#endif
}



