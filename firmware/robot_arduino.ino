/*
 * =====================================================================
 *   ROBOT_ARDUINO  -  Birlesik quadruped + turret firmware
 *   (leg_calibrate.ino + turret_arduino.ino tek sketch)
 * =====================================================================
 *
 *   10 servo + 1 lazer pin tek Arduino'da:
 *       Leg servolari (8):
 *           FL_yaw=D2   FL_lift=D3
 *           FR_yaw=D4   FR_lift=D5
 *           BL_yaw=D7   BL_lift=D8
 *           BR_yaw=D11  BR_lift=D12
 *       Turret (2 + 1):
 *           Pan  = D10
 *           Tilt = D9
 *           Laser= D13   (built-in LED ile ayni pin)
 *
 *   Iki protokol ayni hat uzerinden (115200 baud, LF satir sonu):
 *
 *   A) Turret komutu (avci_taret.py tarafindan kullanilir):
 *       "PAN_X10,TILT_X10,LASER\n"      ornek:  "900,900,0\n"
 *       - 3 sayi, virgulle ayrilmis
 *       - PAN_X10, TILT_X10: derecenin onda biri (0..1800), 90.0 -> 900
 *       - LASER: 0 veya 1
 *       Servo writeMicroseconds ile surulur -> 0.1 derece etkili cozunurluk.
 *
 *   B) Leg komutu (walk.py / WalkController tarafindan kullanilir):
 *       "FL_Y 95\n"             tek servoya mutlak aci
 *       "FL_L 160\n"
 *       "hold\n"                tum bacaklar per-leg HOME pozisyonu
 *       "sweep\n"               kalibrasyon icin her servo ±15 deg
 *       "all_y 80\n"            tum yaw'lar
 *       "all_l 160\n"           tum lift'ler
 *       "us FL_Y 1500\n"        mikrosaniye pulse direct
 *       "range FL_Y\n"          0..180 sirayla
 *       "test FL_Y\n"           20->90->160->home ramp
 *       "trim FL_L +5\n"        per-servo trim (yazilim ofseti)
 *       "enable FL\n"           bacagi aktif et
 *       "disable FL\n"          bacagi devre disi
 *       "?\n"                   durum tablosu
 *
 *   Dispatch: gelen satirda virgul varsa A); aksi takdirde B).
 *
 *   Watchdog: 500 ms turret komutu gelmezse SADECE lazer kapatilir.
 *   Bacak pozisyonlari ASLA otomatik HOLD'a dusmez -- WalkController
 *   yurume state'ini gercek zamanli yonetir, kendi "hold" komutunu
 *   gondererek bacaklari home'a alir.
 *
 *   Guc:
 *       Pi 5V/5A USB-C        -> Pi
 *       Arduino               -> Pi USB
 *       Servo rayi            -> AYRI 5-6V adaptor + 1000 uF cap
 *                                (8 leg + 2 turret + 1 laser = ayri PSU sart!)
 *       Ortak GND             -> Pi + Arduino + servo PSU hepsi birlesik.
 * =====================================================================
 */

#include <Servo.h>

// ============ Turret ==================================================
// YENI PROTOKOL: pan ve tilt artik DERECENIN ONDA BIRI olarak gelir.
// "900,900,0" = 90.0 derece pan, 90.0 derece tilt, lazer kapali.
// Aralik: 0..1800 (0.0..180.0 derece).
// Servo komutu writeMicroseconds ile gonderiliyor -> 0.1 derece etkili
// cozunurluk (1 derece yerine ~0.1 derece). Daha kucuk lock_tol mumkun.
#define PAN_PIN          10
#define TILT_PIN         9
#define LASER_PIN        13
#define WATCHDOG_MS      500
#define TURRET_MIN_US    500    // 0.0 derece servo pulse
#define TURRET_MAX_US    2500   // 180.0 derece servo pulse

Servo panServo, tiltServo;
int   panAngleX10   = 900;       // pan x10 (=90.0 derece)
int   tiltAngleX10  = 900;       // tilt x10
int   laserState    = 0;
unsigned long lastTurretMs = 0;

// ============ Leg =====================================================
enum Leg : uint8_t { FL = 0, FR = 1, BL = 2, BR = 3 };

const uint8_t YAW_PIN[4]  = {  2,  4,  7, 11 };
const uint8_t LIFT_PIN[4] = {  3,  5,  8, 12 };
const char*   LEG_NAME[4] = { "FL", "FR", "BL", "BR" };

// >>> BURAYI DEGISTIR: hangi bacaklar bagli? <<<
bool LEG_ENABLED[4] = { true, true, true, true };

//                            FL   FR   BL   BR
const int YAW_HOME[4]  = {  50, 100,  80,  80 };
const int LIFT_HOME[4] = { 170, 175, 177, 175};

int TRIM_YAW[4]  = { 0, 0, 0, 0 };
int TRIM_LIFT[4] = { 0, 0, 0, 0 };

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

Servo yawServo[4];
Servo liftServo[4];

int yawAngle[4]  = { YAW_HOME[0],  YAW_HOME[1],  YAW_HOME[2],  YAW_HOME[3]  };
int liftAngle[4] = { LIFT_HOME[0], LIFT_HOME[1], LIFT_HOME[2], LIFT_HOME[3] };

// ============ Shared buffer ===========================================
#define BUF_MAX 40
String buf;

// ============ Helpers =================================================
static inline int clampAngle(int a) {
  if (a < 0)   return 0;
  if (a > 180) return 180;
  return a;
}

static inline int clampX10(int a) {
  if (a < 0)    return 0;
  if (a > 1800) return 1800;
  return a;
}

// 0..1800 (0.0..180.0 derece) -> TURRET_MIN_US..TURRET_MAX_US
static inline int x10ToUs(int x10) {
  long us = (long)TURRET_MIN_US +
            ((long)(TURRET_MAX_US - TURRET_MIN_US) * (long)x10) / 1800L;
  return (int)us;
}

void applyTurret() {
  panServo.writeMicroseconds(x10ToUs(clampX10(panAngleX10)));
  tiltServo.writeMicroseconds(x10ToUs(clampX10(tiltAngleX10)));
  digitalWrite(LASER_PIN, laserState ? HIGH : LOW);
}

// "FL"/"FR"/"BL"/"BR" -> 0..3, -1 hata
int parseLeg(const String& s) {
  if (s.length() < 2) return -1;
  String u = s; u.toUpperCase();
  if (u.startsWith("FL")) return FL;
  if (u.startsWith("FR")) return FR;
  if (u.startsWith("BL")) return BL;
  if (u.startsWith("BR")) return BR;
  return -1;
}

bool parseLegAxis(const String& tok, int& leg, bool& isYaw) {
  leg = parseLeg(tok);
  if (leg < 0) return false;
  if (tok.endsWith("_Y") || tok.endsWith("_y")) { isYaw = true;  return true; }
  if (tok.endsWith("_L") || tok.endsWith("_l")) { isYaw = false; return true; }
  return false;
}

void enableLeg(uint8_t leg) {
  if (leg >= 4) return;
  if (LEG_ENABLED[leg] && yawServo[leg].attached()) return;
  LEG_ENABLED[leg] = true;
  yawServo[leg].attach(YAW_PIN[leg], SERVO_MIN_US, SERVO_MAX_US);
  yawServo[leg].write(clampAngle(YAW_HOME[leg] + TRIM_YAW[leg]));
  delay(60);
  liftServo[leg].attach(LIFT_PIN[leg], SERVO_MIN_US, SERVO_MAX_US);
  liftServo[leg].write(clampAngle(LIFT_HOME[leg] + TRIM_LIFT[leg]));
  delay(60);
  yawAngle[leg]  = YAW_HOME[leg]  + TRIM_YAW[leg];
  liftAngle[leg] = LIFT_HOME[leg] + TRIM_LIFT[leg];
}

void disableLeg(uint8_t leg) {
  if (leg >= 4) return;
  yawServo[leg].detach();
  liftServo[leg].detach();
  LEG_ENABLED[leg] = false;
}

void holdAll() {
  for (uint8_t i = 0; i < 4; i++) {
    if (!LEG_ENABLED[i]) continue;
    yawAngle[i]  = YAW_HOME[i]  + TRIM_YAW[i];
    liftAngle[i] = LIFT_HOME[i] + TRIM_LIFT[i];
    yawServo[i].write(clampAngle(yawAngle[i]));
    liftServo[i].write(clampAngle(liftAngle[i]));
  }
}

void sweepAll() {
  for (uint8_t i = 0; i < 4; i++) {
    if (!LEG_ENABLED[i]) continue;
    yawServo[i].write(YAW_HOME[i] - 15);  delay(300);
    yawServo[i].write(YAW_HOME[i] + 15);  delay(300);
    yawServo[i].write(YAW_HOME[i] + TRIM_YAW[i]);  delay(400);
    liftServo[i].write(LIFT_HOME[i] - 15); delay(300);
    liftServo[i].write(LIFT_HOME[i] + 15); delay(300);
    liftServo[i].write(LIFT_HOME[i] + TRIM_LIFT[i]); delay(400);
  }
}

void writeUs(uint8_t leg, bool isYaw, int us) {
  if (leg >= 4 || !LEG_ENABLED[leg]) return;
  if (us < 400)  us = 400;
  if (us > 2600) us = 2600;
  Servo& s = isYaw ? yawServo[leg] : liftServo[leg];
  s.writeMicroseconds(us);
}

void rangeSweep(uint8_t leg, bool isYaw) {
  if (leg >= 4 || !LEG_ENABLED[leg]) return;
  Servo& s = isYaw ? yawServo[leg] : liftServo[leg];
  const int steps[] = { 0, 30, 60, 90, 120, 150, 180 };
  for (uint8_t i = 0; i < 7; i++) {
    s.write(steps[i]);
    delay(800);
  }
  int home = isYaw ? YAW_HOME[leg] : LIFT_HOME[leg];
  int trim = isYaw ? TRIM_YAW[leg] : TRIM_LIFT[leg];
  s.write(clampAngle(home + trim));
}

void testServo(uint8_t leg, bool isYaw) {
  if (leg >= 4 || !LEG_ENABLED[leg]) return;
  Servo& s    = isYaw ? yawServo[leg] : liftServo[leg];
  int    home = isYaw ? YAW_HOME[leg] : LIFT_HOME[leg];
  int    trim = isYaw ? TRIM_YAW[leg] : TRIM_LIFT[leg];
  s.write(20);  delay(600);
  s.write(90);  delay(600);
  s.write(160); delay(600);
  s.write(clampAngle(home + trim));
}

void printState() {
  Serial.println(F("---- Servo durumu ----"));
  for (uint8_t i = 0; i < 4; i++) {
    Serial.print(LEG_NAME[i]);
    Serial.print(LEG_ENABLED[i] ? F(" [ON ]") : F(" [off]"));
    Serial.print(F("  yaw="));  Serial.print(yawAngle[i]);
    Serial.print(F("  lift=")); Serial.println(liftAngle[i]);
  }
  Serial.print(F("Turret pan="));
  Serial.print(panAngleX10 / 10); Serial.print('.'); Serial.print(panAngleX10 % 10);
  Serial.print(F(" tilt="));
  Serial.print(tiltAngleX10 / 10); Serial.print('.'); Serial.print(tiltAngleX10 % 10);
  Serial.print(F(" laser="));     Serial.println(laserState);
}

bool parseSingleLeg(const String& token, const String& valStr) {
  int leg; bool isYaw;
  if (!parseLegAxis(token, leg, isYaw)) return false;
  if (!LEG_ENABLED[leg]) return true;
  int v = clampAngle(valStr.toInt());
  if (isYaw) { yawAngle[leg]  = v; yawServo[leg].write(v);  }
  else       { liftAngle[leg] = v; liftServo[leg].write(v); }
  return true;
}

// ============ Turret komutu: "PAN_X10,TILT_X10,LASER" =================
// PAN_X10 / TILT_X10: derecenin onda biri (0..1800)
//   90.0 derece -> 900,  100.5 derece -> 1005
// LASER: 0 veya 1
bool parseTurretCommand(const String& line) {
  int c1 = line.indexOf(',');
  if (c1 < 0) return false;
  int c2 = line.indexOf(',', c1 + 1);
  if (c2 < 0) return false;

  int p = line.substring(0, c1).toInt();
  int t = line.substring(c1 + 1, c2).toInt();
  int l = line.substring(c2 + 1).toInt();

  panAngleX10  = clampX10(p);
  tiltAngleX10 = clampX10(t);
  laserState   = (l != 0) ? 1 : 0;
  lastTurretMs = millis();
  applyTurret();
  return true;
}

// ============ Leg komutu: kelimeler / tek kelime =======================
void handleLegLine(String line) {
  if (line.length() == 0) return;
  if (line.equalsIgnoreCase("hold"))  { holdAll();    return; }
  if (line.equalsIgnoreCase("sweep")) { sweepAll();   return; }
  if (line == "?")                    { printState(); return; }

  int sp1 = line.indexOf(' ');
  if (sp1 < 0) return;
  String cmd  = line.substring(0, sp1);
  String rest = line.substring(sp1 + 1); rest.trim();

  if (cmd.equalsIgnoreCase("enable")) {
    int leg = parseLeg(rest);
    if (leg >= 0) enableLeg(leg);
    return;
  }
  if (cmd.equalsIgnoreCase("disable")) {
    int leg = parseLeg(rest);
    if (leg >= 0) disableLeg(leg);
    return;
  }
  if (cmd.equalsIgnoreCase("test")) {
    int leg; bool isYaw;
    if (parseLegAxis(rest, leg, isYaw)) testServo(leg, isYaw);
    return;
  }
  if (cmd.equalsIgnoreCase("range")) {
    int leg; bool isYaw;
    if (parseLegAxis(rest, leg, isYaw)) rangeSweep(leg, isYaw);
    return;
  }
  if (cmd.equalsIgnoreCase("us")) {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) return;
    String tok = rest.substring(0, sp2);
    String val = rest.substring(sp2 + 1);
    int leg; bool isYaw;
    if (parseLegAxis(tok, leg, isYaw)) writeUs(leg, isYaw, val.toInt());
    return;
  }
  if (cmd.equalsIgnoreCase("trim")) {
    int sp2 = rest.indexOf(' ');
    if (sp2 < 0) return;
    String tok = rest.substring(0, sp2);
    String val = rest.substring(sp2 + 1);
    int leg; bool isYaw;
    if (!parseLegAxis(tok, leg, isYaw)) return;
    int t = val.toInt();
    if (isYaw) TRIM_YAW[leg]  = t;
    else       TRIM_LIFT[leg] = t;
    if (LEG_ENABLED[leg]) holdAll();
    return;
  }
  if (cmd.equalsIgnoreCase("all_y")) {
    int v = clampAngle(rest.toInt());
    for (uint8_t i = 0; i < 4; i++) {
      if (!LEG_ENABLED[i]) continue;
      yawAngle[i] = v; yawServo[i].write(v);
    }
    return;
  }
  if (cmd.equalsIgnoreCase("all_l")) {
    int v = clampAngle(rest.toInt());
    for (uint8_t i = 0; i < 4; i++) {
      if (!LEG_ENABLED[i]) continue;
      liftAngle[i] = v; liftServo[i].write(v);
    }
    return;
  }
  // "FL_Y 90"
  parseSingleLeg(cmd, rest);
}

// ============ Dispatch: virgul varsa turret, yoksa leg =================
void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  if (line.indexOf(',') >= 0) {
    parseTurretCommand(line);
  } else {
    handleLegLine(line);
  }
}

// ============ Arduino lifecycle =======================================
void setup() {
  Serial.begin(115200);
  buf.reserve(BUF_MAX + 1);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  // Turret servolari - writeMicroseconds icin acik min/max
  panServo.attach(PAN_PIN,  TURRET_MIN_US, TURRET_MAX_US);  delay(80);
  tiltServo.attach(TILT_PIN, TURRET_MIN_US, TURRET_MAX_US); delay(80);
  panAngleX10 = 900; tiltAngleX10 = 900; laserState = 0;
  applyTurret();

  // Leg servolari (attach + hemen home pozisyonu)
  for (uint8_t i = 0; i < 4; i++) {
    if (!LEG_ENABLED[i]) continue;
    yawServo[i].attach(YAW_PIN[i], SERVO_MIN_US, SERVO_MAX_US);
    yawServo[i].write(clampAngle(YAW_HOME[i] + TRIM_YAW[i]));
    delay(80);
    liftServo[i].attach(LIFT_PIN[i], SERVO_MIN_US, SERVO_MAX_US);
    liftServo[i].write(clampAngle(LIFT_HOME[i] + TRIM_LIFT[i]));
    delay(80);
    yawAngle[i]  = YAW_HOME[i]  + TRIM_YAW[i];
    liftAngle[i] = LIFT_HOME[i] + TRIM_LIFT[i];
  }

  lastTurretMs = millis();
  Serial.println(F("==== ROBOT (leg+turret) READY ===="));
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(buf);
      buf = "";
    } else if (c != '\r') {
      if (buf.length() < BUF_MAX) buf += c;
      else                        buf = "";
    }
  }

  // Watchdog: 500 ms turret komutu yoksa SADECE lazeri kapat.
  // Bacaklar son komut pozisyonunda kalir, otomatik HOLD yok.
  if ((millis() - lastTurretMs) > WATCHDOG_MS) {
    if (laserState != 0) {
      laserState = 0;
      digitalWrite(LASER_PIN, LOW);
    }
  }
}
