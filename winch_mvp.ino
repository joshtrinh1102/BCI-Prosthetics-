#include <Servo.h>
#include <math.h>

/*
  SG90 Winch — Spool + Unspool (Absolutely-Works Edition)
  - D9: servo signal
  - D2 -> GND: CLOSE (spool) button, INPUT_PULLUP
  - D3 -> GND: OPEN  (unspool) button, INPUT_PULLUP
  - Non-blocking motion (velocity scheduler with millis)
  - Serial cmds: close [mm], open [mm], home, radius <mm>, limits <min> <max>, invert 0|1, status
  - Tip: Power servo from stable 5V >= 1A; tie grounds with Uno.
*/

// -------- User Defaults (change if needed) --------
const uint8_t PIN_SERVO   = 9;
const uint8_t PIN_CLOSE   = 2;     // to GND
const uint8_t PIN_OPEN    = 3;     // to GND

const float   R_MM_DEFAULT      = 10.0f;   // spool radius (mm)
const float   STEP_MM_DEFAULT   = 5.0f;    // tap step (mm)
const float   HOLD_MM_S_DEFAULT = 25.0f;   // hold creep speed (mm/s)
const float   MOVE_DEG_S        = 360.0f;  // discrete move speed (deg/s)
const int     LIM_MIN_DEFAULT   = 10;      // safe servo min/max (deg) //30
const int     LIM_MAX_DEFAULT   = 170;     // 150
const int     HOME_DEG_DEFAULT  = 180;     // 90
const int     TRIM_DEG          = 2;       // tiny tighten/release nudge

// -------- Timing --------
const uint16_t TICK_MS      = 10;   // motion scheduler tick
const uint16_t DEBOUNCE_MS  = 20;

// -------- State --------
Servo    servo_;
float    posDeg, targetDeg;
float    r_mm       = R_MM_DEFAULT;
float    stepMM     = STEP_MM_DEFAULT;
float    holdVelDeg = 0.0f;           // computed from HOLD_MM_S
bool     invertDir  = false;          // flip winding direction if needed
int      LIM_MIN    = LIM_MIN_DEFAULT;
int      LIM_MAX    = LIM_MAX_DEFAULT;
int      HOME_DEG   = HOME_DEG_DEFAULT;

unsigned long lastTickMs   = 0;
unsigned long lastEdgeMs   = 0;
bool     prevClose = HIGH;            // INPUT_PULLUP idle = HIGH
bool     prevOpen  = HIGH;

float    velDegPerS = 0.0f;           // 0: idle; ±MOVE_DEG_S discrete; ±holdVelDeg creep

// -------- Serial Buffer --------
char     lineBuf[96];
uint8_t  lineLen = 0;

// -------- Utils --------
static inline float clampF(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline int clampI(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
static inline float mmToDeg(float mm) {
  // deg = mm * 180 / (pi * r)
  return (mm * 180.0f) / (3.14159265f * r_mm);
}
static inline float applyDir(float ddeg) {
  return invertDir ? -ddeg : ddeg;
}

void writeServoDeg(float deg) {
  deg = clampF(deg, LIM_MIN, LIM_MAX);
  posDeg = deg;
  servo_.write((int)(deg + 0.5f));
}

// -------- Motion scheduling --------
void setDiscreteTarget(float newTargetDeg, bool addTrim, bool tightening) {
  newTargetDeg = clampF(newTargetDeg, LIM_MIN, LIM_MAX);
  if (addTrim) {
    newTargetDeg += (tightening ? +TRIM_DEG : -TRIM_DEG);
    newTargetDeg = clampF(newTargetDeg, LIM_MIN, LIM_MAX);
  }
  targetDeg  = newTargetDeg;
  velDegPerS = (targetDeg > posDeg) ? +MOVE_DEG_S : -MOVE_DEG_S;
}

void startCreep(bool closing) {
  velDegPerS = closing ? +holdVelDeg : -holdVelDeg;
}

void stopCreep() { velDegPerS = 0.0f; }

void motionTick() {
  unsigned long now = millis();
  if (now - lastTickMs < TICK_MS) return;
  float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;

  if (velDegPerS == 0.0f) return;

  float next = posDeg + velDegPerS * dt;

  // If in discrete move (|vel|==MOVE_DEG_S), stop at target
  bool isDiscrete = (fabsf(fabsf(velDegPerS) - MOVE_DEG_S) < 1e-3f);
  if (isDiscrete) {
    if ((velDegPerS > 0 && next >= targetDeg) || (velDegPerS < 0 && next <= targetDeg)) {
      next = targetDeg;
      velDegPerS = 0.0f;
    }
  }

  // Hard limits (also ends creep)
  if (next <= LIM_MIN) { next = LIM_MIN; velDegPerS = 0.0f; }
  if (next >= LIM_MAX) { next = LIM_MAX; velDegPerS = 0.0f; }

  writeServoDeg(next);
}

// -------- Commands --------
void doHome() {
  setDiscreteTarget(HOME_DEG, false, false);
  Serial.println(F("{\"ok\":true,\"cmd\":\"home\"}"));
}

void doCloseMM(float mm, bool tighten=true) {
  float d = applyDir(mmToDeg(mm));
  setDiscreteTarget(posDeg + d, tighten, /*tightening=*/(d>0));
  Serial.print(F("{\"ok\":true,\"cmd\":\"close\",\"mm\":")); Serial.print(mm,1); Serial.println(F("}"));
}

void doOpenMM(float mm, bool release=true) {
  float d = applyDir(mmToDeg(mm));
  setDiscreteTarget(posDeg - d, release, /*tightening=*/false);
  Serial.print(F("{\"ok\":true,\"cmd\":\"open\",\"mm\":")); Serial.print(mm,1); Serial.println(F("}"));
}

void printStatus() {
  Serial.print(F("{\"status\":{"));
  Serial.print(F("\"posDeg\":"));   Serial.print(posDeg,1); Serial.print(',');
  Serial.print(F("\"r_mm\":"));     Serial.print(r_mm,2);   Serial.print(',');
  Serial.print(F("\"step_mm\":"));  Serial.print(stepMM,1); Serial.print(',');
  Serial.print(F("\"hold_mm_s\":"));Serial.print((holdVelDeg * 3.14159265f * r_mm)/180.0f,1); Serial.print(',');
  Serial.print(F("\"lim_min\":"));  Serial.print(LIM_MIN);  Serial.print(',');
  Serial.print(F("\"lim_max\":"));  Serial.print(LIM_MAX);  Serial.print(',');
  Serial.print(F("\"home\":"));     Serial.print(HOME_DEG); Serial.print(',');
  Serial.print(F("\"invert\":"));   Serial.print(invertDir?1:0);
  Serial.println(F("}}"));
}

// -------- Serial parsing (robust & tiny) --------
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen) {
        // token + up to 2 args
        char *cmd = lineBuf, *a1 = NULL, *a2 = NULL;
        for (uint8_t i=0;i<lineLen;i++){ if (lineBuf[i]==' '){ lineBuf[i]='\0'; a1=&lineBuf[i+1]; break; } }
        if (a1){ for (uint8_t i=0;i<lineLen;i++){ if (a1[i]==' '){ a1[i]='\0'; a2=&a1[i+1]; break; } } }

        if      (!strcmp(cmd,"close")) { float mm = (a1&&*a1)? atof(a1) : stepMM; doCloseMM(mm,true); }
        else if (!strcmp(cmd,"open"))  { float mm = (a1&&*a1)? atof(a1) : stepMM; doOpenMM (mm,true); }
        else if (!strcmp(cmd,"home"))  { doHome(); }
        else if (!strcmp(cmd,"radius")){ if (a1){ r_mm = fmaxf(1.0f, atof(a1)); Serial.println(F("{\"ok\":true,\"set\":\"radius\"}")); } }
        else if (!strcmp(cmd,"limits")){ if (a1&&a2){ LIM_MIN=clampI(atoi(a1),0,180); LIM_MAX=clampI(atoi(a2),0,180); if(LIM_MIN>LIM_MAX){int t=LIM_MIN;LIM_MIN=LIM_MAX;LIM_MAX=t;} Serial.println(F("{\"ok\":true,\"set\":\"limits\"}")); } }
        else if (!strcmp(cmd,"invert")){ if (a1){ invertDir = (atoi(a1)!=0); Serial.println(F("{\"ok\":true,\"set\":\"invert\"}")); } }
        else if (!strcmp(cmd,"status")){ printStatus(); }
        else if (!strcmp(cmd,"step"))  { if (a1){ stepMM = fmaxf(0.5f, atof(a1)); Serial.println(F("{\"ok\":true,\"set\":\"step\"}")); } }
        else if (!strcmp(cmd,"hold"))  { if (a1){ float mmps = fmaxf(1.0f, atof(a1)); holdVelDeg = mmToDeg(mmps); Serial.println(F("{\"ok\":true,\"set\":\"hold_speed\"}")); } }
        else                           { Serial.println(F("{\"ok\":false,\"err\":\"unknown_cmd\"}")); }
      }
      lineLen = 0;
    } else {
      if (lineLen < sizeof(lineBuf)-1) lineBuf[lineLen++] = c;
    }
  }
}

// -------- Buttons (debounced tap + hold) --------
bool edge(uint8_t pin, bool &prev) {
  bool v = digitalRead(pin);
  if (v != prev) {
    unsigned long now = millis();
    if (now - lastEdgeMs >= DEBOUNCE_MS) {
      prev = v; lastEdgeMs = now; return true;
    }
  }
  return false;
}

void handleButtons() {
  // taps → discrete steps
  if (edge(PIN_CLOSE, prevClose) && prevClose == LOW) doCloseMM(stepMM, true);
  if (edge(PIN_OPEN,  prevOpen ) && prevOpen  == LOW) doOpenMM (stepMM, true);

  // holds → creep
  bool closeHeld = (digitalRead(PIN_CLOSE) == LOW);
  bool openHeld  = (digitalRead(PIN_OPEN ) == LOW);

  if (closeHeld && !openHeld) {
    startCreep(true);
  } else if (openHeld && !closeHeld) {
    startCreep(false);
  } else {
    // stop creep only (don’t interrupt discrete move)
    bool isDiscrete = (fabsf(fabsf(velDegPerS) - MOVE_DEG_S) < 1e-3f);
    if (!isDiscrete) stopCreep();
  }
}

// -------- Setup / Loop --------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_CLOSE, INPUT_PULLUP);
  pinMode(PIN_OPEN,  INPUT_PULLUP);

  // compute hold velocity in deg/s from default mm/s
  holdVelDeg = mmToDeg(HOLD_MM_S_DEFAULT);

  servo_.attach(PIN_SERVO);
  posDeg = targetDeg = HOME_DEG_DEFAULT;
  servo_.write(HOME_DEG_DEFAULT);
  lastTickMs = millis();

  Serial.println(F("{\"ready\":true,\"hint\":\"close|open [mm], home, radius <mm>, limits <a> <b>, invert 0|1, status\"}"));
  printStatus();
}

void loop() {
  handleSerial();
  handleButtons();
  motionTick();
}

