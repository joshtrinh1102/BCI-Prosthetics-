#include <Servo.h>
#include <math.h>

/*
  Dual-Mode Winch: Servo OR DC Motor (non-blocking)
  - Preserves your existing servo API and behavior.
  - Adds DC mode: close/open run the motor for a fixed time (default 1000 ms).

  Pins:
    Servo signal -> D9
    CLOSE button -> D2 to GND (INPUT_PULLUP)
    OPEN  button -> D3 to GND (INPUT_PULLUP)
    Mode select  -> D4 (INPUT_PULLUP). LOW=DC, HIGH=Servo when in "mode auto"

    DC Motor driver (TB6612/L9110/L298-style single channel):
      DIR1 -> D5
      DIR2 -> D6
      PWM  -> D10  (enable/speed, PWM)
    Power: Use a proper motor supply; DO NOT power motor from Arduino 5V.
*/

enum Mode { MODE_SERVO = 0, MODE_DC = 1, MODE_AUTO = 2 };

// ======== USER DEFAULTS ========
const uint8_t PIN_SERVO     = 9;
const uint8_t PIN_BTN_CLOSE = 2;
const uint8_t PIN_BTN_OPEN  = 3;
const uint8_t PIN_MODE_SEL  = 4;  // jumper to GND => DC when in MODE_AUTO

// DC driver pins
const uint8_t PIN_DC_DIR1   = 5;
const uint8_t PIN_DC_DIR2   = 6;
const uint8_t PIN_DC_PWM    = 10;

// Servo mechanics
float   R_MM_DEFAULT      = 10.0f;
float   STEP_MM_DEFAULT   = 5.0f;
float   HOLD_MM_S_DEFAULT = 25.0f;
float   MOVE_DEG_S        = 360.0f;
int     LIM_MIN_DEFAULT   = 30;
int     LIM_MAX_DEFAULT   = 150;
int     HOME_DEG_DEFAULT  = 90;
int     TRIM_DEG          = 2;

// DC behavior
uint16_t DC_SPIN_MS_DEFAULT = 1000; // 1 second spin per command
uint8_t  DC_SPEED_DEFAULT   = 200;  // 0..255 PWM

// ======== TIMING ========
const uint16_t TICK_MS     = 10;
const uint16_t DEBOUNCE_MS = 20;

// ======== STATE ========
Servo    servo_;
float    r_mm, stepMM, holdVelDeg;
int      LIM_MIN, LIM_MAX, HOME_DEG;
float    posDeg, targetDeg, velDegPerS;
bool     prevClose = HIGH, prevOpen = HIGH;
unsigned long lastTickMs = 0, lastEdgeMs = 0;

Mode     modeSetting = MODE_SERVO; // default safe: Servo
uint8_t  dcSpeed     = DC_SPEED_DEFAULT;
uint16_t dcSpinMs    = DC_SPIN_MS_DEFAULT;

// DC runtime
bool           dcActive      = false;
unsigned long  dcStopAtMs    = 0;

// Serial buffer
char     lineBuf[96]; uint8_t lineLen = 0;

// ======== UTILS ========
static inline float clampF(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
static inline int   clampI(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }
static inline float mmToDeg(float mm){ return (mm * 180.0f) / (3.14159265f * r_mm); }
static inline float applyDir(float ddeg, bool invert=false){ return invert ? -ddeg : ddeg; }

void logStatus(){
  Serial.print(F("{\"mode\":\""));
  if(modeSetting==MODE_SERVO) Serial.print(F("servo"));
  else if(modeSetting==MODE_DC) Serial.print(F("dc"));
  else Serial.print(F("auto"));
  Serial.print(F("\",\"servo\":{\"pos\":")); Serial.print(posDeg,1);
  Serial.print(F(",\"r_mm\":")); Serial.print(r_mm,2);
  Serial.print(F(",\"step_mm\":")); Serial.print(stepMM,1);
  Serial.print(F(",\"limits\":[\"")); Serial.print(LIM_MIN); Serial.print(F("\",\"")); Serial.print(LIM_MAX); Serial.print(F("\"]}"));
  Serial.print(F(",\"dc\":{\"speed\":")); Serial.print(dcSpeed);
  Serial.print(F(",\"spin_ms\":")); Serial.print(dcSpinMs);
  Serial.println(F("}}"));
}

// ======== MODE RESOLUTION ========
Mode resolveMode(){
  if(modeSetting != MODE_AUTO) return modeSetting;
  pinMode(PIN_MODE_SEL, INPUT_PULLUP);
  bool lowMeansDC = (digitalRead(PIN_MODE_SEL)==LOW);
  return lowMeansDC ? MODE_DC : MODE_SERVO;
}

// ======== SERVO PATH ========
void writeServoDeg(float deg){ deg=clampF(deg,LIM_MIN,LIM_MAX); posDeg=deg; servo_.write((int)(deg+0.5f)); }

void setDiscreteTarget(float newTargetDeg, bool addTrim, bool tightening){
  newTargetDeg = clampF(newTargetDeg, LIM_MIN, LIM_MAX);
  if(addTrim){
    newTargetDeg += (tightening? +TRIM_DEG : -TRIM_DEG);
    newTargetDeg = clampF(newTargetDeg, LIM_MIN, LIM_MAX);
  }
  targetDeg  = newTargetDeg;
  velDegPerS = (targetDeg > posDeg) ? +MOVE_DEG_S : -MOVE_DEG_S;
}

void startCreep(bool closing){ velDegPerS = closing ? +holdVelDeg : -holdVelDeg; }
void stopCreep(){ velDegPerS = 0.0f; }

void motionTick(){
  unsigned long now = millis();
  if(now - lastTickMs < TICK_MS) return;
  float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;

  if(velDegPerS == 0.0f) return;

  float next = posDeg + velDegPerS * dt;
  bool isDiscrete = (fabsf(fabsf(velDegPerS) - MOVE_DEG_S) < 1e-3f);

  if(isDiscrete){
    if((velDegPerS>0 && next>=targetDeg) || (velDegPerS<0 && next<=targetDeg)){
      next = targetDeg;
      velDegPerS = 0.0f;
    }
  }
  if(next <= LIM_MIN){ next=LIM_MIN; velDegPerS=0.0f; }
  if(next >= LIM_MAX){ next=LIM_MAX; velDegPerS=0.0f; }

  writeServoDeg(next);
}

// ======== DC PATH ========
void dcStop(){
  dcActive = false;
  analogWrite(PIN_DC_PWM, 0);
  digitalWrite(PIN_DC_DIR1, LOW);
  digitalWrite(PIN_DC_DIR2, LOW);
}

void dcStart(bool closing){
  // closing = CW (DIR1=HIGH, DIR2=LOW), opening = CCW (DIR1=LOW, DIR2=HIGH)
  if(closing){
    digitalWrite(PIN_DC_DIR1, HIGH);
    digitalWrite(PIN_DC_DIR2, LOW);
  } else {
    digitalWrite(PIN_DC_DIR1, LOW);
    digitalWrite(PIN_DC_DIR2, HIGH);
  }
  analogWrite(PIN_DC_PWM, dcSpeed);
  dcActive = true;
  dcStopAtMs = millis() + dcSpinMs;
}

void dcTick(){
  if(dcActive && millis() >= dcStopAtMs){
    dcStop();
  }
}

// ======== COMMAND ADAPTERS (same API) ========
void doHome(){
  if(resolveMode()==MODE_SERVO){
    setDiscreteTarget(HOME_DEG_DEFAULT, false, false);
  }else{
    // DC doesn't have a position; do nothing
  }
  Serial.println(F("{\"ok\":true,\"cmd\":\"home\"}"));
}

void doCloseMM(float mm, bool tighten=true){
  if(resolveMode()==MODE_SERVO){
    float d = applyDir(mmToDeg(mm));
    setDiscreteTarget(posDeg + d, tighten, (d>0));
  } else {
    (void)mm; // ignored in DC mode
    dcStart(true); // spin forward for dcSpinMs
  }
  Serial.print(F("{\"ok\":true,\"cmd\":\"close\",\"arg\":")); Serial.print(mm,1); Serial.println(F("}"));
}

void doOpenMM(float mm, bool release=true){
  if(resolveMode()==MODE_SERVO){
    float d = applyDir(mmToDeg(mm));
    setDiscreteTarget(posDeg - d, release, false);
  } else {
    (void)mm;
    dcStart(false); // spin reverse for dcSpinMs
  }
  Serial.print(F("{\"ok\":true,\"cmd\":\"open\",\"arg\":")); Serial.print(mm,1); Serial.println(F("}"));
}

// ======== BUTTONS ========
bool edge(uint8_t pin, bool &prev){
  bool v = digitalRead(pin);
  if(v!=prev){
    unsigned long now=millis();
    if(now - lastEdgeMs >= DEBOUNCE_MS){ prev=v; lastEdgeMs=now; return true; }
  }
  return false;
}

void handleButtons(){
  // taps
  if(edge(PIN_BTN_CLOSE, prevClose) && prevClose==LOW) doCloseMM(stepMM, true);
  if(edge(PIN_BTN_OPEN,  prevOpen ) && prevOpen ==LOW) doOpenMM (stepMM, true);

  // holds (servo only)
  if(resolveMode()==MODE_SERVO){
    bool closeHeld = (digitalRead(PIN_BTN_CLOSE) == LOW);
    bool openHeld  = (digitalRead(PIN_BTN_OPEN ) == LOW);
    if(closeHeld && !openHeld) startCreep(true);
    else if(openHeld && !closeHeld) startCreep(false);
    else {
      bool isDiscrete = (fabsf(fabsf(velDegPerS) - MOVE_DEG_S) < 1e-3f);
      if(!isDiscrete) stopCreep();
    }
  }
}

// ======== SERIAL ========
void handleSerial(){
  while(Serial.available()){
    char c = Serial.read();
    if(c=='\r') continue;
    if(c=='\n'){
      lineBuf[lineLen]='\0';
      if(lineLen){
        char* cmd=lineBuf; char* a1=nullptr; char* a2=nullptr;
        for(uint8_t i=0;i<lineLen;i++){ if(lineBuf[i]==' '){ lineBuf[i]='\0'; a1=&lineBuf[i+1]; break; } }
        if(a1){ for(uint8_t i=0;i<lineLen;i++){ if(a1[i]==' '){ a1[i]='\0'; a2=&a1[i+1]; break; } } }

        if      (!strcmp(cmd,"close"))    { float mm=a1&&*a1? atof(a1): stepMM; doCloseMM(mm,true); }
        else if (!strcmp(cmd,"open"))     { float mm=a1&&*a1? atof(a1): stepMM; doOpenMM (mm,true); }
        else if (!strcmp(cmd,"home"))     { doHome(); }
        else if (!strcmp(cmd,"status"))   { logStatus(); }
        else if (!strcmp(cmd,"radius"))   { if(a1){ r_mm = fmaxf(1.0f, atof(a1)); Serial.println(F("{\"ok\":true,\"set\":\"radius\"}")); } }
        else if (!strcmp(cmd,"limits"))   { if(a1&&a2){ LIM_MIN=clampI(atoi(a1),0,180); LIM_MAX=clampI(atoi(a2),0,180); if(LIM_MIN>LIM_MAX){int t=LIM_MIN;LIM_MIN=LIM_MAX;LIM_MAX=t;} Serial.println(F("{\"ok\":true,\"set\":\"limits\"}")); } }
        else if (!strcmp(cmd,"step"))     { if(a1){ stepMM = fmaxf(0.5f, atof(a1)); Serial.println(F("{\"ok\":true,\"set\":\"step\"}")); } }
        else if (!strcmp(cmd,"hold"))     { if(a1){ float mmps = fmaxf(1.0f, atof(a1)); holdVelDeg = mmToDeg(mmps); Serial.println(F("{\"ok\":true,\"set\":\"hold_speed\"}")); } }
        else if (!strcmp(cmd,"mode"))     { // mode servo|dc|auto
          if(a1){
            if(!strcmp(a1,"servo")) modeSetting=MODE_SERVO;
            else if(!strcmp(a1,"dc")) modeSetting=MODE_DC;
            else modeSetting=MODE_AUTO;
            Serial.println(F("{\"ok\":true,\"set\":\"mode\"}"));
          }
        }
        else if (!strcmp(cmd,"dcspeed"))  { if(a1){ dcSpeed = (uint8_t)clampI(atoi(a1),0,255); Serial.println(F("{\"ok\":true,\"set\":\"dcspeed\"}")); } }
        else if (!strcmp(cmd,"dctime"))   { if(a1){ dcSpinMs = (uint16_t)clampI(atoi(a1),50,10000); Serial.println(F("{\"ok\":true,\"set\":\"dctime\"}")); } }
        else                              { Serial.println(F("{\"ok\":false,\"err\":\"unknown_cmd\"}")); }
      }
      lineLen=0;
    } else {
      if(lineLen < sizeof(lineBuf)-1) lineBuf[lineLen++]=c;
    }
  }
}

// ======== SETUP / LOOP ========
void setup(){
  Serial.begin(115200);

  pinMode(PIN_BTN_CLOSE, INPUT_PULLUP);
  pinMode(PIN_BTN_OPEN,  INPUT_PULLUP);
  pinMode(PIN_MODE_SEL,  INPUT_PULLUP);

  // DC pins
  pinMode(PIN_DC_DIR1, OUTPUT);
  pinMode(PIN_DC_DIR2, OUTPUT);
  pinMode(PIN_DC_PWM,  OUTPUT);
  dcStop();

  // Servo defaults
  r_mm        = R_MM_DEFAULT;
  stepMM      = STEP_MM_DEFAULT;
  holdVelDeg  = mmToDeg(HOLD_MM_S_DEFAULT);
  LIM_MIN     = LIM_MIN_DEFAULT;
  LIM_MAX     = LIM_MAX_DEFAULT;
  posDeg = targetDeg = HOME_DEG_DEFAULT;

  // Attach servo (safe even if unused in DC mode)
  servo_.attach(PIN_SERVO);
  servo_.write(HOME_DEG_DEFAULT);

  lastTickMs = millis();

  Serial.println(F("{\"ready\":true,\"hint\":\"close|open [mm], mode servo|dc|auto, dcspeed <0-255>, dctime <ms>, status\"}"));
  logStatus();
}

void loop(){
  handleSerial();
  handleButtons();

  if(resolveMode()==MODE_SERVO){
    motionTick();
  } else {
    dcTick();
  }
}
