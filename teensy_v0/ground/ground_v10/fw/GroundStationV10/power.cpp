#include "power.h"
#include "sdlog.h"
#include "ui.h"

// RS-485 master protocol (mirrors arduino_remote_module)

HardwareSerial &pwrSerial = PWR_RS485_SERIAL;

// Timing
// TX period kept safely below Power module's LINK_TIMEOUT_MS (300 ms).
static const uint16_t TX_PERIOD_MS   = 150;
static const uint16_t POLL_PERIOD_MS = 300;
static const uint16_t LINK_FRESH_MS  = 500;
static const uint16_t FAST_BLINK_MS  = 120;
static const uint32_t FIRE_MAX_MS    = 10000UL;

// State from Power module
volatile uint32_t lastRxMs = 0;

uint32_t lastTxMs=0, lastPollMs=0, lastBlinkMs=0, lastStatusMs=0;
uint32_t tRate=0;
uint16_t rxCount=0, rxCountLast=0;
uint16_t pwr_rxRate=0;
bool blink=false;
uint8_t seq=0;

bool pwr_armA_seen=false, pwr_onA=false, pwr_armB_seen=false, pwr_onB=false;
bool pwr_key_ok=false, pwr_presA=false, pwr_presB=false;
bool pwr_faultAny=false;

uint8_t pwr_vbat_x10=0, pwr_ia_x10=0, pwr_ib_x10=0;
float   pwr_localVbat=0.0f;
bool    pwr_anyArmed=false;
uint32_t pwr_lastArmOnMs=0;

// Safety: START only valid if START was released after arming
static bool okToIgniteA=false;
static bool okToIgniteB=false;
static bool prevArmA=false;
static bool prevArmB=false;
static bool prevStartAButton=false;
static bool prevStartBButton=false;

// Master-side fire timeouts
static bool m_fireTimeoutA=false;
static bool m_fireTimeoutB=false;
static uint32_t m_tStartA=0;
static uint32_t m_tStartB=0;
static bool prevDesiredA=false;
static bool prevDesiredB=false;

static bool pwrFireLogActive=false;
static uint32_t pwrFireLogStartMs=0;
static uint32_t pwrFireLogLastMs=0;
static uint16_t pwrFireLogEvent=0;
static uint16_t pwrFireLogSample=0;
static float pwrFirePeakA=0.0f;
static float pwrFirePeakB=0.0f;
static char pwrFireEventName[8] = "NONE";

bool power_link_fresh() {
    return (millis() - lastStatusMs) < LINK_FRESH_MS;
}

// --- Local battery read (e.g., 3.7V via 100k/100k divider) ---
static float readLocalVbat() {
    int raw = analogRead(PWR_VBAT_PIN);
    float v_pin = (float)raw * ADC_REF_V / ADC_MAX_COUNTS;
    float v_batt = v_pin * (GND_VBAT_R1_OHMS + GND_VBAT_R2_OHMS) / GND_VBAT_R2_OHMS;
    return v_batt * GND_VBAT_CAL_FACTOR;
}

// CRC-8 Dallas/Maxim
static uint8_t crc8(const uint8_t* d, uint8_t len){
  uint8_t c=0;
  for(uint8_t i=0;i<len;i++){
    uint8_t in=d[i];
    for(uint8_t b=0;b<8;b++){
      uint8_t mix=(c ^ in) & 0x01;
      c >>= 1;
      if(mix) c ^= 0x8C;
      in >>= 1;
    }
  }
  return c;
}

static inline void rsWrite(const uint8_t* buf, uint8_t n){
  digitalWrite(PWR_RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(PWR_DE_PRE_US);
  pwrSerial.write(buf, n);
  pwrSerial.flush();
  delayMicroseconds(PWR_DE_POST_US);
  digitalWrite(PWR_RS485_DE_RE_PIN, LOW);
}

static void sendMasterFrame(bool poll, bool armA_sw, bool startA_ok, bool armB_sw, bool startB_ok){
  uint8_t buf[6];
  buf[0]=0xAA; buf[1]=0x55;
  buf[2]= poll ? 0x02 : 0x01;
  buf[3]= seq++;
  uint8_t bits = (armA_sw?1:0) | (startA_ok?2:0) | (armB_sw?4:0) | (startB_ok?8:0);
  buf[4]= bits;
  buf[5]= crc8(&buf[2], 3);
  rsWrite(buf, sizeof(buf));
}

// ----- Status RX from Power -----
enum RxState { RS_WAIT_55, RS_WAIT_AA, RS_PAYLOAD, RS_CRC };
static RxState rxState=RS_WAIT_55;
static uint8_t rxBuf[6];
static uint8_t rxIdx=0;

static void logPowerStartEvent(const char *eventName,
                               bool armA_sw,
                               bool startA_btn,
                               bool startA_ok,
                               bool armB_sw,
                               bool startB_btn,
                               bool startB_ok) {
  char line[256];
  snprintf(line, sizeof(line),
           "PWR_START,event=%s,ms=%lu,ign_v=%.1f,gnd_v=%.2f,ia=%.1f,ib=%.1f,"
           "key=%d,presA=%d,presB=%d,fault=%d,"
           "armA_sw=%d,startA_btn=%d,startA_ok=%d,armA_seen=%d,onA=%d,"
           "armB_sw=%d,startB_btn=%d,startB_ok=%d,armB_seen=%d,onB=%d,link=%d",
           eventName,
           (unsigned long)millis(),
           pwr_vbat_x10 / 10.0f,
           pwr_localVbat,
           pwr_ia_x10 / 10.0f,
           pwr_ib_x10 / 10.0f,
           pwr_key_ok ? 1 : 0,
           pwr_presA ? 1 : 0,
           pwr_presB ? 1 : 0,
           pwr_faultAny ? 1 : 0,
           armA_sw ? 1 : 0,
           startA_btn ? 1 : 0,
           startA_ok ? 1 : 0,
           pwr_armA_seen ? 1 : 0,
           pwr_onA ? 1 : 0,
           armB_sw ? 1 : 0,
           startB_btn ? 1 : 0,
           startB_ok ? 1 : 0,
           pwr_armB_seen ? 1 : 0,
           pwr_onB ? 1 : 0,
           power_link_fresh() ? 1 : 0);
  sdlog_write(line);
}

static void beginPowerFireLog(const char *eventName) {
  pwrFireLogActive = true;
  pwrFireLogStartMs = millis();
  pwrFireLogLastMs = 0;
  pwrFireLogEvent++;
  pwrFireLogSample = 0;
  pwrFirePeakA = 0.0f;
  pwrFirePeakB = 0.0f;
  strncpy(pwrFireEventName, eventName, sizeof(pwrFireEventName) - 1);
  pwrFireEventName[sizeof(pwrFireEventName) - 1] = '\0';
}

static void logPowerFireSample(bool armA_sw,
                               bool startA_btn,
                               bool startA_ok,
                               bool armB_sw,
                               bool startB_btn,
                               bool startB_ok) {
  const float ia = pwr_ia_x10 / 10.0f;
  const float ib = pwr_ib_x10 / 10.0f;
  if (ia > pwrFirePeakA) pwrFirePeakA = ia;
  if (ib > pwrFirePeakB) pwrFirePeakB = ib;

  char line[320];
  snprintf(line, sizeof(line),
           "PWR_FIRE,event=%s,id=%u,sample=%u,ms=%lu,dt_ms=%lu,"
           "ign_v=%.1f,gnd_v=%.2f,ia=%.1f,ib=%.1f,peak_ia=%.1f,peak_ib=%.1f,"
           "key=%d,presA=%d,presB=%d,fault=%d,"
           "armA_sw=%d,startA_btn=%d,startA_ok=%d,armA_seen=%d,onA=%d,"
           "armB_sw=%d,startB_btn=%d,startB_ok=%d,armB_seen=%d,onB=%d,"
           "link=%d,rx_rate=%u",
           pwrFireEventName,
           (unsigned int)pwrFireLogEvent,
           (unsigned int)pwrFireLogSample++,
           (unsigned long)millis(),
           (unsigned long)(millis() - pwrFireLogStartMs),
           pwr_vbat_x10 / 10.0f,
           pwr_localVbat,
           ia,
           ib,
           pwrFirePeakA,
           pwrFirePeakB,
           pwr_key_ok ? 1 : 0,
           pwr_presA ? 1 : 0,
           pwr_presB ? 1 : 0,
           pwr_faultAny ? 1 : 0,
           armA_sw ? 1 : 0,
           startA_btn ? 1 : 0,
           startA_ok ? 1 : 0,
           pwr_armA_seen ? 1 : 0,
           pwr_onA ? 1 : 0,
           armB_sw ? 1 : 0,
           startB_btn ? 1 : 0,
           startB_ok ? 1 : 0,
           pwr_armB_seen ? 1 : 0,
           pwr_onB ? 1 : 0,
           power_link_fresh() ? 1 : 0,
           (unsigned int)pwr_rxRate);
  sdlog_write_now(line);
}

static void updatePowerFireLog(bool armA_sw,
                               bool startA_btn,
                               bool startA_ok,
                               bool armB_sw,
                               bool startB_btn,
                               bool startB_ok) {
  if (!pwrFireLogActive) return;

  const uint32_t now = millis();
  if ((uint32_t)(now - pwrFireLogStartMs) > PWR_FIRE_LOG_WINDOW_MS) {
    pwrFireLogActive = false;
    return;
  }

  if (pwrFireLogLastMs == 0 ||
      (uint32_t)(now - pwrFireLogLastMs) >= PWR_FIRE_LOG_MS) {
    pwrFireLogLastMs = now;
    logPowerFireSample(armA_sw, startA_btn, startA_ok, armB_sw, startB_btn, startB_ok);
  }
}

static void serviceRx(){
  while(pwrSerial.available()){
    uint8_t c = pwrSerial.read();
    lastRxMs = millis();
    switch(rxState){
      case RS_WAIT_55:
        if(c==0x55){
          rxState=RS_WAIT_AA;
          rxIdx=0;
        }
        break;
      case RS_WAIT_AA:
        if(c==0xAA){
          rxState=RS_PAYLOAD;
          rxIdx=0;
        } else {
          rxState=RS_WAIT_55;
        }
        break;
      case RS_PAYLOAD:
        rxBuf[rxIdx++] = c;
        if(rxIdx==6) rxState=RS_CRC;
        break;
      case RS_CRC:{
        uint8_t crc = c;
        uint8_t calc = crc8(rxBuf, 6);
        rxState=RS_WAIT_55;
        if(calc!=crc) break;
        if(rxBuf[0]==0x81){
          uint8_t bits = rxBuf[2];
          pwr_armA_seen = bits & 0x01;
          pwr_onA       = bits & 0x02;
          pwr_armB_seen = bits & 0x04;
          pwr_onB       = bits & 0x08;
          pwr_key_ok    = bits & 0x10;
          pwr_presA     = bits & 0x20;
          pwr_presB     = bits & 0x40;
          pwr_faultAny  = bits & 0x80;

          pwr_vbat_x10  = rxBuf[3];
          pwr_ia_x10    = rxBuf[4];
          pwr_ib_x10    = rxBuf[5];

          lastStatusMs = millis();
          rxCount++;
          ui_markDirty();
        }
      } break;
    }
  }
}

void power_init() {
    pinMode(PWR_RS485_DE_RE_PIN, OUTPUT);
    digitalWrite(PWR_RS485_DE_RE_PIN, LOW);

    pwrSerial.begin(PWR_RS485_BAUD);

    pinMode(PWR_ARM_A_PIN,   INPUT_PULLUP);
    pinMode(PWR_ARM_B_PIN,   INPUT_PULLUP);
    pinMode(PWR_START_A_PIN, INPUT_PULLUP);
    pinMode(PWR_START_B_PIN, INPUT_PULLUP);

    pinMode(PWR_LED_A_PIN, OUTPUT);
    pinMode(PWR_LED_B_PIN, OUTPUT);
    digitalWrite(PWR_LED_A_PIN, LOW);
    digitalWrite(PWR_LED_B_PIN, LOW);

    pinMode(PWR_VBAT_PIN, INPUT);
#if defined(__IMXRT1062__)
    analogReadResolution(12);
#endif

    lastTxMs = lastPollMs = lastBlinkMs = lastStatusMs = millis();
}

void power_update() {
  serviceRx();

  uint32_t now = millis();

  // RX rate for UI (frames per second)
  if (now - tRate >= 1000) {
    tRate = now;
    rxCountLast = rxCount;
    rxCount = 0;
    pwr_rxRate = rxCountLast;
  }

  // Read inputs (active-low)
  bool armA_sw    = (digitalRead(PWR_ARM_A_PIN)==LOW);
  bool armB_sw    = (digitalRead(PWR_ARM_B_PIN)==LOW);
  bool startA_btn = (digitalRead(PWR_START_A_PIN)==LOW);
  bool startB_btn = (digitalRead(PWR_START_B_PIN)==LOW);

  // Safety: require START release after arming
  if (armA_sw && !prevArmA) { okToIgniteA = !startA_btn; }
  if (!armA_sw)             { okToIgniteA = false; }
  if (!startA_btn)          { okToIgniteA = armA_sw; }

  if (armB_sw && !prevArmB) { okToIgniteB = !startB_btn; }
  if (!armB_sw)             { okToIgniteB = false; }
  if (!startB_btn)          { okToIgniteB = armB_sw; }

  prevArmA=armA_sw;
  prevArmB=armB_sw;

  // Track overall arm state and latest arm-on edge time
  bool anyArm = armA_sw || armB_sw;
  static bool prevAnyArm=false;
  if (anyArm && !prevAnyArm) {
    pwr_lastArmOnMs = now;
  }
  pwr_anyArmed = anyArm;
  prevAnyArm = anyArm;

  // Desired START, block if key missing or fault active
  bool desiredA = armA_sw && startA_btn && okToIgniteA && pwr_key_ok && !pwr_faultAny;
  bool desiredB = armB_sw && startB_btn && okToIgniteB && pwr_key_ok && !pwr_faultAny;

  // Master-side fire timeout (10 s)
  if (desiredA && !prevDesiredA) m_tStartA = now;
  if (desiredB && !prevDesiredB) m_tStartB = now;

  if (desiredA && !m_fireTimeoutA && (now - m_tStartA >= FIRE_MAX_MS))
    m_fireTimeoutA = true;
  if (desiredB && !m_fireTimeoutB && (now - m_tStartB >= FIRE_MAX_MS))
    m_fireTimeoutB = true;

  // Releasing START clears local timeout latches
  if (!startA_btn) m_fireTimeoutA = false;
  if (!startB_btn) m_fireTimeoutB = false;

  bool startA_ok = desiredA && !m_fireTimeoutA;
  bool startB_ok = desiredB && !m_fireTimeoutB;

  if (startA_btn && !prevStartAButton) {
    logPowerStartEvent("A_PRESS", armA_sw, startA_btn, startA_ok, armB_sw, startB_btn, startB_ok);
    beginPowerFireLog(startB_btn ? "AB" : "A");
  }
  if (startB_btn && !prevStartBButton) {
    logPowerStartEvent("B_PRESS", armA_sw, startA_btn, startA_ok, armB_sw, startB_btn, startB_ok);
    beginPowerFireLog(startA_btn ? "AB" : "B");
  }
  prevStartAButton = startA_btn;
  prevStartBButton = startB_btn;

  updatePowerFireLog(armA_sw, startA_btn, startA_ok, armB_sw, startB_btn, startB_ok);

  prevDesiredA = desiredA;
  prevDesiredB = desiredB;

  // Blink state for normal firing animation
  if(now - lastBlinkMs >= FAST_BLINK_MS){
    lastBlinkMs=now;
    blink=!blink;
  }

  // LEDs:
  // - If link stale, key missing, or fault: turn off channel LEDs
  // - Otherwise: solid when armed, blink while pressed/ON
  if (!power_link_fresh() || !pwr_key_ok || pwr_faultAny) {
    digitalWrite(PWR_LED_A_PIN, LOW);
    digitalWrite(PWR_LED_B_PIN, LOW);
  } else {
    auto driveLed = [&](bool arm_sw, bool btn, bool statusOn, uint8_t pin){
      if(!arm_sw){
        digitalWrite(pin, LOW);
        return;
      }
      if(btn || statusOn){
        digitalWrite(pin, blink ? HIGH : LOW);
      } else {
        digitalWrite(pin, HIGH);
      }
    };
    driveLed(armA_sw, startA_btn && !m_fireTimeoutA, pwr_onA, PWR_LED_A_PIN);
    driveLed(armB_sw, startB_btn && !m_fireTimeoutB, pwr_onB, PWR_LED_B_PIN);
  }

  // Periodic TX to Power (state + occasional poll)
  if(now - lastTxMs >= TX_PERIOD_MS){
    lastTxMs = now;
    bool doPoll = (now - lastPollMs >= POLL_PERIOD_MS);
    if(doPoll) lastPollMs = now;
    sendMasterFrame(doPoll, armA_sw, startA_ok, armB_sw, startB_ok);
  }

  // Local battery measurement for UI (slow, ~1 Hz)
  static uint32_t lastVbatMs = 0;
  if (now - lastVbatMs >= 1000) {
    lastVbatMs = now;
    pwr_localVbat = readLocalVbat();
  }
}
