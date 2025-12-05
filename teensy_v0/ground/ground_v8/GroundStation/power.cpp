#include "power.h"

// RS-485 master protocol (mirrors arduino_remote_module)

HardwareSerial &pwrSerial = PWR_RS485_SERIAL;

// Timing
static const uint16_t TX_PERIOD_MS   = 50;
static const uint16_t POLL_PERIOD_MS = 300;
static const uint16_t LINK_FRESH_MS  = 500;
static const uint16_t FAST_BLINK_MS  = 120;
static const uint32_t FIRE_MAX_MS    = 10000UL;

// State from Power module
volatile uint32_t lastRxMs = 0;

uint32_t lastTxMs=0, lastPollMs=0, lastBlinkMs=0, lastStatusMs=0;
uint32_t tRate=0;
uint16_t rxCount=0, rxCountLast=0;
bool blink=false;
uint8_t seq=0;

bool pwr_armA_seen=false, pwr_onA=false, pwr_armB_seen=false, pwr_onB=false;
bool pwr_key_ok=false, pwr_presA=false, pwr_presB=false;
bool pwr_faultAny=false;

uint8_t pwr_vbat_x10=0, pwr_ia_x10=0, pwr_ib_x10=0;
float   pwr_localVbat=0.0f;

// Safety: START only valid if START was released after arming
static bool okToIgniteA=false;
static bool okToIgniteB=false;
static bool prevArmA=false;
static bool prevArmB=false;

// Master-side fire timeouts
static bool m_fireTimeoutA=false;
static bool m_fireTimeoutB=false;
static uint32_t m_tStartA=0;
static uint32_t m_tStartB=0;
static bool prevDesiredA=false;
static bool prevDesiredB=false;

bool power_link_fresh() {
    return (millis() - lastStatusMs) < LINK_FRESH_MS;
}

// --- Local battery read (e.g., 3.7V via 100k/100k divider) ---
static float readLocalVbat() {
    // Assume 3.3V reference and 10-bit ADC; good enough for status.
    int raw = analogRead(PWR_VBAT_PIN);
    const float vref = 3.3f;
    float v_pin = (float)raw * vref / 1023.0f;
    return v_pin * 2.0f;
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

  // Local battery measurement for UI
  pwr_localVbat = readLocalVbat();
}

