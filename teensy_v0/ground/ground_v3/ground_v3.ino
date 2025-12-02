#include <SPI.h>
#include <LoRa.h>

const long LORA_FREQUENCY = 915E6;
const int LORA_CS_PIN    = 10;
const int LORA_RST_PIN   = 9;
const int LORA_DIO0_PIN  = 2;

enum FlightState : uint8_t {
  FS_IDLE = 0,
  FS_PAD,
  FS_ASCENT,
  FS_COAST,
  FS_DESCENT,
  FS_LANDED,
  FS_ABORT
};

const char* stateName(uint8_t s) {
  switch (s) {
    case FS_IDLE:    return "IDLE";
    case FS_PAD:     return "PAD";
    case FS_ASCENT:  return "ASCENT";
    case FS_COAST:   return "COAST";
    case FS_DESCENT: return "DESCENT";
    case FS_LANDED:  return "LANDED";
    case FS_ABORT:   return "ABORT";
    default:         return "UNKNOWN";
  }
}

struct TelemetryPacketV4 {
  uint8_t  version;
  uint8_t  state;
  uint16_t flags;
  uint32_t seq;
  uint32_t ms;
  float    alt;
  float    temp;
  float    vel_z;
  float    ax;
  float    ay;
  float    az;
  float    gx;
  float    gy;
  float    gz;
  float    roll_deg;
  float    pitch_deg;
};

void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    while (true) {}
  }
}

void setup() {
  Serial.begin(115200);
  setupLoRa();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  TelemetryPacketV4 pkt;
  int expectedSize = sizeof(pkt);

  if (packetSize >= expectedSize) {
    LoRa.readBytes((uint8_t*)&pkt, expectedSize);
    int rssi = LoRa.packetRssi();

    Serial.print("V=");      Serial.print(pkt.version);
    Serial.print(", STATE=");Serial.print(stateName(pkt.state));
    Serial.print(", SEQ=");  Serial.print(pkt.seq);
    Serial.print(", ms=");   Serial.print(pkt.ms);
    Serial.print(", alt=");  Serial.print(pkt.alt);
    Serial.print(", temp="); Serial.print(pkt.temp);
    Serial.print(", vel=");  Serial.print(pkt.vel_z);
    Serial.print(", ax=");   Serial.print(pkt.ax);
    Serial.print(", ay=");   Serial.print(pkt.ay);
    Serial.print(", az=");   Serial.print(pkt.az);
    Serial.print(", gx=");   Serial.print(pkt.gx);
    Serial.print(", gy=");   Serial.print(pkt.gy);
    Serial.print(", gz=");   Serial.print(pkt.gz);
    Serial.print(", roll="); Serial.print(pkt.roll_deg);
    Serial.print(", pitch=");Serial.print(pkt.pitch_deg);
    Serial.print(", FLAGS=0x"); Serial.print(pkt.flags, HEX);
    Serial.print(", RSSI="); Serial.println(rssi);
  } else {
    while (LoRa.available()) LoRa.read();
  }
}
