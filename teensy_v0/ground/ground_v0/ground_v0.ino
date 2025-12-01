#include <SPI.h>
#include <LoRa.h>

const long LORA_FREQUENCY = 915E6;

// Pin mapping (same)
const int LORA_CS_PIN    = 10;
const int LORA_RST_PIN   = 9;
const int LORA_DIO0_PIN  = 2;

struct TelemetryPacketV2 {
  uint8_t  version;
  uint32_t seq;
  uint32_t ms;
  float    alt;
  float    temp;
  float    ax;
  float    ay;
  float    az;
  float    gx;
  float    gy;
  float    gz;
};

void setupLoRa() {
  LoRa.setPins(LORA_CS_PIN, LORA_RST_PIN, LORA_DIO0_PIN);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    while (true) { } // fail hard for now
  }
}

void setup() {
  Serial.begin(115200);
  setupLoRa();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) {
    return;
  }

  TelemetryPacketV2 pkt;
  int expectedSize = sizeof(pkt);

  if (packetSize >= expectedSize) {
    LoRa.readBytes((uint8_t*)&pkt, expectedSize);
    int rssi = LoRa.packetRssi();

    Serial.print("V=");   Serial.print(pkt.version);
    Serial.print(", SEQ="); Serial.print(pkt.seq);
    Serial.print(", ms=");  Serial.print(pkt.ms);
    Serial.print(", alt="); Serial.print(pkt.alt);
    Serial.print(", temp=");Serial.print(pkt.temp);
    Serial.print(", ax=");  Serial.print(pkt.ax);
    Serial.print(", ay=");  Serial.print(pkt.ay);
    Serial.print(", az=");  Serial.print(pkt.az);
    Serial.print(", gx=");  Serial.print(pkt.gx);
    Serial.print(", gy=");  Serial.print(pkt.gy);
    Serial.print(", gz=");  Serial.print(pkt.gz);
    Serial.print(", RSSI=");Serial.println(rssi);
  } else {
    while (LoRa.available()) LoRa.read();
  }
}
