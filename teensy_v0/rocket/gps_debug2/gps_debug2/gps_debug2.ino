#include <TinyGPSPlus.h>

TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);   // GT-U7 default
  delay(1000);
  Serial.println("TinyGPS++ test start");
}

void loop() {
  // Feed TinyGPS++ with NMEA from GPS
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
  }

  // Every second, print status
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= 1000) {
    lastPrint = now;

    Serial.print("chars=");
    Serial.print(gps.charsProcessed());

    Serial.print("  locValid=");
    Serial.print(gps.location.isValid());
    Serial.print("  satsValid=");
    Serial.print(gps.satellites.isValid());
    Serial.print("  hdopValid=");
    Serial.print(gps.hdop.isValid());

    Serial.print("  sats=");
    if (gps.satellites.isValid())
      Serial.print(gps.satellites.value());
    else
      Serial.print(-1);

    Serial.print("  hdop=");
    if (gps.hdop.isValid())
      Serial.print(gps.hdop.value() * 0.01f);
    else
      Serial.print(-1);

    if (gps.location.isValid()) {
      Serial.print("  lat=");
      Serial.print(gps.location.lat(), 6);
      Serial.print("  lon=");
      Serial.print(gps.location.lng(), 6);
    }

    if (gps.altitude.isValid()) {
      Serial.print("  alt=");
      Serial.print(gps.altitude.meters());
    }

    Serial.println();
  }
}
