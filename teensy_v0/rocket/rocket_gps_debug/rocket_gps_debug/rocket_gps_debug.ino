void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);  // GT-U7 default
  delay(1000);
  Serial.println("GPS raw test start");
}

void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    Serial.write(c);  // echo NMEA sentences to USB serial
  }
}
