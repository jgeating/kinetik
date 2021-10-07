void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
}

void loop() {
  Serial.println(pulseIn(2, 20000));
  Serial.println(pulseIn(3, 20000));
  delay(50);
}
