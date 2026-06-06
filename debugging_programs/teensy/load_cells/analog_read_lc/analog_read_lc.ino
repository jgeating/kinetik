void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("A3:");
  Serial.print(analogRead(A3));
  Serial.print(" A2:");
  Serial.print(analogRead(A2));
  Serial.print(" A1:");
  Serial.print(analogRead(A1));
  Serial.print(" A0:");
  Serial.print(analogRead(A0)); 
  Serial.print(" A17:");
  Serial.print(analogRead(A17));
  Serial.print(" A16:");
  Serial.print(analogRead(A16));
  Serial.print(" A15:");
  Serial.print(analogRead(A15));
  Serial.print(" A14:");
  Serial.println(analogRead(A14));
  delay(10);
}