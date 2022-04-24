int apins[] = {A0, A1, A2, A3};
int offsets[] = {2112, 2066, 2060, 2113};
int npins = 4;

void setup() {
  Serial.begin(119200);
  analogReadResolution(12);
  }

void loop() {
  Serial.println("******");
  for (int k = 0; k < npins; k++){
    Serial.print(k);
    Serial.print(": ");
    Serial.println((int)(analogRead(apins[k])) - offsets[k]);
  }
  delay(50);
}
