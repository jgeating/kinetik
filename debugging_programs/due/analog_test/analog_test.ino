int apins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
int offsets[] = {1979, 1849, 2186, 2370, 2258, 1993, 2134, 2128};
int npins = 8;

void setup() {
  Serial.begin(119200);
  analogReadResolution(12);
  }

void loop() {
  Serial.println("******");
  for (int k = 0; k < npins; k++){
    Serial.print(k);
    Serial.print(": ");
    // Serial.println((int)(analogRead(apins[k])));
    Serial.println((int)(analogRead(apins[k])) - offsets[k]);
  }
  delay(50);
}
