int ir_pins[4] = {22, 24, 26, 28};
int n = 4;

void setup() {
  Serial.begin(115200);
  
  for (int k = 0; k<n; k++){
    pinMode(ir_pins[k], INPUT);
  }
}

void loop() {
  Serial.println("--------------");
  for (int k = 0; k<n; k++){
    Serial.print(k);
    Serial.print(": ");
    Serial.println(digitalRead(ir_pins[k]));
  }
  delay(25);
}