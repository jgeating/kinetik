void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  analogReadResolution(12);
}

void loop() {
  Serial.println("****************");
  Serial.print("A0: ");
  Serial.println(analogRead(A0));
  Serial.print("A1: ");
  Serial.println(analogRead(A1));
  Serial.print("A2: ");
  Serial.println(analogRead(A2));
  Serial.print("A3: ");
  Serial.println(analogRead(A3));
  Serial.print("D2: ");
  Serial.println(digitalRead(2));
  Serial.print("D3: ");
  Serial.println(digitalRead(3));
  delay(20);
}

// A0 = Right Vert
// A1 = Right Hor
// A2 = Left Hor
// A3 = Left Vert 
// D2 = Right SW
// D3 = Left SW
