int forcepins[8] =   {  15,   14,   16,   17,   40,   41,   38,   39}; // Sets force pad pins. See description above 
int analogZeros[8] = {   0,    0,    0,    0,    0,    0,    0,    0}; // zero point in ADC counts of sensor. Does not get calibrated during runtime. Spring preload calculated by subtracting from these 
int calADCcnts[8] =  {   0,    0,    0,    0,    0,    0,    0,    0}; // ADC counts corresponding to calibration weight. Used to determine scaling factor of load cells.

void setup() {
  Serial.begin(119200);
  // put your setup code here, to run once:
  analogReadResolution(12);
}

void loop() {
  for (int i = 0; i < 8; i++){
    Serial.print(analogRead(forcepins[i]));
    Serial.print(", ");
  }
  Serial.println();
  delay(10);
}