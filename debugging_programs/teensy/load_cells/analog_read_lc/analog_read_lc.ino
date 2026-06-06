int forcepins[8] =   {  17,   16,   15,   14,   41,   40,   39,   38}; // Sets force pad pins. See description above 
int analogZeros[8] = {   0,    0,    0,    0,    0,    0,    0,    0}; // zero point in ADC counts of sensor. Does not get calibrated during runtime. Spring preload calculated by subtracting from these 
int calADCcnts[8] =  {   0,    0,    0,    0,    0,    0,    0,    0}; // ADC counts corresponding to calibration weight. Used to determine scaling factor of load cells.

constexpr int n_avg = 15;
int vals[8][n_avg];

int arr_index = 0;

void setup() {
  Serial.begin(119200);
  // put your setup code here, to run once:
  analogReadResolution(12);
  for (int i = 0; i < 8; i++){
    for (int j = 0; j < 8; j++){
      vals[i][j] = 0;
    }
  }
}

void loop() {
  for (int i = 0; i < 8; i++){
    vals[i][arr_index] = analogRead(forcepins[i]);
    double temp = 0;
    for (int j = 0; j < n_avg; j++){
      temp += vals[i][j];
    }
    temp = temp / (double)(n_avg);
    Serial.print("lc");
    Serial.print(i);
    Serial.print(":");
    Serial.print(temp);
    // Serial.print(analogRead(forcepins[i]));
    Serial.print(", ");
  }

  arr_index ++;
  if (arr_index > n_avg - 1){
    arr_index = 0;
  }
  Serial.println();
  delay(30);
}