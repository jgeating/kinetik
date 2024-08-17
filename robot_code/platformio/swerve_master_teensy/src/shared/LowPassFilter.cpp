class LowPassFilter {
private:
  int numReadings;  // number of readings to be averaged
  int *readings;    // array to store the readings
  int index;        // index of the current reading
  int total;        // running total of the readings
  int average;      // calculated average value
public:
  LowPassFilter(int n) {
    numReadings = n;
    readings = new int[numReadings];
    for (int i = 0; i < numReadings; i++) {
      readings[i] = 0;
    }
    index = 0;
    total = 0;
    average = 0;
  }
  ~LowPassFilter() {
    delete[] readings;
  }
  int filter(int value) {
    // subtract the last reading:
    total -= readings[index];
    // add the new reading to the total:
    total += value;
    // store the new reading in the array:
    readings[index] = value;
    // advance to the next position in the array:
    index++;
    // if we've reached the end of the array, wrap around to the beginning:
    if (index >= numReadings) {
      index = 0;
    }
    // calculate the average value:
    average = total / numReadings;
    // return the filtered value:
    return average;
  }
};