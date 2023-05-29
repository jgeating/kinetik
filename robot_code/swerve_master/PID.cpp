// PID.cpp

#include "PID.h"
#include <Arduino.h>

PID::PID(float Kp, float Ki, float Kd, float sampleTime, int lagFilterSize) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->sampleTime = sampleTime;
  setpoint = 0.0;
  input = 0.0;
  lastInput = 0.0;
  output = 0.0;
  errorSum = 0.0;
  lastError = 0.0;
  derivative = 0.0;
  this->lagFilterSize = lagFilterSize;

  // Initialize derivative lag filter array
  derivativeLagFilter = new float[lagFilterSize];
  for (int i = 0; i < lagFilterSize; i++) {
    derivativeLagFilter[i] = 0.0;
  }
}

PID::~PID() {
  delete[] derivativeLagFilter;
}

void PID::setSetpoint(float setpoint) {
  this->setpoint = setpoint;
}

void PID::setInput(float input) {
  this->input = input;
}

float PID::compute() {
  // Compute error and error sum
  float error = setpoint - input;
  errorSum += error;

  // Compute derivative
  derivative = (input - lastInput) / sampleTime;
  updateDerivativeLagFilter();

  // Compute PID output
  output = Kp * error + Ki * errorSum + Kd * derivativeLagFilter[0];

  // Store current input as last input for next iteration
  lastInput = input;

  return output;
}

void PID::updateDerivativeLagFilter() {
  // Shift the values in the derivative lag filter array
  for (int i = lagFilterSize - 1; i > 0; i--) {
    derivativeLagFilter[i] = derivativeLagFilter[i - 1];
  }

  // Update the first element of the derivative lag filter array with the current derivative
  derivativeLagFilter[0] = derivative;
}
