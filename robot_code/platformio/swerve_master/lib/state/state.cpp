// state.cpp

#include "state.h"
#include <Arduino.h>

state::state(float sampleTime, int lagFilterSize)
{
    this->sampleTime = sampleTime;
    input = 0.0;
    lastInput = 0.0;
    derivative = 0.0;
    this->lagFilterSize = lagFilterSize;

    // Initialize derivative lag filter array
    derivativeLagFilter = new float[lagFilterSize];
    for (int i = 0; i < lagFilterSize; i++)
    {
        derivativeLagFilter[i] = 0.0;
    }
}

state::~state()
{
    delete[] derivativeLagFilter;
}

void state::setInput(float input)
{
    this->input = input;
}

float state::compute()
{
    // Compute derivative
    this->derivative = (input - lastInput) / sampleTime;
    updateDerivativeLagFilter();

    float deriv_avg = 0;
    for (int i = lagFilterSize - 1; i > 0; i--)
    {
        deriv_avg += derivativeLagFilter[i];
    }
    deriv_avg = deriv_avg / lagFilterSize;

    return deriv_avg;
}

void state::updateDerivativeLagFilter()
{
    // Shift the values in the derivative lag filter array
    for (int i = lagFilterSize - 1; i > 0; i--)
    {
        derivativeLagFilter[i] = derivativeLagFilter[i - 1];
    }

    // Update the first element of the derivative lag filter array with the current derivative
    derivativeLagFilter[0] = derivative;
}
