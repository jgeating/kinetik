// state.h

#ifndef state_H
#define state_H

class state
{
public:
    state(float sampleTime, int lagFilterSize);
    ~state();
    void setSetpoint(float setpoint);
    void setInput(float input);
    float compute();

private:
    float sampleTime;
    float input;
    float lastInput;
    float derivative;
    float *derivativeLagFilter; // Pointer to derivative lag filter array
    int lagFilterSize;

    void updateDerivativeLagFilter();
};

#endif
