// PID.h

#ifndef PID_H
#define PID_H

class PID {
  public:
    PID(float Kp, float Ki, float Kd, float sampleTime, int lagFilterSize);
    ~PID();
    void setSetpoint(float setpoint);
    void setInput(float input);
    float compute();
    
  private:
    float Kp;
    float Ki;
    float Kd;
    float sampleTime;
    float setpoint;
    float input;
    float lastInput;
    float output;
    float errorSum;
    float lastError;
    float derivative;
    float* derivativeLagFilter; // Pointer to derivative lag filter array
    int lagFilterSize;
    
    void updateDerivativeLagFilter();
};

#endif
