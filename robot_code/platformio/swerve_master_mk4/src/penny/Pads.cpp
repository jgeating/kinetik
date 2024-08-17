#include "penny/Pads.h"
#include <math.h>
#include "DueCANLayer.h"

double avg(double val1, double val2){
  return (val1+val2)/2;
}

Pads::Pads() {
  this->intToN = 1.65*this->capacity/((this->mvv/1000)*this->ampgain*pow(2, 11));  // conversion factor from 12 bit ADC values to Newtons
}

void Pads::calcVector() {
  this->getForces();
  //                  0    1    2    3    4    5    6    7
  //Corresponds to  {RY+, RY-, RX+, RX-, LY+, LY-, LX+, LX-}
  //Values are:     {RFL, RFR, RBL, RBR, LFL, LFR, LBL, LBR} (i.e. Left back right = left pad, in the back right corner)
  double temp = 0;

  //Right Side
  this->cart[0] = avg(this->forces[0], this->forces[1]);  // front
  this->cart[1] = avg(this->forces[2], this->forces[3]);  // back
  this->cart[2] = avg(this->forces[1], this->forces[3]);  // right
  this->cart[3] = avg(this->forces[0], this->forces[2]);  // left
  //Left Side
  this->cart[4] = avg(this->forces[4], this->forces[5]);  // front
  this->cart[5] = avg(this->forces[6], this->forces[7]);  // back
  this->cart[6] = avg(this->forces[5], this->forces[7]);  // right
  this->cart[7] = avg(this->forces[4], this->forces[6]);  // left

  // temp = this->cart[0] + this->cart[4] +  this->cart[1] + this->cart[5];  // sum all forces
  if (this->totalweight >= 0){
    this->x_out  = this->cart[4] + this->cart[5] - (this->cart[0] + this->cart[1]);  //  1 * (front right + back right - front left - back left)
    this->x_out  /= this->totalweight;
    this->y_out  = this->cart[1] + this->cart[5] - (this->cart[0] + this->cart[4]);  //  1 * (front right + front left - back right - back left)
    this->y_out  /= this->totalweight;
    this->z_out  = this->cart[0] + this->cart[5] - (this->cart[1] + this->cart[4]);  // -1 * (front right + back left - back right - front left)
    this->z_out  /= this->totalweight;
  } else {
    // Serial.println("div 0 bypassed to 0");
    this->x_out = 0;
    this->y_out = 0;
    this->z_out = 0;
  }
}

void Pads::printDebug(){
  Serial.println("****Carts****");
  for (int i = 0; i < 8; i++){
    Serial.print(i);
    Serial.print(": ");
    Serial.println(this->cart[i]);
  }
  delay(100);
}

void Pads::getForces() {
  this->getRawForces();
  for (int i = 0; i < numForceSensors; i++){
    this->forces[i] -= fzeros[i];
  }
  this->forces[3] = this->forces[2];  // 7/21/2024, bad sensor. Temporarily overriding to zero until spare arrives
  // this->forces[2] = this->forces[2] * 2;  // Also doubling index 2 to quasi-compensate (it's redundant)
}

void Pads::getRawForces() {
  this->totalweight = 0;
  for (int i = 0; i < numForceSensors; i++){
    this->forces[i] = this->intToN*((int)analogRead(this->forcepins[i])-this->analogZeros[i]);
    if (i == 3){ // 7/21/2024, bad sensor. Temporarily overriding to zero until spare arrives
      this->forces[3] = this->forces[2];
    }
    // this->forces[i] = analogRead(this->forcepins[i]);
    //load cell: 1mV/V, amplifier: 495x
    this->totalweight += this->forces[i];
  }
}

double Pads::getForce(int ch) {
  return this->forces[ch];
}

void Pads::calibrate() {
  this->getRawForces();
  for (int i = 0; i < numForceSensors; i++){
    this->fzeros[i] = this->forces[i];
  }
  this->getForces();
}

bool Pads::fallDetected() {
  return this->totalweight < 30*9.81; //edit this
}

double Pads::getY() {
  return this->y_out;
}

double Pads::getX() {
  return this->x_out;
}

double Pads::getZ() {
  return this->z_out;
}
