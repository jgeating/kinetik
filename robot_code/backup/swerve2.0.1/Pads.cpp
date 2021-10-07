#include "Pads.h"
#include <math.h>
#include "DueCANLayer.h"

double avg(double val1, double val2){
  return (val1+val2)/2;
}

Pads::Pads(
   int forcepins[8],
   int forcezeros[8], 
   double xgain, 
   double ygain, 
   double zgain, 
   double vMax, 
   double wMax, 
   double weightthresh
) {
  for (int i = 0; i < numForceSensors; i++) {
    this->forcepins[i] = forcepins[i];
  }
  this->xgain = xgain;
  this->ygain = ygain;
  this->zgain = zgain;  
  this->vMax = vMax;
  this->wMax = wMax;
  this->weightthresh = weightthresh;
  this->intToN = 1.65*this->capacity/((this->mvv/1000)*this->ampgain*pow(2, 11));  // conversion factor from 12 bit ADC values to Newtons
}

void Pads::calcVector() {
   this->getForces();
   Serial.println(this->getForce(1));
  //                  0    1    2    3    4    5    6    7
  //Corresponds to  {RY+, RY-, RX+, RX-, LY+, LY-, LX+, LX-}
  //Values are:     {RFL, RFR, RBL, RBR, LFL, LFR, LBL, LBR} (i.e. Left back right = left pad, in the back right corner)
  double cart[numForceSensors] = {0, 0, 0, 0, 0, 0, 0, 0};  //('cart' short for cartesion)

  //Right Side
  cart[0] = avg(this->forces[0], this->forces[1]);  // front
  cart[1] = avg(this->forces[2], this->forces[3]);  // back
  cart[2] = avg(this->forces[1], this->forces[3]);  // right
  cart[3] = avg(this->forces[0], this->forces[2]);  // left
  //Left Side
  cart[4] = avg(this->forces[4], this->forces[5]);  // front
  cart[5] = avg(this->forces[6], this->forces[7]);  // back
  cart[6] = avg(this->forces[5], this->forces[7]);  // right
  cart[7] = avg(this->forces[4], this->forces[6]);  // left

  this->yvel    =  1 * (avg(cart[0], cart[4]) - avg(cart[1], cart[5]));  //  1 * (front right + front left - back right - back left)
  this->xvel    =  1 * (avg(cart[0], cart[1]) - avg(cart[4], cart[5]));  //  1 * (front right + back right - front left - back left)
  this->spinvel = -1 * (avg(cart[0], cart[5]) - avg(cart[1], cart[4]));  // -1 * (front right + back left - back right - front left)

  //Scaling via gains
  //this->yvel = constrain(this->yvel*ygain, -vMax, vMax);
  //this->xvel = constrain(this->xvel*xgain, -vMax, vMax);
  //this->spinvel = constrain(this->spinvel*zgain, -wMax, wMax);
}

void Pads::getForces() {
  //Force control stuff
  this->getRawForces();
  for (int i = 0; i < numForceSensors; i++){
    this->forces[i] -= fzeros[i];
  }
}

void Pads::getRawForces() {
  this->totalweight = 0;
  for (int i = 0; i < numForceSensors; i++){
    this->forces[i] = this->intToN*analogRead(this->forcepins[i]-this->forcezeros[i]);
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
    fzeros[i] = forces[i];
  }
  this->getForces();
}

bool Pads::fallDetected() {
  return this->totalweight < 30*9.81; //edit this
}

double Pads::getYVel() {
  return this->yvel;
}

double Pads::getXVel() {
  return this->xvel;
}

double Pads::getSpinVel() {
  return this->spinvel;
}
