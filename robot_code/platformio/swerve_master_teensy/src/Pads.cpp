#include "Pads.h"
#include <math.h>
#include "Arduino.h"

double avg(double val1, double val2){
  return (val1+val2)/2;
}

Pads::Pads() {
  // this->intToN = 1.65*this->capacity/((this->mvv/1000)*this->ampgain*pow(2, 11));  // conversion factor from 12 bit ADC values to Newtons
  for (int i = 0; i < numForceSensors; i++){
    this->scaling[i] = this->cal_weight / (this->calADCcnts[i]-this->analogZeros[i]); // scaling factor from ADC counts to Newtons
  }
}

void Pads::calcVector() {
  this->getForces();
  double ftemp[8] = {this->forces[0], this->forces[1], this->forces[2], this->forces[3], this->forces[4], this->forces[5], this->forces[6], this->forces[7]};
  double m_x[8], m_y[8]; 
  double m_x_left = 0;
  double m_y_left = 0;
  double m_x_right = 0;
  double m_y_right = 0;

  for (int i = 0; i < 8; i++){ // calculate moment arms of each load cell. relative to center of robot 
    ftemp[i] = ftemp[i] - ftare[i]; // subtract tare value to zero out forces when rider is on board
    m_x[i] = ftemp[i] * this->x_loc[i]; // x moment arm of loads (left/right)
    m_y[i] = ftemp[i] * this->y_loc[i]; // y moment arm of loads (fore/aft)
  }

  for (int i = 0; i < 4; i++){ // sum up moments per pad
    m_x_right += m_x[i];
    m_y_right += m_y[i];
    m_x_left += m_x[i+4];
    m_y_left += m_y[i+4];
  }

  double f_right = ftemp[0] + ftemp[1] + ftemp[2] + ftemp[3];
  double f_left =  ftemp[4] + ftemp[5] + ftemp[6] + ftemp[7];

  double f_front = ftemp[2] + ftemp[3] + ftemp[4] + ftemp[4];
  double f_back =  ftemp[0] + ftemp[1] + ftemp[6] + ftemp[7];

  double f_cw =    ftemp[0] + ftemp[1] + ftemp[4] + ftemp[5];
  double f_ccw =   ftemp[2] + ftemp[3] + ftemp[6] + ftemp[7];

  this->cop_x_l = m_x_left / this->totalweight; // x coordinate of center of pressure for left pad, positive is right
  this->cop_y_l = m_y_left / this->totalweight; // y coordinate of center of pressure for left pad, positive is front
  this->cop_x_r = m_x_right / this->totalweight; // x coordinate of center of pressure for right pad, positive is right
  this->cop_y_r = m_y_right / this->totalweight; // y coordinate of center of pressure for right pad, positive is front
  this->cop_x = (m_x_right + m_x_left) / this->totalweight; // x coordinate of center of pressure, positive is right
  this->cop_y = (m_y_right + m_y_left) / this->totalweight; // y coordinate of center of pressure, positive is front
  this->cop_z = (m_y_right - m_y_left) / this->totalweight; // rotational component of center of pressure, positive
  // this->x_out = constrain(this->cop_x / this->x_out_scale, -1, 1);  // x coordinate of center of pressure, positive is right
  // this->y_out = constrain(this->cop_y / this->y_out_scale, -1, 1);  // y coordinate of center of pressure, positive is front
  // this->z_out = constrain(this->cop_z / this->z_out_scale, -1, 1);  // rotational component of center of pressure, positive is clockwise
  
  this->x_out = constrain((f_right - f_left) / this->totalweight, -1, 1);
  this->y_out = constrain((f_front - f_back) / this->totalweight, -1, 1);  
  this->z_out = constrain((f_cw - f_ccw) / this->totalweight, -1, 1);  
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
  this->totalweight = 0;
  for (int i = 0; i < numForceSensors; i++){
    this->forces[i] = this->rawforces[i] - this->fzeros[i];
    this->totalweight += this->rawforces[i];
  }
}

void Pads::getRawForces() {
  this->totalweight = 0;
  for (int i = 0; i < numForceSensors; i++){
    this->rawforces[i] = (double)(analogRead(this->forcepins[i]) - this->analogZeros[i]) * this->scaling[i];
  }
}

double Pads::getForce(int ch) {
  return this->forces[ch];
}

double Pads::getRawForce(int ch) {
  return this->rawforces[ch];
}

double Pads::getTotalWeight() {
  return this->totalweight;
}

void Pads::tare() {
  this->getForces();
  for (int i = 0; i < numForceSensors; i++){
    this->ftare[i] = this->forces[i];
  }
}

void Pads::calibrate() {
  this->getRawForces();
  for (int i = 0; i < numForceSensors; i++){
    this->fzeros[i] = this->rawforces[i];
  }
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

#pragma region Getters for COP coordinates

double Pads::get_cop_x() {
  return this->cop_x;
}

double Pads::get_cop_y() {
  return this->cop_y;
}

double Pads::get_cop_z() {
  return this->cop_z;
}

double Pads::get_cop_x_l() {
  return this->cop_x_l;
}

double Pads::get_cop_y_l() {
  return this->cop_y_l;
}

double Pads::get_cop_x_r() {
  return this->cop_x_r;
}

double Pads::get_cop_y_r() {
  return this->cop_y_r;
}

#pragma endregion
