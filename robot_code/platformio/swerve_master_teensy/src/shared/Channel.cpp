#include "shared/Channel.h"
#include "Arduino.h"

Channel::Channel(int pin, short offset) {
  this->PIN = pin;
  this->timer_start = 0;
  this->ch = 0;
  this->offset = offset;
}

void Channel::calc() {
  this->last_interrupt_time = micros(); 
  if (digitalRead(this->PIN) == HIGH) { 
     this->timer_start = micros(); 
  } else if(this->timer_start != 0) { 
    this->ch = ((volatile int)micros() - this->timer_start - this->offset);
    this->timer_start = 0; 
  }
}

void Channel::zero() {
  this->offset = this->offset + this->ch;
}

int Channel::getPin() {
  return this->PIN;
}

volatile int Channel::getLastInterruptTime() {
  return this->last_interrupt_time;
}

volatile unsigned long Channel::getTimerStart() {
  return this->timer_start;
}

int Channel::getCh() {
  return this->ch;
}

short Channel::getOffset() {
  return this->offset;
}
