#ifndef _CHANNEL_
#define _CHANNEL_


class Channel
{
  private:
    int PIN;
    volatile int last_interrupt_time;
    volatile unsigned long timer_start;
    int ch;
    short offset;
  public:
    Channel(int pin, short chOff);
    void calc();
    void zero();
    int getPin();
    volatile int getLastInterruptTime();
    volatile unsigned long getTimerStart();
    int getCh();
    short getOffset();
}; 


#endif
