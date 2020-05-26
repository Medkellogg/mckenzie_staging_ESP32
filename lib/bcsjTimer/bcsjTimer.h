/*
  bcsjTimer.h - BCSJ communications timer library
  Copyright (c) 2014 Charlie Comstock.  All right reserved.
*/


#ifndef __BCSJTIMER_H__
#define __BCSJTIMER_H__

#include "arduino.h"

typedef unsigned long bcsjTime;

class bcsjTimer
{

#define MAXTIMEVALUE 0xffffffff

  //
  // PUBLIC function definitons
  //
  public:
             bcsjTimer();                  // constructor
    void     start( bcsjTime interval );   // start a timer running
    void     restart( void );              // start a timer running from previous startTime + deltaTime
    void     restart( bcsjTime interval ); // start a timer running from previous startTime + deltaTime
    boolean  running( void );              // is timer still running?
    boolean  done( void );                 // has timer finished?
    boolean  active( void );               // is timer active?
    void     disable( void );              // mark time inactive
    bcsjTime delta( void );                // returns microseconds since timer started
    bcsjTime test( void );                 // returns microseconds since timer started
   

  private:
    boolean  timerEnabled;                 // timer is active?
    bcsjTime startTime;                    // beginning of this time span
    bcsjTime deltaTime;                    // the duration of this time span

};

#endif


