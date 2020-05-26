
#include "bcsjTimer.h"


/*---------------------------------------------------------------------------
** CLEAR TIMER
**
** Disables timer
**--------------------------------------------------------------------------*/
bcsjTimer::bcsjTimer(void)
{
  startTime = 0;
  deltaTime = 0;
  timerEnabled = true;
}


/*---------------------------------------------------------------------------
** START TIMER
**
**--------------------------------------------------------------------------*/
void bcsjTimer::start( bcsjTime interval )
{
  startTime = micros();
  deltaTime = interval;
  timerEnabled = true;
}


/*---------------------------------------------------------------------------
** RESTART TIMER 
**
** Restarts timer with the previous time interval
**--------------------------------------------------------------------------*/
void bcsjTimer::restart( void )
{
  if (timerEnabled) {
    startTime += deltaTime;
  }
  else {
    startTime = micros();
    timerEnabled = true;
  }
}


/*---------------------------------------------------------------------------
** RESTART TIMER
**
** Restart timer with new time interval
**--------------------------------------------------------------------------*/
void bcsjTimer::restart( bcsjTime interval )
{
  if (timerEnabled) {
    startTime += deltaTime;
  }
  else {
    startTime = micros();
    timerEnabled = true;
  }
  deltaTime = interval;
}


/*---------------------------------------------------------------------------
** RUNNING TIMER
**
** Returns true if the timer is still running
**        false if the timer has finished
**--------------------------------------------------------------------------*/
boolean bcsjTimer::running( void )
{
  if (!timerEnabled) {
    return false;
  }
  if ((micros()-startTime) < deltaTime) {
    return true;
  }
  return false;
}


/*---------------------------------------------------------------------------
** DONE TIMER
**
** Returns true if the timer was running but has finished
**        false if the timer is still running or if it isn't enabled
**--------------------------------------------------------------------------*/
boolean bcsjTimer::done( void )
{
  if (timerEnabled) {
    if ((micros()-startTime) >= deltaTime) {
      return true;
    }
  }
  return false;
}


/*---------------------------------------------------------------------------
** ACTIVE TIMER
**
** Returns true if timer is active
**         false if timer is inactive
**--------------------------------------------------------------------------*/
boolean bcsjTimer::active( void )
{
  return timerEnabled;
}


/*---------------------------------------------------------------------------
** DISABLE TIMER
**
** Disables timer
**--------------------------------------------------------------------------*/
void bcsjTimer::disable( void )
{
  deltaTime = 0L;
  timerEnabled = false;
}


/*---------------------------------------------------------------------------
** DELTA
**
** Returns how long the timer has been running in microseconds
**         0xffffffff if the timer is disabled
**--------------------------------------------------------------------------*/
bcsjTime bcsjTimer::delta( void )
{
  if (timerEnabled) {
    return micros()-startTime;
  }
  return 0xffffffff;
}


/*---------------------------------------------------------------------------
** GET START TIME
**
** Returns how long the timer has been running in microseconds
**         0xffffffff if the timer is disabled
**--------------------------------------------------------------------------*/
bcsjTime bcsjTimer::test( void )
{
  if (timerEnabled) {
    return micros() - (startTime + deltaTime);
  }
  return 0;
}




