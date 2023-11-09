#include "switch.h"
//#define ALGO_SWITCH	5
#define DEBOUNCE_TIME 50 //ms


//ezButton algoSwitch(ALGO_SWITCH);



void setup_switch(ezButton algoSwitch)
{
  algoSwitch.setDebounceTime(DEBOUNCE_TIME);
}

/* This function needs to be called first in the void loop() */
void algoSwitchInit(ezButton algoSwitch)
{
  algoSwitch.loop();
}

int algoSwitchCheck(ezButton algoSwitch)
{
  if (algoSwitch.isPressed())
  {
    return 1;
  }
  else if (algoSwitch.isReleased())
  {
    return 0;
  }
  else 
  {
    return 2;
  }
}