#include "EVNISRTimer.h"
#include <Arduino.h>

RPI_PICO_Timer EVNISRTimer::timer(2);
RPI_PICO_ISR_Timer EVNISRTimer::ISRtimer;