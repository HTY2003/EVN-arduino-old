#include "EVNISRTimer.h"
#include <Arduino.h>

RPI_PICO_Timer EVNISRTimer::timer(3);
RPI_PICO_ISR_Timer EVNISRTimer::ISRtimer;