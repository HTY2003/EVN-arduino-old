#ifndef EVNISRTimer_h
#define EVNISRTimer_h
#include <Arduino.h>
#include "RPI_PICO_TimerInterrupt/src/RPi_Pico_TimerInterrupt.h"
#include "RPI_PICO_TimerInterrupt/src/RPi_Pico_ISR_Timer.hpp"

#define HW_TIMER_INTERVAL_MS    1

class EVNISRTimer
{
public:
    static RPI_PICO_ISR_Timer& sharedISRTimer() { static EVNISRTimer shared; return shared.ISRtimer; }

private:
    static RPI_PICO_ISR_Timer ISRtimer; // means that no other class can use the ISRtimer (I think)
    static RPI_PICO_Timer timer;		// static timer shared by all instances

    EVNISRTimer() { timer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, isr); };

    static bool isr(struct repeating_timer* t)
    {
        ISRtimer.run();
        return true;
    }
};

#endif