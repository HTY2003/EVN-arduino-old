#ifndef EVNISRTimer_h
#define EVNISRTimer_h
#include <Arduino.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"

class EVNISRTimer
{
public:
    EVNISRTimer() {};
    static repeating_timer& sharedISRTimer(uint8_t timer_no) { static EVNISRTimer shared; return shared.timers[timer_no]; }
    static alarm_pool* sharedAlarmPool() { static EVNISRTimer shared; return shared.pool; }

private:
    static struct alarm_pool* pool;
    static struct repeating_timer timers[16];
};

#endif