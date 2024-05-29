#include "EVNISRTimer.h"

struct repeating_timer EVNISRTimer0::timers[16];
struct alarm_pool* EVNISRTimer0::pool;
bool EVNISRTimer0::_started = false;

struct repeating_timer EVNISRTimer1::timers[16];
struct alarm_pool* EVNISRTimer1::pool;
bool EVNISRTimer1::_started = false;