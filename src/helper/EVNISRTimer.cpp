#include "EVNISRTimer.h"

struct repeating_timer EVNISRTimer::timers[16];
struct alarm_pool* EVNISRTimer::pool = alarm_pool_create(0, 16);