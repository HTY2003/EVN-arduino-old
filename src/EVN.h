#ifndef EVN_h
#define EVN_h

#include "EVNAlpha.h"
#include "EVNMotor.h"
#include "EVNDisplay.h"
#include "EVNColourSensor.h"
#include "EVNCompassSensor.h"
#include "EVNDistanceSensor.h"
#include "PIDController.h"
#include "RPI_PICO_TimerInterrupt/src/RPi_Pico_ISR_Timer.h"

#define calibrate(reading, low, high) ((reading - low) / (high - low))

#endif