# EVN Alpha
Software Repository for EVN Alpha

## Table of contents
- [App Pre-Requisites](#app-pre-requisites)
- [Test Upload](#test-upload)
- [Library Installation](#library-installation)
- [Usage in Arduino IDE](#usage-in-arduino-ide)

## App Pre-Requisites
The following apps should be installed
* Arduino IDE 1.8.x (latest version)

Add the Raspberry Pi Pico to your Arduino Installation by adding Earle F Philhower's Arduino-Pico core using the Boards Manager (Tools - Board - Boards Manager - search "Raspberry Pi Pico/RP2040" and install latest version).

To upload, select Raspberry Pi Pico as your board (Tools - Board - Raspberry Pi RP2040 Boards - Raspberry Pi Pico).

Plug in the USB cable to the EVN Alpha. 

When the EVN Alpha is on (red light), it will appear as an option in the Ports menu (Tools - Port). Select this COM Port to upload to it.

## Test Upload
To test EVN Alpha, you can use the Blink example in Arduino (File - Examples - 01.Basics - Blink).

## Library Installation

Transfer each folder to the libraries folder in File Explorer. On Windows, this should be in Documents/Arduino/libraries. Follow the folder structure shown below:

```
Documents/
└── Arduino/
    └── libraries/
        ├── EVNMotor/
        │   ├── EVNMotor.h
        │   └── EVNMotor.cpp
        ├── EVNSensorHelper/
        │   ├── EVNSensorHelper.h
        │   └── EVNSensorHelper.cpp
        └── EVNButton/
            └── EVNButton.h
            └── EVNButton.cpp

```

Ensure that in each EVN folder, there is only one .h and .cpp file.
Ensure that there are no 2 folders with the same name in the libraries folder.

Additionally for EVNMotor, install the RPI_PICO_TimerInterrupt Ver 1.3.1 library by Khoi Hoang in Arduino's Library Manager (Sketch - Include Library - Manage Libraries...).


## Usage in Arduino IDE
To include all libraries, include them at the start of the library:

```
#include "RPi_Pico_ISR_Timer.h" //this is a helper library for the EVNMotor library
#include <EVNMotor.h>
#include <EVNButton.h>
#include <EVNSensorHelper.h>
```

Create instances before void setup():

```
EVNButton butt;
EVNMotor motora(1, EV3_LARGE, &butt); //(port number from 1-4, motor type, pointer to button object)
EVNMotor motorb(2, EV3_MED, &butt); 
EVNSensorHelper helper;
```


And initalize them in void setup() and void setup1():
```
void setup()
{
    butt.init();
    helper.init();
}

void setup1()
{
    motora.init(); //initializing motors on second core improves performance, if the 2nd core is not in use
    motorb.init();
}

```

Detailed functions of each class will be outlined in the individual folder READMEs.