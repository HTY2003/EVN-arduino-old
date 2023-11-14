# EVN Alpha
Software Repository for EVN Alpha

## Table of contents
- [App Installation](#app-installation)
- [Test Upload](#test-upload)
- [Library Installation](#library-installation)
- [Usage in Arduino IDE](#usage-in-arduino-ide)

## App Installation
The following apps should be installed
* [Arduino IDE 1.8.19] (https://www.arduino.cc/en/software)

After installing Arduino IDE, install Earle F Philhower's Arduino-Pico core using the Boards Manager (Tools - Board - Boards Manager - search "Raspberry Pi Pico/RP2040" - Install "Raspberry Pi Pico/RP2040 by Earle F Philhower III").

To upload, select Board: Raspberry Pi Pico (Tools - Board - Raspberry Pi RP2040 Boards - Click Raspberry Pi Pico).

Plug in the USB cable to the EVN Alpha. 

When the EVN Alpha is on (red light), it will appear as an option in the Ports menu (e.g. Tools - Port - COM7 (Raspberry Pi Pico)). Select this COM Port to upload to EVN Alpha.

## Test Upload
To test EVN Alpha, use the Blink example in Arduino (File - Examples - 01.Basics - Blink).

## Library Installation
Download this repository as a .zip file.

Install in Arduino IDE as follows:
Sketch - Include Library - Add .ZIP Library - Browse to downloaded .zip file and select.


## Usage in Arduino IDE
To include all libraries, include them at the start of the library:

```
#include <EVN.h>
```

Create instances before void setup():

```
EVNSensorHelper helper;
EVNButton button;
EVNDisplay display(0); //i2c bus

EVNMotor motora(1, EV3_LARGE, &button);
EVNMotor motorb(2, EV3_LARGE, &button);
EVNMotor motorc(3, EV3_MED, &button); 
EVNDrivebase(60, 175, &motora, &motorb, DIRECT, DIRECT);

PIDController(1,0,0, DIRECT);

```


And initalize the hardware components in void setup() and void setup1():
```
void setup()
{
    button.init();
    helper.init();

    helper.selectPort(0, 0) //i2c bus 0, port 0
    display
    display.init();
}

void setup1()
{
    //initializing motors on second core improves performance, if the 2nd core is not in use
    motora.init(); 
    motorb.init();
    motorc.init();
}

```

Detailed functions of each class will be outlined soon.