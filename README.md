# EVN
Software Repository for EVN Alpha

## Table of Contents
- [EVN](#evn)
  - [Table of Contents](#table-of-contents)
  - [App Installation](#app-installation)
  - [Board Installation](#board-installation)
  - [Library Installation](#library-installation)
  - [Test Upload](#test-upload)
  - [Usage in Arduino IDE](#usage-in-arduino-ide)

## App Installation
The following apps should be installed
* [Arduino IDE 1.8.19] (https://www.arduino.cc/en/software)

## Board Installation
After installing Arduino IDE, install Earle F Philhower's Arduino-Pico core. This is done in 2 steps:

1. Open Preferences in Arduino IDE (`File` - `Preferences`)

2. Enter the following URL in the `Additional Boards Manager URLs` field:

https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json


3. Open Boards Manager in Arduino IDE (`Tools` - `Board` - `Boards Manager...`)


4. Search and install `"Raspberry Pi Pico/RP2040"` by Earle F Philhower III

To upload to EVN Alpha, select Raspberry Pi Pico as your board (`Tools` - `Board` - `Raspberry Pi RP2040 Boards` - `Raspberry Pi Pico`)

When the EVN Alpha is on (red light) and plugged in via a USB cable, it will appear as a COM Port which you can select to upload upload (e.g. `Tools` - `Port` - `COM7 (Raspberry Pi Pico)`).

## Library Installation
Download this repository as a .zip file and install in Arduino IDE (
`Sketch` - `Include Library` - `Add .ZIP Library` - Select downloaded .zip file).

## Test Upload
To test uploading to EVN Alpha, you can use the Blink example in Arduino (`File` - `Examples` - `01.Basics` - `Blink`).

## Usage in Arduino IDE
To include all EVN classes, include this header at the start of the library:

```
#include <EVN.h>
```

Declare instances before `void setup()`, in global scope:

```
EVNAlpha board;
EVNDisplay display(1); //port from 1-16

EVNMotor motora(1, EV3_LARGE);
EVNMotor motorb(2, EV3_LARGE);
EVNMotor motorc(3, EV3_MED);
EVNDrivebase(60, 175, &motora, &motorb);

PIDController(1,0,0, DIRECT);
```


And initalize the hardware components in `void setup()` and `void setup1()` with `begin()`:
```
void setup()
{
    board.begin();
    display.begin();
}

void setup1()
{
    //initializing motors on second core improves performance, if the 2nd core is not in use

    //however, motors can still be initialized on 1st core
    motora.begin(); 
    motorb.begin();
    motorc.begin();
}

```

Detailed functions of each class will be outlined in its own README in the `/docs` folder.