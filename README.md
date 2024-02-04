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
**Both** versions of Arduino IDE will **both** work with this library
* [Arduino IDE 2.2.1] (https://www.arduino.cc/en/software)
* [Arduino IDE 1.8.19] (https://www.arduino.cc/en/software)

## Board Installation
After installing Arduino IDE, install Earle F Philhower's Arduino-Pico core. This is done in 2 steps:

1. Open Preferences in Arduino IDE (`File` - `Preferences`)

2. Enter the following URL in the `Additional Boards Manager URLs` field:

https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json


3. Open Boards Manager in Arduino IDE (`Tools` - `Board` - `Boards Manager...`)

4. Search and install `"Raspberry Pi Pico/RP2040"` by Earle F Philhower III. Install version 3.6.3 and above.

### Updating Arduino-Pico Core
If you have an existing installation of the Arduino-Pico core, and want to update it, you may be unable to successfully compile after updating. This is an issue with the [Arduino IDE](https://github.com/arduino/Arduino/issues/11842) failing to extract the files correctly, the best solution is to do a clean removal of the core before reinstalling.

1. Uninstall Arduino-Pico in Boards Manager. Close Arduino IDE.

2. Delete the following folder: `C:\Users\%USERNAME%\AppData\Local\Arduino15\packages\rp2040`

3. Reopen Arduino IDE and do a fresh install as described above.

## Library Installation
Download this repository as a .zip file and install in Arduino IDE (
`Sketch` - `Include Library` - `Add .ZIP Library` - Select downloaded .zip file).

Note: We plan to publish the EVN library to the Arduino IDE Library Manager by official release.

## Test Upload
Start by opening the Blink example sketch in Arduino (`File` - `Examples` - `01.Basics` - `Blink`).

To upload to EVN Alpha, select Generic RP2040 as your board (`Tools` - `Board` - `Raspberry Pi RP2040 Boards` - `Generic RP2040`), then the Generic SPI/2 Boot Stage 2 option (`Tools` - `Boot Stage 2` - `Generic SPI /4`). These settings ensure that the USB Serial connection and upload process are stable.

After selecting the correct board, press the Upload button (right arrow) in the top left corner. For fresh installs of the Arduino-Pico core, the first compile may take a while, but subsequent uploads should take less time.

Note: We plan to add the EVN Alpha to the Arduino-Pico core as its own Board option by official release.

## Usage in Arduino IDE
Detailed functions of each class are in the `/docs` folder.
Examples of each class (and combined usage) are in the `/examples` folder.


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
    //initializing motors on second core improves performance if the 2nd core is not in use
    //however, motors can still be initialized on 1st core
    motora.begin(); 
    motorb.begin();
    motorc.begin();
}

```