###EVN Alpha
Software Repository for EVN Alpha

## Table of contents

- [Installation](#installation)
- [Usage in Arduino IDE](#usage-in-arduino-ide)


## Installation

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

## Usage in Arduino IDE
To include all libraries, include them at the start of the library:
```
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