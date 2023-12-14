# EVNDisplay
EVNDisplay is a class used to interface EVN Alpha with our OLED Standard Peripheral, which uses a 128x64 OLED Display with SSD1315 IC (also works with SSD1306).

## Some Technical Details
EVNDisplay is a wrapper library for u8x8 from [u8g2](https://github.com/olikraus/u8g2), an incredibly versatile graphics library with many fonts and drawing functions. The reasons we made our own library around it are twofold:

1. Most of the time, displays are used to debug text information.
2. Updating the display takes time. On I2C, it's particularly slow.


This library attempts to address both problems by focusing on using the display for data logging. The functions below allow the user to print to 1 of 8 rows. They can print a row label once in `void setup()`, which appears at the start of the row. In `void loop`, they can print data which appears after the row label. By only refreshing pixels after the row label, time is saved for other timing-critical functions in the loop.


Note: EVNDisplay does port selection and deselection automatically, so users do not need to call `setPort()` on their `EVNAlpha` objects.


## List of Functions
- [Constructor](#constructor)
- [begin()](#void-begin)
- [rotate()](#void-rotate)
- [splashEVN()](#void-splashevn)
- [clear()](#void-clear)
- [clearLine()](#void-clearlineuint8t-row)
- [writeLabel()](#void-writelabeluint8t-row-label) / [writeLine()](#void-writelineuint8t-row-label) / [write()](#void-writeuint8t-row-label) / [print()](#void-printuint8t-row-label) 
- [writeData()](#void-writedatauint8t-row-data)

## Constructor
##### `EVNDisplay(port, rotate = DISPLAY_0DEG)`

Arguments:
* port: the I2C port the display is connected to (1-16, left to right)
* rotate:
    * `DISPLAY_0DEG` (default, text is displayed in the same orientation as the connector labels)
    * `DISPLAY_180DEG` (display is rotated by 180deg)

Example:
```
EVNDisplay display;
```
Another example:
```
EVNDisplay display(DISPLAY_180DEG);
```

## Functions
##### `void begin()`
Initializes display and clears any on-screen data from a previous program.

Example:
```
void setup()
{
    display.begin();
    //...
}
```

##### `void rotate()`
Rotates display by 180deg.

Example:
```
void setup()
{
    display.begin();
    display.rotate();
    //...
}
```

##### `void splashEVN()`
Splash-screens a big EVN logo on the display, with an animation.

Example:
```
void setup()
{
    display.begin();
    display.splashEVN();
    //...
}
```

##### `void clear()`
Clears all on-screen data from display. Useful for removing big unnecessar...ily cool splash screens.

Example:
```
void setup()
{
    display.begin();
    display.splashEVN();
    display.clear();
    //...
}
```

### **void clearLine(uint8_t row)**
Clears data in given row.

Arguments:
* row: row number to be cleared (0-7), row 0 is the topmost row

Example:
```
void loop()
{
    display.clearLine(0);
}
```

##### `void writeLabel(uint8_t row, label)`
Writes data to the start of the given row

Returns:
* row number (0-7), row 0 is the topmost row
* data: any data type is accepted

Example:
```
void setup()
{
    display.begin();
    display.writeLabel(0, "LEFT: ");
    //...
}
```

##### `void writeLine(uint8_t row, label)`
##### `void write(uint8_t row, label)`
##### `void print(uint8_t row, label)`
Same as [writeLabel()](#void-writelabeluint8t-row-label).

##### `void writeData(uint8_t row, data)`
Writes data to the given row, after the row label

Returns:
* row number (0-7), row 0 is the topmost row
* data: any data type is accepted

Example:
```
void loop()
{
    int reading = 0;
    //get reading from sensor
    display.writeLabel(reading);
    //...
}
```