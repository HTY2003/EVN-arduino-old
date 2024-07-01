Connecting 3rd-Party I2C Devices
================================

This guide is for those who wish to interface third-party I2C peripherals. 
Example Arduino code for these devices will work with EVN Alpha, but requires some code modifications.

For those who want to get straight to the point, skip `here`_.

.. _here: third-party-i2c.html#making-changes-to-code

How I2C Works
-------------

I2C uses a **shared bus**, where **multiple peripherals** are connected to the same 2 pins on the host: **SDA (Serial Data) and SCL (Serial Clock)**. 

It's a serial bus since bits are sent one after another, similar to USB (Universal Serial Bus).
In that sense, the Arduino terminology of Serial and what we call "Serial Ports" is a bit reductive, as it actually refers to **Serial UART** 
(Universal Asynchronous Receive/Transmit).


The I2C host (EVN Alpha in this case) identifies each peripheral on the bus by its **I2C address**.

Think of the I2C address as a **name**. The host calls for a specific peripheral's address to start a transmission. 
That peripheral responds and either transmits or receives data.
Meanwhile, the other peripherals ignore everything until that transmission ends, as their "names" have not been called.

Why Modifications Are Needed
------------------------------

The trouble arises when multiple devices with the **same address** are connected to the same bus.
If 2 (or more) devices try to transmit data at the same time, nothing works.

There are some solutions for this:

1. Some peripherals expose a **pin** you can use to **enable/disable** them. To communicate with a given peripheral, we can disable all other peripherals with the same address using IO pins.

2. Some peripherals have **pins or pads** that you can solder together to **change their I2C address**, so there are no address clashes.

But these workarounds require peripherals to **support** those functions (not all do), along with additional IO pins or soldering irons.

Hence, we decided on this:

3. Use an **I2C multiplexer**. I2C peripherals are connected to the multiplexer (instead of directly to the bus), and the multiplexer is connected to the I2C bus. The multiplexer acts like a **switch**, able to **control which I2C peripheral is visible** on the shared bus.

It's similar to solution #1, except that we use the multiplexer and I2C commands to make other peripherals invisible instead of digital pins.

EVN Alpha uses an 8-channel TCA9548A multiplexer on both I2C buses, which represent I2C ports 1-16.

Our I2C communications process has now changed from:

* Call for peripheral's I2C address to start transmission
* Transmit / Request for data from peripheral
* End transmission

To:

* **Call for multiplexer's I2C address to start transmission**
* **Transmit command to multiplexer to make peripheral on desired port (e.g. port 1) visible on I2C bus (peripherals on the multiplexer's other ports are made invisible)**
* **End transmission**

* Call for peripheral's I2C address to start transmission
* Transmit / request for data from peripheral
* End transmission

Long story short, there is an **additional I2C transmission** to make before interacting with the peripheral normally. This is the modification we must make.

Making Changes to Code
-----------------------

For this guide, we will use example code for DFRobot's HuskyLens. This is lifted from their `wiki`_.

.. _wiki: https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336

.. code-block:: c++

    #include "HUSKYLENS.h"
    #include "SoftwareSerial.h"

    HUSKYLENS huskylens;
    //HUSKYLENS green line >> SDA; blue line >> SCL
    void printResult(HUSKYLENSResult result);

    void setup() {
        Serial.begin(115200);
        Wire.begin();
        while (!huskylens.begin(Wire))
        {
            Serial.println(F("Begin failed!"));
            Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
            Serial.println(F("2.Please recheck the connection."));
            delay(100);
        }
    }

    void loop() {
        if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
        else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
        else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
        else
        {
            Serial.println(F("###########"));
            while (huskylens.available())
            {
                HUSKYLENSResult result = huskylens.read();
                printResult(result);
            }    
        }
    }

    void printResult(HUSKYLENSResult result){
        if (result.command == COMMAND_RETURN_BLOCK){
            Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
        }
        else if (result.command == COMMAND_RETURN_ARROW){
            Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
        }
        else{
            Serial.println("Object unknown!");
        }
    }

Here are the following changes:
* Include the EVN library header (if not already included)
* Declare an EVNAlpha object 
* Initialize the EVNAlpha object in ``void setup()``
* Call ``board.setPort(the peripheral's I2C port)`` whenever we are about to begin, read or write to the HuskyLens. For this example, let's assume the HuskyLens is connected to I2C Port 1. 

These 4 changes actually amount to just 5 lines, which have been labelled with ``//EVN modification``.

.. code-block:: c++

    #include "HUSKYLENS.h"
    #include "SoftwareSerial.h"
    #include "EVN.h"    //EVN modification

    HUSKYLENS huskylens;
    //HUSKYLENS green line >> SDA; blue line >> SCL

    EVNAlpha board;     //EVN modification

    void printResult(HUSKYLENSResult result);

    void setup() {
        board.begin();  //EVN modification
        Serial.begin(115200);
        Wire.begin();

        board.setPort(1);      //EVN modification
        while (!huskylens.begin(Wire))
        {
            Serial.println(F("Begin failed!"));
            Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
            Serial.println(F("2.Please recheck the connection."));
            delay(100);
        }
    }

    void loop() {
        board.setPort(1);   //EVN modification
        if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
        else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
        else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
        else
        {
            Serial.println(F("###########"));
            while (huskylens.available())
            {
                HUSKYLENSResult result = huskylens.read();
                printResult(result);
            }    
        }
    }

    void printResult(HUSKYLENSResult result){
        if (result.command == COMMAND_RETURN_BLOCK){
            Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
        }
        else if (result.command == COMMAND_RETURN_ARROW){
            Serial.println(String()+F("Arrow:xOrigin=")+result.xOrigin+F(",yOrigin=")+result.yOrigin+F(",xTarget=")+result.xTarget+F(",yTarget=")+result.yTarget+F(",ID=")+result.ID);
        }
        else{
            Serial.println("Object unknown!");
        }
    }

That's it! 5 additional lines of code.

As long as you always call ``board.setPort(port_number)`` before you communicate with the peripheral, everything should work just fine.

Final Notes and Limitations
---------------------------

Here are some final things to keep note of when getting your third-party devices to work:

* **I2C ports 9-16** are connected to the I2C1 bus, controlled using the ``Wire1`` class in Arduino. Your third-party library will have to be set to use ``Wire1`` instead of ``Wire`` for these ports.

* The multiplexer allows for 2 peripherals with the same address to be connected at the same time, but no connected peripherals can clash with the multiplexer's I2C address, which is **0x70**.

* EVN libraries have port selection built-in, so you do not need to call ``board.setPort(port_number)`` for Standard Peripherals/battery voltage measurement. However, the ports are not de-selected, so after using an EVN library function you must call ``board.setPort(port_number)`` again to use your third-party peripheral.

* The multiplexers are only rated for an I2C frequency of up to 400kHz. Higher than that, your mileage may vary.

* If you wish, you may control the multiplexer using ``Wire`` functions instead of ``board.setPort(port_number)``. However, this may break functionality with the EVN libraries, and ``board.setPort(port_number)`` is optimized to avoid sending unnecessary I2C commands if the correct port is already selected, so we strongly recommend using it.