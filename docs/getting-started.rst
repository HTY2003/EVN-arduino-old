Getting Started
===============

What you Need
-------------

Here's the list of items you need to get EVN Alpha up and running:

* 1 EVN Alpha Board, fully assembled with headers, ports and a battery holder
* 2 18650 Cells
* 1 USB 2.0-capable Type C cable for program upload

* 1 5V 3A USB Type-C Power Source for charging

    * Most modern laptops are capable of this, but any charging brick works too

If you have purchased a Hacker Kit, refer to our Soldering & Assembly Guide (coming soon!) on how to solder a board.

18650 Battery Selection
-----------------------

In EVN Alpha, 2 18650 cells are connected in series for a nominal voltage of 7.4V.

Current Rating
^^^^^^^^^^^^^^
One load the batteries have to supply current for are the 3.3V and 5V regulators which draw a maximum 5A of battery current. Another load is the motor drivers, which are rated for a 3A continuous load current with 4A max.
Therefore, each cell would need to be rated for **21A** of maximum discharge current to push the onboard hardware to its limits.


But thanks to the work by `PhiloHome`_, we know the stall current of the NXT and EV3 Motors currently supported:

.. _PhiloHome: https://www.philohome.com/motors/motorcomp.htm

* NXT Large Servo:  2A
* EV3 Large Servo:  1.8A
* EV3 Medium Servo: 0.8A

So if you only intend to use LEGO motors, a current rating of 13A is sufficient (enough for 4 stalled NXT motors at the same time).

Protected Cells
^^^^^^^^^^^^^^^

EVN Alpha contains the following protection circuits for the cells:

* Reverse Voltage Protection (the board and cells are protected if either/both cells are connected in reverse)
* Undervoltage Lockout (if the combined voltage dips below 6.3V, the board is locked into charging mode only to prevent excessive over-discharge)
* Overvoltage Protection (charging is suspended when either cell is over-voltage)
* 160mA Passive Cell Balancing (in Charging Mode)

These protections constitute the minimum requirements for us to feel comfortable selling a product with user-installed and user-replaceable 18650 cells, but they are not foolproof.

If you wish an additional layer of safety, we recommend using protected 18650 cells, which have similar protections built into the housing, as well as temperature monitoring.

Other Details
^^^^^^^^^^^^^
Both batteries should have similar capacity, discharge characteristics, state of charge and level of wear (i.e. 2 new fully-charged batteries of the same model is best).

.. warning::
    
    Remember that proper handling and usage are crucial for safety when dealing with lithium-ion batteries. Always follow manufacturer guidelines and avoid using knock-off or low-quality brands. 

Battery Installation
--------------------
1. Remove the battery cover from the brick. Ensure that the USB-C port is **not** plugged in.

2. There are **2** battery slots in the holder. Inspect the battery polarity markings for each slot on the battery holder and battery cover (which should match each other). Note that the negative terminal of one cell should be near the postive terminal of the other cell, and vice versa.

3. Install the inner cell into place (the battery slot further from the edge of the circuit board). It is recommended to contact the cell's negative terminal with the - contact on the holder first, before pressing the positive end into place.

4. Install the outer cell into place using the same instructions, making sure to orient the cell properly.

5. Firmly press down on both cells to ensure they are well seated in the holder.

Power-On Test
-------------

Once the batteries are inserted, pressing the on/off button (rightmost on the board) should toggle the red Power LED on and off.

If the red Power LED does not turn on, there are 2 possibilities:

* The cells may be inserted in the wrong orientation. Check the polarities by referring to the battery cover, and correct if necessary. Using a screwdriver/stick may be helpful in removing the inner cell.

* The battery voltages are too low. Use a multimeter or battery checker to check if the cells are sufficiently charged. If the combined cell voltage is below 6.3V, EVN Alpha will not power on.

If the board still fails to power on, feel free to reach out on our Discord and we'll assist you.

Charging Test
-------------
TBA

Board Detection Test
--------------------
TBA


Proceed to Hardware Overview to learn more about EVN's hardware functions.