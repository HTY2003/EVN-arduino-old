/*readGesture.ino

The following program demonstrates some basic EVNGestureSensor functionality.
*/

#include <EVN.h>

#define GESTURE_SENS_PORT 1  //set I2C port for gesture sensor here

EVNAlpha board;
EVNGestureSensor gesture(GESTURE_SENS_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    Serial.begin(9600);
    gesture.begin();
}

void loop()
{
    int direction = gesture.readGesture();

    switch (direction)
    {
    case GESTURE_LEFT:
        Serial.println("Left Gesture Detected!");
        break;

    case GESTURE_RIGHT:
        Serial.println("Right Gesture Detected!");
        break;

    case GESTURE_UP:
        Serial.println("Up Gesture Detected!");
        break;

    case GESTURE_DOWN:
        Serial.println("Down Gesture Detected!");
        break;
    }

    delay(100);
}