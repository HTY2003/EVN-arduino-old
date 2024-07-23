/*readIMU.ino

The following program demonstrates some basic EVNIMUSensor functionality.
*/

#include <EVN.h>

#define IMU_I2C_PORT 1  //set I2C port for IMU sensor here

EVNAlpha board;
EVNIMUSensor imu(IMU_I2C_PORT);

void setup()
{
    board.begin();  //initialize board at start of void setup()
    imu.begin();    //sensor initialization comes after
    Serial.begin(9600);
}

void loop()
{
    float accel_x = imu.readAccelX();
    float accel_y = imu.readAccelY(false);
    float accel_z = imu.readAccelZ(false);
    float gyro_x = imu.readGyroX(false);
    float gyro_y = imu.readGyroY(false);
    float gyro_z = imu.readGyroZ(false);

    Serial.print("Accel X ");
    Serial.print(accel_x);
    Serial.print(" Y ");
    Serial.print(accel_y);
    Serial.print(" Z ");
    Serial.print(accel_z);

    Serial.print(" | Gyro X ");
    Serial.print(gyro_x);
    Serial.print(" Y ");
    Serial.print(gyro_y);
    Serial.print(" Z ");
    Serial.println(gyro_z);
}