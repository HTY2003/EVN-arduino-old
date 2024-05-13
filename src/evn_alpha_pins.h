//PIN DEFINITIONS FOR EVN ALPHA

#define BUTTONPIN       24
#define LEDPIN          25

#define OUTPUT1MOTORA   29
#define OUTPUT1MOTORB   28
#define OUTPUT2MOTORA   27
#define OUTPUT2MOTORB   26
#define OUTPUT3MOTORA   23
#define OUTPUT3MOTORB   22
#define OUTPUT4MOTORA   21
#define OUTPUT4MOTORB   20

#define OUTPUT1ENCA     18
#define OUTPUT1ENCB     19
#define OUTPUT2ENCA     17
#define OUTPUT2ENCB     16
#define OUTPUT3ENCA     14
#define OUTPUT3ENCB     15
#define OUTPUT4ENCA     13
#define OUTPUT4ENCB     12

#define WIRE0_SDA       4
#define WIRE0_SCL       5
#define WIRE1_SDA       6
#define WIRE1_SCL       7

#define SERIAL1_RX      1
#define SERIAL1_TX      0
#define SERIAL2_RX      9
#define SERIAL2_TX      8

#define SERVO_PORT_1      2
#define SERVO_PORT_2      3
#define SERVO_PORT_3      10
#define SERVO_PORT_4      11


#define LOW_BATTERY_THRESHOLD_MV        6700

#define BQ25887_IC_I2C_PORT             16
#define BQ25887_IC_I2C_ADDRESS          0x6A

#define BQ25887_REG_ADC_CONTROL         0x15
#define BQ25887_REG_VBAT_ADC1           0x1D
#define BQ25887_REG_VCELLTOP_ADC1       0x1F
#define BQ25887_REG_PART_INFO           0x25

#define BQ25887_CMD_ADC_CONTROL_ENABLE  0b10110000
#define BQ25887_MASK_PART_INFO          0b01111000
#define BQ25887_ID                      0x05