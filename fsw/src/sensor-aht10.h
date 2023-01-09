/**
 * @file
 *
 * @brief Temperature and Humidity Sensor AHT10 Driver API
 *
 * @ingroup I2CSensorAHT10
 */


#ifndef _DEV_I2C_SENSOR_AHT10_H
#define _DEV_I2C_SENSOR_AHT10_H

#include <dev/i2c/i2c.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define TEMP_READ
#define HUMID_READ

/* list of I2C addresses */
#define AHT10_ADDRESS_X38                 0x38  //AHT10 I2C address if address pin to GND
#define AHT10_ADDRESS_X39                 0x39  //AHT10 I2C address, if address pin to Vcc

/* list of command registers */
#define AHT1X_INIT_REG                    0xE1  //initialization register, for AHT1x only
#define AHTXX_STATUS_REG                  0x71  //read status byte register
#define AHTXX_START_MEASUREMENT_REG       0xAC  //start measurement register
#define AHTXX_SOFT_RESET_REG              0xBA  //soft reset register

/* calibration register controls */
#define AHT1X_INIT_CTRL_NORMAL_MODE       0x00  //normal mode on/off       bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CYCLE_MODE        0x20  //cycle mode on/off        bit[6:5], for AHT1x only
#define AHT1X_INIT_CTRL_CMD_MODE          0x40  //command mode  on/off     bit[6:5], for AHT1x only
#define AHTXX_INIT_CTRL_CAL_ON            0x08  //calibration coeff on/off bit[3]
#define AHTXX_INIT_CTRL_NOP               0x00  //NOP control, send after any "AHT1X_INIT_CTRL..."

/* status byte register controls */
#define AHTXX_STATUS_CTRL_BUSY            0x80  //busy                      bit[7]
#define AHT1X_STATUS_CTRL_NORMAL_MODE     0x00  //normal mode status        bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CYCLE_MODE      0x20  //cycle mode status         bit[6:5], for AHT1x only
#define AHT1X_STATUS_CTRL_CMD_MODE        0x40  //command mode status       bit[6:5], for AHT1x only
#define AHTXX_STATUS_CTRL_CRC             0x10  //CRC8 status               bit[4], no info in datasheet
#define AHTXX_STATUS_CTRL_CAL_ON          0x08  //calibration coeff status  bit[3]
#define AHTXX_STATUS_CTRL_FIFO_ON         0x04  //FIFO on status            bit[2], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_FULL       0x02  //FIFO full status          bit[1], no info in datasheet
#define AHTXX_STATUS_CTRL_FIFO_EMPTY      0x02  //FIFO empty status         bit[1], no info in datasheet

/* measurement register controls */
#define AHTXX_START_MEASUREMENT_CTRL      0x33  //measurement controls, suspect this is temperature & humidity DAC resolution
#define AHTXX_START_MEASUREMENT_CTRL_NOP  0x00  //NOP control, send after any "AHTXX_START_MEASUREMENT_CTRL..."

/* sensor delays */
#define AHTXX_CMD_DELAY          10      //delay between commands, in milliseconds
#define AHTXX_MEASUREMENT_DELAY  80      //wait for measurement to complete, in milliseconds
#define AHT1X_POWER_ON_DELAY     40      //wait for AHT1x to initialize after power-on, in milliseconds
#define AHTXX_SOFT_RESET_DELAY   20      //less than 20 milliseconds

/* misc */
#define AHTXX_I2C_SPEED_HZ       100000  //sensor I2C speed 100KHz..400KHz, in Hz
#define AHTXX_I2C_STRETCH_USEC   1000    //I2C stretch time, in usec

#define AHTXX_NO_ERROR           0x00    //success, no errors
#define AHTXX_BUSY_ERROR         0x01    //sensor is busy
#define AHTXX_ACK_ERROR          0x02    //sensor didn't return ACK (not connected, broken, long wires (reduce speed), bus locked by slave (increase stretch limit))
#define AHTXX_DATA_ERROR         0x03    //received data smaller than expected
#define AHTXX_ERROR              0xFF    //other errors

/**************************************************************************/
/*
    NOTE:
    - Relative humidity range........ 0%..100%
    - Relative humidity resolution... 0.024%
    - Relative humidity accuracy..... +-2%
    - Temperature range........ -40C..+85C
    - Temperature resolution... 0.01C
    - Temperature accuracy..... +-0.3C
    - Response time............ 5..30sec
    - Measurement with high frequency leads to heating of the
      sensor, must be > 2 seconds apart to keep self-heating below 0.1C

    - Sensors data structure:
      - {status, RH, RH, RH+T, T, T}

    - Status register controls:
      7    6    5    4   3    2   1   0
      BSY, MOD, MOD, xx, CAL, xx, xx, xx
      - BSY:
        - 1, sensor busy/measuring
        - 0, sensor idle/sleeping
      - MOD:
        - 00, normal mode
        - 01, cycle mode
        - 1x, comand mode
      - CAL:
        - 1, calibration on
        - 0, calibration off
    - under normal conditions status is 0x18 & 0x80 if the sensor is busy
*/
/**************************************************************************/

/**
 * @defgroup I2CSensorAHT10 Temperature and Humidity Sensor AHT10 Driver
 *
 * @ingroup I2CDevice
 *
 * @brief Driver for AHT10 Temperature and Humidity sensor.
 *
 * @{
 */

#define delay_ms(m) rtems_task_wake_after(1 + ((m)/rtems_configuration_get_milliseconds_per_tick()))

#define temperature_read
#define humidity_read

typedef enum {
  SENSOR_AHT10_SOFT_RST,
  SENSOR_AHT10_NORMAL_MODE,
  SENSOR_AHT10_READ
} sensor_aht10_command;

typedef struct{
  float sensor_humidity;
  float sensor_temperature;
  uint8_t rawData[5];
  uint8_t status;
}SENSOR_AHT10_Data_t;


int i2c_dev_register_sensor_aht10(const char *bus_path, const char *dev_path);
int sensor_aht10_begin(int fd);
int sensor_aht10_read(int fd);

// I2C functions
float sensor_aht10_get_temp(void);
float sensor_aht10_get_humid(void);


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_I2C_SENSOR_AHT10_H */
