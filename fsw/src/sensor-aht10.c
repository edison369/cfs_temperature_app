/**
 * @file
 *
 * @brief Temperature and Humidity Sensor AHT10 Driver Implementation
 *
 *
 */

#include "sensor-aht10.h"

SENSOR_AHT10_Data_t SENSOR_AHT10_Data;

static const char bus_path[] = "/dev/i2c-1";

/*
 * Prototypes for the Private Functions
 */
static int sensor_aht10_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);

static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);
static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes);
static int sensor_aht10_get_reg_8(uint8_t register_add, uint8_t **buff);

static void updateHumidity(void);
static void updateTemperature(void);
static int readMeasurement(uint8_t **buff);

static int readStatusRegister(uint8_t **buff);
static uint8_t get_calibration_bit(void);
static void get_busy_bit(void);

/*
 * Private Functions
 */
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff){
  int rv;
  uint8_t value[nr_bytes];
  i2c_msg msgs[] = {{
    .addr = i2c_address,
    .flags = 0,
    .buf = &data_address,
    .len = 1,
  }, {
    .addr = i2c_address,
    .flags = I2C_M_RD,
    .buf = value,
    .len = nr_bytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };
  uint16_t i;

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    printf("ioctl failed...\n");
  } else {

    free(*buff);
    *buff = malloc(nr_bytes * sizeof(uint8_t));

    for (i = 0; i < nr_bytes; ++i) {
      (*buff)[i] = value[i];
    }
  }

  return rv;
}

static int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes){

  int fd;
  int rv;

  if(chip_address == 0){
    chip_address = (uint16_t) AHT10_ADDRESS_X38;
  }

  uint8_t writebuff[numBytes];

  for(int i = 0; i<numBytes; i++){
    writebuff[i] = (*val)[i];
  }

  i2c_msg msgs[] = {{
    .addr = chip_address,
    .flags = 0,
    .buf = writebuff,
    .len = numBytes,
  }};
  struct i2c_rdwr_ioctl_data payload = {
    .msgs = msgs,
    .nmsgs = sizeof(msgs)/sizeof(msgs[0]),
  };

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = ioctl(fd, I2C_RDWR, &payload);
  if (rv < 0) {
    perror("ioctl failed");
  }
  close(fd);

  return rv;
}

static int sensor_aht10_get_reg_8(uint8_t register_add, uint8_t **buff){

  int fd;
  int rv;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(1 * sizeof(uint8_t));

  uint16_t nr_bytes = (uint16_t) 1;
  uint16_t chip_address = (uint16_t) AHT10_ADDRESS_X38;
  uint8_t data_address = (uint8_t) register_add;

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  (*buff)[0] = *tmp;
  free(tmp);

  return rv;
}

static int sensor_aht10_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg){
  int err;
  uint8_t *val;
  int rv;

  switch (command) {
    case SENSOR_AHT10_SOFT_RST:

      val = NULL;
      val = malloc(1 * sizeof(uint8_t));

      val[0] = AHTXX_SOFT_RESET_REG;

      err = set_bytes(AHT10_ADDRESS_X38, &val, 1);

      OS_TaskDelay(AHTXX_SOFT_RESET_DELAY);
      break;

    case SENSOR_AHT10_NORMAL_MODE:
      OS_TaskDelay(AHTXX_CMD_DELAY);

      val = NULL;
      val = malloc(3 * sizeof(uint8_t));

      val[0] = AHT1X_INIT_REG;
      val[1] = AHTXX_INIT_CTRL_CAL_ON | AHT1X_INIT_CTRL_NORMAL_MODE;
      val[2] = AHTXX_INIT_CTRL_NOP;

      err = set_bytes(AHT10_ADDRESS_X38, &val, 3);

      break;

    case SENSOR_AHT10_READ:
      val = NULL;
      val = malloc(6 * sizeof(uint8_t));
      rv = readMeasurement(&val);

      if (rv >= 0){
        SENSOR_AHT10_Data.status = (val)[0];
        for (int i = 0; i < 5; i++) {
          SENSOR_AHT10_Data.rawData[i] = (val)[i+1];
        }
        updateHumidity();
        updateTemperature();
        err = 0;
      }else{
        printf("Error reading data...\n");
        err = -1;
      }
      break;

    default:
      err = -ENOTTY;
      break;
  }

  return err;
}

static void updateHumidity(void){
  uint32_t  humidity   = SENSOR_AHT10_Data.rawData[0];  //20-bit raw humidity data
            humidity <<= 8;
            humidity  |= SENSOR_AHT10_Data.rawData[1];
            humidity <<= 4;
            humidity  |= SENSOR_AHT10_Data.rawData[2] >> 4;

  if (humidity > 0x100000) {humidity = 0x100000;}             //check if RH>100

  SENSOR_AHT10_Data.sensor_humidity = ((float)humidity / 0x100000) * 100;
}

static void updateTemperature(void){
  uint32_t temperature   = SENSOR_AHT10_Data.rawData[2] & 0x0F; //20-bit raw temperature data
           temperature <<= 8;
           temperature  |= SENSOR_AHT10_Data.rawData[3];
           temperature <<= 8;
           temperature  |= SENSOR_AHT10_Data.rawData[4];

  SENSOR_AHT10_Data.sensor_temperature = ((float)temperature / 0x100000) * 200 - 50;
}

static int readMeasurement(uint8_t **buff){
  int fd;
  int rv;

  /* send measurement command */
  uint8_t *val;
  val = NULL;
  val = malloc(4 * sizeof(uint8_t));

  val[0] = AHTXX_START_MEASUREMENT_CTRL_NOP;
  val[1] = AHTXX_START_MEASUREMENT_REG;
  val[2] = AHTXX_START_MEASUREMENT_CTRL;
  val[3] = AHTXX_START_MEASUREMENT_CTRL_NOP;

  set_bytes(AHT10_ADDRESS_X38, &val, 4);

  /* check busy bit */
  get_busy_bit();  //update status byte, read status byte & check busy bit
  if      (SENSOR_AHT10_Data.status == AHTXX_BUSY_ERROR) {OS_TaskDelay(AHTXX_MEASUREMENT_DELAY - AHTXX_CMD_DELAY);}
  else if (SENSOR_AHT10_Data.status != AHTXX_NO_ERROR)   {return 1;}                                           //no reason to continue, received data smaller than expected

  /* read data from sensor */
  uint16_t nr_bytes = (uint16_t) 6;

  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(nr_bytes * sizeof(uint8_t));

  uint16_t chip_address = (uint16_t) AHT10_ADDRESS_X38;
  uint8_t data_address = (uint8_t) 0x00;  // No register address to read

  fd = open(&bus_path[0], O_RDWR);
  if (fd < 0) {
    printf("Couldn't open bus...\n");
    return 1;
  }

  rv = read_bytes(fd, chip_address, data_address, nr_bytes, &tmp);

  close(fd);

  for (int i = 0; i < nr_bytes; ++i) {
    (*buff)[i] = tmp[i];
  }
  free(tmp);

  /* check busy bit after measurement delay */
  get_busy_bit(); //update status byte, read status byte & check busy bit

  if (SENSOR_AHT10_Data.status != AHTXX_NO_ERROR){
    return 1;
  } //no reason to continue, sensor is busy

  return rv;

}

static int readStatusRegister(uint8_t **buff){
  int err;

  OS_TaskDelay(AHTXX_CMD_DELAY);
  uint8_t *tmp;
  tmp = NULL;

  free(*buff);
  *buff = malloc(sizeof(uint8_t));

  err = sensor_aht10_get_reg_8(AHTXX_STATUS_REG, &tmp);
  (*buff)[0] = *tmp;

  return err;
}

static uint8_t get_calibration_bit(void){
  uint8_t *value;
  value = NULL;

  readStatusRegister(&value);

  return ((*value) & AHTXX_STATUS_CTRL_CAL_ON); //0x08=loaded, 0x00=not loaded
}

static void get_busy_bit(void){

  OS_TaskDelay(AHTXX_CMD_DELAY);

  uint8_t *value;
  value = NULL;

  readStatusRegister(&value);

  if(((*value) & AHTXX_STATUS_CTRL_BUSY) == AHTXX_STATUS_CTRL_BUSY){
    SENSOR_AHT10_Data.status = AHTXX_BUSY_ERROR; //0x80=busy, 0x00=measurement completed
  }else{
    SENSOR_AHT10_Data.status = AHTXX_NO_ERROR;
  }
}

/*
 * Public Functions
 */
int i2c_dev_register_sensor_aht10(const char *bus_path, const char *dev_path){
  i2c_dev *dev;

  dev = i2c_dev_alloc_and_init(sizeof(*dev), bus_path, AHT10_ADDRESS_X38);
  if (dev == NULL) {
    return -1;
  }

  dev->ioctl = sensor_aht10_ioctl;

  return i2c_dev_register(dev, dev_path);
}

int sensor_aht10_begin(int fd){
  int err;

  // Begin variables
  SENSOR_AHT10_Data.sensor_humidity = 0;
  SENSOR_AHT10_Data.sensor_temperature = 0;
  for (int i = 0; i < 5; i++) {
    SENSOR_AHT10_Data.rawData[i] = 0;
  }

  OS_TaskDelay(100); //wait for sensor to initialize

  // Do a soft reset before setting Normal Mode
  ioctl(fd, SENSOR_AHT10_SOFT_RST, NULL);
  ioctl(fd, SENSOR_AHT10_NORMAL_MODE, NULL);
  if(get_calibration_bit() == AHTXX_STATUS_CTRL_CAL_ON){
    err = 0;
  }else{
    err = 1;
  }
  return err;
}

int sensor_aht10_read(int fd){
  return ioctl(fd, SENSOR_AHT10_READ, NULL);
}

float sensor_aht10_get_temp(void){
  return SENSOR_AHT10_Data.sensor_temperature;
}

float sensor_aht10_get_humid(void){
  return SENSOR_AHT10_Data.sensor_humidity;
}
