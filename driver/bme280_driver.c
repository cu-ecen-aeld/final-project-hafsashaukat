// BME280 Linux I2C Driver
// Environmental Monitoring Final Project
// This driver registers with the Linux I2C subsystem and verifies the BME280 sensor's chip ID.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/bits.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/math64.h>

#define DRIVER_NAME "bme280_custom"
#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID     0x60

// BME280 configuration registers
#define BME280_CTRL_HUM_REG   0xF2
#define BME280_CTRL_MEAS_REG  0xF4
#define BME280_CONFIG_REG     0xF5

// first register containing measurement data
#define BME280_DATA_REG       0xF7

#define BME280_DATA_LENGTH 8


/*
 * Factory calibration coefficients stored in the BME280.
 * These values are used to convert raw ADC readings
 * into temperature, pressure, and humidity.
 */
struct bme280_calibration {
    u16 dig_T1;
    s16 dig_T2;
    s16 dig_T3;

    u16 dig_P1;
    s16 dig_P2;
    s16 dig_P3;
    s16 dig_P4;
    s16 dig_P5;
    s16 dig_P6;
    s16 dig_P7;
    s16 dig_P8;
    s16 dig_P9;

    u8  dig_H1;
    s16 dig_H2;
    u8  dig_H3;
    s16 dig_H4;
    s16 dig_H5;
    s8  dig_H6;
};

// stores info associated with the BME280 sensor
// remains available after probe() returns
struct bme280_data {
    struct i2c_client *client;
    struct bme280_calibration cal;
    struct miscdevice miscdev;
    struct mutex lock;
};

// configure the BME280 sensor
static int bme280_init_sensor(struct i2c_client *client)
{
    int ret;

    // set humidity oversampling to x1
    ret = i2c_smbus_write_byte_data(
        client, BME280_CTRL_HUM_REG, 0x01);

    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to configure humidity\n");
        return ret;
    }

    // set standby time to 1000 ms and filter off
    ret = i2c_smbus_write_byte_data(
        client, BME280_CONFIG_REG, 0xA0);

    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to configure sensor settings\n");
        return ret;
    }

    // temperature x1, pressure x1, normal mode
    ret = i2c_smbus_write_byte_data(
        client, BME280_CTRL_MEAS_REG, 0x27);

    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to configure measurement mode\n");
        return ret;
    }

    dev_info(&client->dev,
             "BME280 sensor initialized successfully\n");

    return 0;
}

// read raw pressure, temp, and humidity measurements
// sensor stores data in registers 0xF7 - 0xFE
static int bme280_read_raw(struct i2c_client *client,
                           u32 *pressure,
                           u32 *temperature,
                           u16 *humidity)
{
    u8 data[BME280_DATA_LENGTH];
    int ret;

    // read all eight measurement registers
    ret = i2c_smbus_read_i2c_block_data(
        client,
        BME280_DATA_REG,
        BME280_DATA_LENGTH,
        data
    );

    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to read sensor measurements: %d\n",
                ret);
        return ret;
    }

    if (ret != BME280_DATA_LENGTH)
        return -EIO;

    // pressure is stored as a 20-bit value
    *pressure = ((u32)data[0] << 12) |
                ((u32)data[1] << 4) |
                (data[2] >> 4);

    // temperature is stored as a 20-bit value
    *temperature = ((u32)data[3] << 12) |
                   ((u32)data[4] << 4) |
                   (data[5] >> 4);

    // humidity is stored as a 16-bit value
    *humidity = ((u16)data[6] << 8) |
                data[7];

    return 0;
}


/*
 * Read a 16-bit little-endian calibration value.
 * The BME280 stores the least significant byte first.
 */
static int bme280_read_calib_u16(struct i2c_client *client,
                                u8 reg, u16 *value)
{
    int low, high;

    low = i2c_smbus_read_byte_data(client, reg);
    if (low < 0)
        return low;

    high = i2c_smbus_read_byte_data(client, reg + 1);
    if (high < 0)
        return high;

    *value = (u16)(low | (high << 8));
    return 0;
}

// read BME280 factory calibration coefficients
static int bme280_read_calibration(
    struct i2c_client *client,
    struct bme280_calibration *cal)
{
    int ret;
    int h4_msb, h4_lsb, h5_msb;
    int value;

#define READ_CAL(reg, field) do {                         \
    ret = bme280_read_calib_u16(client, reg, &value16);  \
    if (ret < 0)                                         \
        return ret;                                      \
    cal->field = value16;                                \
} while (0)

    u16 value16;

    READ_CAL(0x88, dig_T1);
    READ_CAL(0x8A, dig_T2);
    READ_CAL(0x8C, dig_T3);

    READ_CAL(0x8E, dig_P1);
    READ_CAL(0x90, dig_P2);
    READ_CAL(0x92, dig_P3);
    READ_CAL(0x94, dig_P4);
    READ_CAL(0x96, dig_P5);
    READ_CAL(0x98, dig_P6);
    READ_CAL(0x9A, dig_P7);
    READ_CAL(0x9C, dig_P8);
    READ_CAL(0x9E, dig_P9);

#undef READ_CAL

    value = i2c_smbus_read_byte_data(client, 0xA1);
    if (value < 0)
        return value;
    cal->dig_H1 = value;

    ret = bme280_read_calib_u16(client, 0xE1, &value16);
    if (ret < 0)
        return ret;
    cal->dig_H2 = (s16)value16;

    value = i2c_smbus_read_byte_data(client, 0xE3);
    if (value < 0)
        return value;
    cal->dig_H3 = value;

    h4_msb = i2c_smbus_read_byte_data(client, 0xE4);
    h4_lsb = i2c_smbus_read_byte_data(client, 0xE5);
    h5_msb = i2c_smbus_read_byte_data(client, 0xE6);

    if (h4_msb < 0 || h4_lsb < 0 || h5_msb < 0)
        return -EIO;

    // extract and sign-extend the packed 12-bit coefficients
    cal->dig_H4 = sign_extend32((h4_msb << 4) | (h4_lsb & 0x0F), 11);
    
    cal->dig_H5 = sign_extend32((h5_msb << 4) | (h4_lsb >> 4), 11);

    value = i2c_smbus_read_byte_data(client, 0xE7);
    if (value < 0)
        return value;
    cal->dig_H6 = (s8)value;

    return 0;
}

// convert raw BME280 temp to hundreths of a degree C
// ex: 2350 represents 23.50 C
static s32 bme280_compensate_temperature(
    const struct bme280_calibration *cal,
    u32 adc_T,
    s32 *t_fine)
{
    s32 var1, var2;

    var1 = ((((s32)adc_T >> 3) -
             ((s32)cal->dig_T1 << 1)) *
             (s32)cal->dig_T2) >> 11;

    var2 = (((((s32)adc_T >> 4) -
              (s32)cal->dig_T1) *
             (((s32)adc_T >> 4) -
               (s32)cal->dig_T1)) >> 12) *
             (s32)cal->dig_T3;

    var2 = var2 >> 14;

    *t_fine = var1 + var2;

    return (*t_fine * 5 + 128) >> 8;
}

// convert raw pressure to Q24.8 fixed-point Pascals
// divide result by 256 to obtain Pascals, or by 25600 to obtain hPa
static u32 bme280_compensate_pressure(
    const struct bme280_calibration *cal,
    u32 adc_P,
    s32 t_fine)
{
    s64 var1, var2, p;

    var1 = (s64)t_fine - 128000;

    var2 = var1 * var1 * cal->dig_P6;
    var2 += (var1 * cal->dig_P5) << 17;
    var2 += ((s64)cal->dig_P4) << 35;

    var1 = ((var1 * var1 * cal->dig_P3) >> 8) +
           ((var1 * cal->dig_P2) << 12);

    var1 = (((((s64)1) << 47) + var1) *
            cal->dig_P1) >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = div64_s64(((p << 31) - var2) * 3125, var1);

    var1 = (cal->dig_P9 * (p >> 13) *
            (p >> 13)) >> 25;

    var2 = (cal->dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) +
        (((s64)cal->dig_P7) << 4);

    return (u32)p;
}

// convert raw humidity to relative humidity
// returns humidity in Q22,10 fixed-point format, divide by 1024 to get %RH
static u32 bme280_compensate_humidity(
    const struct bme280_calibration *cal,
    u16 adc_H,
    s32 t_fine)
{
    s32 v;

    v = t_fine - 76800;

    v = (((((s32)adc_H << 14) -
           ((s32)cal->dig_H4 << 20) -
           ((s32)cal->dig_H5 * v)) + 16384) >> 15) *
        (((((((v * (s32)cal->dig_H6) >> 10) *
              (((v * (s32)cal->dig_H3) >> 11) +
               32768)) >> 10) + 2097152) *
           (s32)cal->dig_H2 + 8192) >> 14);

    v = v - (((((v >> 15) * (v >> 15)) >> 7) *
               (s32)cal->dig_H1) >> 4);

    v = clamp_t(s32, v, 0, 419430400);

    return (u32)(v >> 12);
}

// read fresh BME280 measurements and return them to userspace as formatted text
static ssize_t bme280_dev_read(
    struct file *file,
    char __user *buf,
    size_t count,
    loff_t *ppos)
{
    struct miscdevice *misc = file->private_data;
    struct bme280_data *data =
        container_of(misc, struct bme280_data, miscdev);

    u32 raw_pressure, raw_temperature;
    u16 raw_humidity;
    s32 temperature, t_fine;
    u32 pressure, humidity;
    char output[128];
    int ret, len;

    ret = mutex_lock_interruptible(&data->lock);
    if (ret)
        return ret;

    ret = bme280_read_raw(
        data->client,
        &raw_pressure,
        &raw_temperature,
        &raw_humidity);

    if (ret < 0)
        goto unlock;

    temperature = bme280_compensate_temperature(
        &data->cal, raw_temperature, &t_fine);

    pressure = bme280_compensate_pressure(
        &data->cal, raw_pressure, t_fine);

    humidity = bme280_compensate_humidity(
        &data->cal, raw_humidity, t_fine);

    len = scnprintf(output, sizeof(output),
        "Temperature: %d.%02d C\n"
        "Pressure: %u.%02u hPa\n"
        "Humidity: %u.%02u %%RH\n",
        temperature / 100,
        abs(temperature % 100),
        pressure / 25600,
        (pressure % 25600) * 100 / 25600,
        humidity / 1024,
        (humidity % 1024) * 100 / 1024);

    ret = simple_read_from_buffer(
        buf, count, ppos, output, len);

unlock:
    mutex_unlock(&data->lock);
    return ret;
}

// character device operations
static const struct file_operations bme280_fops = {
    .owner = THIS_MODULE,
    .read = bme280_dev_read,
    .llseek = no_llseek,
};


// called when Linux matches the driver to a BME280 device
static int bme280_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    int chip_id;
    u32 raw_pressure;
    u32 raw_temperature;
    u16 raw_humidity;
    int ret;
    struct bme280_data *data;
    s32 temperature;
    s32 t_fine;
    u32 pressure;
    u32 humidity;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);

    if (!data)
    	return -ENOMEM;

    data->client = client;

    mutex_init(&data->lock);

    i2c_set_clientdata(client, data);

    dev_info(&client->dev, "Probing BME280 sensor\n");

    // read the sensor's identification register
    chip_id = i2c_smbus_read_byte_data(
        client, BME280_CHIP_ID_REG);

    if (chip_id < 0) {
        dev_err(&client->dev,
                "Failed to read chip ID: %d\n", chip_id);
        return chip_id;
    }

    // confirm that the connected sensor is a BME280
    if (chip_id != BME280_CHIP_ID) {
        dev_err(&client->dev,
                "Unexpected chip ID: 0x%02x\n", chip_id);
        return -ENODEV;
    }

    dev_info(&client->dev,
         "BME280 detected, chip ID: 0x%02x\n",
         chip_id);

    // read the sensor's factory calibration coefficients
    ret = bme280_read_calibration(client, &data->cal);

    if (ret < 0) {
    	dev_err(&client->dev,
            	"Failed to read calibration data: %d\n", ret);
    	return ret;
    }

    dev_info(&client->dev,
		"BME280 calibration coefficients loaded\n");

    chip_id = bme280_init_sensor(client);

    if (chip_id < 0)
    	return chip_id;

    // read one sample after sensor initialization
    msleep(50);
    
    ret = bme280_read_raw(client,
                      &raw_pressure,
                      &raw_temperature,
                      &raw_humidity);
    
    if (ret < 0)
	return ret;

    dev_info(&client->dev,
    	     "Raw measurements: T=%u P=%u H=%u\n",
             raw_temperature,
             raw_pressure,
             raw_humidity);

    temperature = bme280_compensate_temperature(&data->cal, raw_temperature, &t_fine);
	    
    dev_info(&client->dev,
         "Temperature: %d.%02d C\n",
         temperature / 100,
         abs(temperature % 100));

    pressure = bme280_compensate_pressure(&data->cal, raw_pressure, t_fine);

    dev_info(&client->dev,
         "Pressure: %u.%02u hPa\n",
         pressure / 25600,
         (pressure % 25600) * 100 / 25600);

    humidity = bme280_compensate_humidity(&data->cal, raw_humidity, t_fine);

    dev_info(&client->dev,
         "Humidity: %u.%02u %%RH\n",
         humidity / 1024,
         (humidity % 1024) * 100 / 1024);

    data->miscdev.minor = MISC_DYNAMIC_MINOR;
    data->miscdev.name = "bme280";
    data->miscdev.fops = &bme280_fops;
    data->miscdev.parent = &client->dev;
    
    ret = misc_register(&data->miscdev);
    
    if (ret) {
	 dev_err(&client->dev,
                 "Failed to register character device: %d\n",ret);
         return ret;
    }

    dev_info(&client->dev,
             "Character device /dev/bme280 registered\n");

    return 0;
}

// called when the driver is removed from the device
static void bme280_remove(struct i2c_client *client)
{
    struct bme280_data *data = i2c_get_clientdata(client);

    misc_deregister(&data->miscdev);

    dev_info(&client->dev, "BME280 driver removed\n");
}

// device names supported by this driver
static const struct i2c_device_id bme280_id[] = {
    { "bme280_custom", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, bme280_id);

// register the driver with the Linux I2C subsystem
static struct i2c_driver bme280_driver = {
    .driver = {
        .name = DRIVER_NAME,
    },
    .probe = bme280_probe,
    .remove = bme280_remove,
    .id_table = bme280_id,
};

module_i2c_driver(bme280_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Environmental Monitoring Project");
MODULE_DESCRIPTION("Custom BME280 Linux I2C driver");

