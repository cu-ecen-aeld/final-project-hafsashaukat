// BME280 Linux I2C Driver
// Environmental Monitoring Final Project
// This driver registers with the Linux I2C subsystem
// and verifies the BME280 sensor's chip ID.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>

#define DRIVER_NAME "bme280_custom"
#define BME280_CHIP_ID_REG 0xD0
#define BME280_CHIP_ID     0x60

// BME280 configuration registers
#define BME280_CTRL_HUM_REG   0xF2
#define BME280_CTRL_MEAS_REG  0xF4
#define BME280_CONFIG_REG     0xF5

// first register containing measurement data
#define BME280_DATA_REG       0xF7

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


// called when Linux matches the driver to a BME280 device
static int bme280_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    int chip_id;

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

    // Initialize the sensor after confirming its chip ID.
    chip_id = bme280_init_sensor(client);

    if (chip_id < 0)
    	return chip_id;

    return 0;
}

// called when the driver is removed from the device
static void bme280_remove(struct i2c_client *client)
{
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

