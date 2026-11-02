
#include "IMU.h"
#include "lsm6dsv_reg.h"

#define IMU_ADDRESS 0x6A

static stmdev_ctx_t dev_ctx;


// -------------------------------
// I2C: Daten schreiben
// -------------------------------

static int32_t platform_write(
    void *handle,
    uint8_t reg,
    const uint8_t *bufp,
    uint16_t len)
{
    TwoWire *bus = (TwoWire *)handle;

    bus->beginTransmission(IMU_ADDRESS);
    bus->write(reg);
    bus->write(bufp, len);

    return bus->endTransmission();
}


// -------------------------------
// I2C: Daten lesen
// -------------------------------

static int32_t platform_read(
    void *handle,
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{
    TwoWire *bus = (TwoWire *)handle;

    bus->beginTransmission(IMU_ADDRESS);
    bus->write(reg);

    if (bus->endTransmission(false) != 0)
        return -1;

    if (bus->requestFrom(IMU_ADDRESS, (uint8_t)len) != len)
        return -1;

    for (uint16_t i = 0; i < len; i++)
        bufp[i] = bus->read();

    return 0;
}


// -------------------------------
// Sensor initialisieren
// -------------------------------

bool IMU_begin(TwoWire &bus)
{
    bus.begin();
    bus.setClock(400000);

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = [](uint32_t ms) { delay(ms); };
    dev_ctx.handle = &bus;

    uint8_t id = 0;

    if (lsm6dsv_device_id_get(&dev_ctx, &id) != 0)
        return false;

    if (id != LSM6DSV_ID)
        return false;

    lsm6dsv_block_data_update_set(
        &dev_ctx, PROPERTY_ENABLE);

    lsm6dsv_auto_increment_set(
        &dev_ctx, PROPERTY_ENABLE);

    lsm6dsv_xl_full_scale_set(
        &dev_ctx, LSM6DSV_2g);

    lsm6dsv_gy_full_scale_set(
        &dev_ctx, LSM6DSV_250dps);

    lsm6dsv_xl_data_rate_set(
        &dev_ctx, LSM6DSV_ODR_AT_120Hz);

    lsm6dsv_gy_data_rate_set(
        &dev_ctx, LSM6DSV_ODR_AT_120Hz);

    return true;
}


// -------------------------------
// Alle Messwerte auslesen
// -------------------------------

bool IMU_read(IMUData &data)
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t temp;

    if (lsm6dsv_acceleration_raw_get(
            &dev_ctx, acc) != 0)
        return false;

    if (lsm6dsv_angular_rate_raw_get(
            &dev_ctx, gyro) != 0)
        return false;

    if (lsm6dsv_temperature_raw_get(
            &dev_ctx, &temp) != 0)
        return false;


    // Beschleunigung in m/s²
    data.accX =
        lsm6dsv_from_fs2_to_mg(acc[0]) * 0.00980665f;

    data.accY =
        lsm6dsv_from_fs2_to_mg(acc[1]) * 0.00980665f;

    data.accZ =
        lsm6dsv_from_fs2_to_mg(acc[2]) * 0.00980665f;


    // Drehgeschwindigkeit in °/s
    data.gyroX =
        lsm6dsv_from_fs250_to_mdps(gyro[0]) / 1000.0f;

    data.gyroY =
        lsm6dsv_from_fs250_to_mdps(gyro[1]) / 1000.0f;

    data.gyroZ =
        lsm6dsv_from_fs250_to_mdps(gyro[2]) / 1000.0f;


    // Temperatur in °C
    data.temperature =
        lsm6dsv_from_lsb_to_celsius(temp);

    return true;
}
