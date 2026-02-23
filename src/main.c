/*
 * Copyright (c) 2023 STMicroelectronics
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

static volatile uint32_t interrupt_count = 0;

static void trigger_handler(const struct device *dev,
                          const struct sensor_trigger *trigger)
{
    interrupt_count++;

    struct sensor_value acc[3], gyr[3];

    /* Получаем данные при наступлении прерывания */
    if (sensor_sample_fetch(dev) < 0) {
        printf("Sensor sample update error\n");
        return;
    }

    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);

    /* Вывод данных с использованием float (так как включен CONFIG_CBPRINTF_FP_SUPPORT) */
    // printf("DRDY - A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\n",
    //        sensor_value_to_double(&acc[0]),
    //        sensor_value_to_double(&acc[1]),
    //        sensor_value_to_double(&acc[2]),
    //        sensor_value_to_double(&gyr[0]),
    //        sensor_value_to_double(&gyr[1]),
    //        sensor_value_to_double(&gyr[2]));
}

int main(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(imu0));
    struct sensor_trigger trig;

    if (!device_is_ready(dev)) {
        printf("Device %s is not ready\n", dev->name);
        return 0;
    }

    printf("Device %s is ready\n", dev->name);

    /* --- Настройка прерывания Data Ready --- */
    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    if (sensor_trigger_set(dev, &trig, trigger_handler) != 0) {
        printf("Error: could not set trigger\n");
        return 0;
    }

    printf("Data Ready interrupt enabled. Waiting for samples...\n");

    while (1) {
        k_sleep(K_MSEC(1000));
        printf("--- Interrupt Frequency: %u Hz ---\n", interrupt_count);
        interrupt_count = 0;
    }
    return 0;
}