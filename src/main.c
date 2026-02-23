#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>
#include <stdlib.h>

// Получаем устройство из Devicetree по алиасу imu0
const struct device *const imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));

int main(void)
{
    k_sleep(K_MSEC(1000));

    printf("LSM6DSV32X Polling Example (Single Sensor)\n");

    if (!device_is_ready(imu_dev)) {
        printf("Error: Device imu0 is not ready\n");
        return 0;
    }

    printf("Sensor configured. Starting polling loop...\n");

    struct sensor_value acc[3];
    struct sensor_value gyr[3];
    
    while (1) {
        // Опрос датчика
        int ret = sensor_sample_fetch(imu_dev);
        if (ret < 0) {
            printf("Sensor fetch failed: %d\n", ret);
        } else {
            // Чтение каналов
            sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyr);

            // Вывод данных (используем sensor_value_to_double для удобства, раз включен FP_SUPPORT)
            printf("A: %.2f %.2f %.2f | G: %.2f %.2f %.2f\n",
                   sensor_value_to_double(&acc[0]),
                   sensor_value_to_double(&acc[1]),
                   sensor_value_to_double(&acc[2]),
                   sensor_value_to_double(&gyr[0]),
                   sensor_value_to_double(&gyr[1]),
                   sensor_value_to_double(&gyr[2]));
        }

        // Пауза 100 мс (10 Гц вывод)
        k_sleep(K_MSEC(100));
    }
    return 0;
}