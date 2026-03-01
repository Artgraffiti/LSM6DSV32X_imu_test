#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define IMU_COUNT 4
#define MASTER_IMU_INDEX 0

#define MONITOR_STACK_SIZE 2048

#define MONITOR_PRIORITY   5

struct imu_context {
    const struct device *dev;
    struct sensor_value acc[3];
    struct sensor_value gyr[3];
};

static struct imu_context imu_ctxs[IMU_COUNT];

static const struct device *const imu_devs[IMU_COUNT] = {
    DEVICE_DT_GET(DT_ALIAS(imu0)),
    DEVICE_DT_GET(DT_ALIAS(imu1)),
    DEVICE_DT_GET(DT_ALIAS(imu2)),
    DEVICE_DT_GET(DT_ALIAS(imu3)),
};

static volatile uint32_t interrupt_count = 0;

// 1. Обработчик прерывания: ТОЛЬКО читает данные (чтобы сбросить пин) и кладет их в очередь
static void trigger_handler(const struct device *dev,
                          const struct sensor_trigger *trigger)
{
    interrupt_count++;

    for (int i = 0; i < IMU_COUNT; i++) {
        struct imu_context *ctx = &imu_ctxs[i];
        if (!ctx->dev || !device_is_ready(ctx->dev)) continue;

        // Чтение обязательно должно быть здесь, чтобы железо сняло флаг DRDY
        if (sensor_sample_fetch(ctx->dev) == 0) {
            sensor_channel_get(ctx->dev, SENSOR_CHAN_ACCEL_XYZ, ctx->acc);
            sensor_channel_get(ctx->dev, SENSOR_CHAN_GYRO_XYZ, ctx->gyr);
        }
    }
}

void init_sensors(void)
{
    struct sensor_trigger trig;

    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    printf("Initializing %d sensors...\n", IMU_COUNT);

    for (int i = 0; i < IMU_COUNT; i++) {
        imu_ctxs[i].dev = imu_devs[i];
        
        if (!device_is_ready(imu_ctxs[i].dev)) {
            printf("Error: Device imu%d not ready\n", i);
            continue;
        }

        printf("IMU%d initialized from Device Tree configs.\n", i);
    }

    const struct device *master_dev = imu_ctxs[MASTER_IMU_INDEX].dev;

    if (device_is_ready(master_dev)) {
        printf("All sensors checked. Enabling trigger on MASTER (IMU%d)...\n", MASTER_IMU_INDEX);
        
        if (sensor_trigger_set(master_dev, &trig, trigger_handler) != 0) {
            printf("Error: Could not set trigger for MASTER imu%d\n", MASTER_IMU_INDEX);
        } else {
            printf("Trigger enabled. Data stream started.\n");
        }
    } else {
        printf("Error: Master device is not ready, cannot set trigger.\n");
    }
}

void monitor_thread_entry(void *p1, void *p2, void *p3)
{
    int64_t last_time = k_uptime_get();

    while (1) {
        k_sleep(K_SECONDS(10));

        int64_t now = k_uptime_get();
        int64_t delta_ms = now - last_time;
        last_time = now;

        if (delta_ms <= 0) delta_ms = 1;

        printf("\n=== IMU Status (Trigger: IMU%d) | Interval: %lld ms ===\n", MASTER_IMU_INDEX, delta_ms);
        
        for (int i = 0; i < IMU_COUNT; i++) {
            struct imu_context *ctx = &imu_ctxs[i];
            
            if (!device_is_ready(ctx->dev)) {
                printf("[IMU%d] Not ready\n", i);
                continue;
            }

            printf("[IMU%d] A: %3d.%02d %3d.%02d %3d.%02d | G: %3d.%02d %3d.%02d %3d.%02d\n",
                   i, 
                   ctx->acc[0].val1, abs(ctx->acc[0].val2 / 10000),
                   ctx->acc[1].val1, abs(ctx->acc[1].val2 / 10000),
                   ctx->acc[2].val1, abs(ctx->acc[2].val2 / 10000),
                   ctx->gyr[0].val1, abs(ctx->gyr[0].val2 / 10000),
                   ctx->gyr[1].val1, abs(ctx->gyr[1].val2 / 10000),
                   ctx->gyr[2].val1, abs(ctx->gyr[2].val2 / 10000));
        }
        printf("=======================================\n");
    }
}

K_THREAD_DEFINE(monitor_tid, MONITOR_STACK_SIZE, monitor_thread_entry, NULL, NULL, NULL,
                MONITOR_PRIORITY, 0, 0);
                
int main(void)
{
    init_sensors();

    while (1) {
        k_sleep(K_MSEC(1000));
        printf("--- Interrupt Frequency: %u Hz ---\n", interrupt_count);
        interrupt_count = 0;
    }
    return 0;
}