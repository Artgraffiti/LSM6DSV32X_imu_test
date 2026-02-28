#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>

#define IMU_COUNT 4
#define MASTER_IMU_INDEX 0

#define WORKER_STACK_SIZE  2048
#define MONITOR_STACK_SIZE 2048

#define WORKER_PRIORITY    2
#define MONITOR_PRIORITY   5

struct imu_context {
    const struct device *dev;
    atomic_t sample_count;
    struct sensor_value acc[3];
    struct sensor_value gyr[3];
};

// Структура пакета данных для передачи через очередь
struct imu_sensor_data {
    struct sensor_value acc[IMU_COUNT][3];
    struct sensor_value gyr[IMU_COUNT][3];
};

static struct imu_context imu_ctxs[IMU_COUNT];

static const struct device *const imu_devs[IMU_COUNT] = {
    DEVICE_DT_GET(DT_ALIAS(imu0)),
    DEVICE_DT_GET(DT_ALIAS(imu1)),
    DEVICE_DT_GET(DT_ALIAS(imu2)),
    DEVICE_DT_GET(DT_ALIAS(imu3)),
};

static volatile uint32_t interrupt_count = 0;

// Создаем очередь на 10 пакетов данных
K_MSGQ_DEFINE(imu_msgq, sizeof(struct imu_sensor_data), 10, 4);

// 1. Обработчик прерывания: ТОЛЬКО читает данные (чтобы сбросить пин) и кладет их в очередь
static void trigger_handler(const struct device *dev,
                          const struct sensor_trigger *trigger)
{
    interrupt_count++;
    struct imu_sensor_data data;

    for (int i = 0; i < IMU_COUNT; i++) {
        struct imu_context *ctx = &imu_ctxs[i];
        if (!ctx->dev || !device_is_ready(ctx->dev)) continue;

        // Чтение обязательно должно быть здесь, чтобы железо сняло флаг DRDY
        if (sensor_sample_fetch(ctx->dev) == 0) {
            sensor_channel_get(ctx->dev, SENSOR_CHAN_ACCEL_XYZ, data.acc[i]);
            sensor_channel_get(ctx->dev, SENSOR_CHAN_GYRO_XYZ, data.gyr[i]);
            atomic_inc(&ctx->sample_count);
        }
    }

    // Отправляем пакет в очередь без блокировки потока (K_NO_WAIT)
    k_msgq_put(&imu_msgq, &data, K_NO_WAIT);
}

// 2. Рабочий поток: просыпается при поступлении данных и выполняет основную логику
void worker_thread_entry(void *p1, void *p2, void *p3)
{
    struct imu_sensor_data data;

    while (1) {
        // Поток спит, пока в очереди не появятся данные
        if (k_msgq_get(&imu_msgq, &data, K_FOREVER) == 0) {
            
            // ЗДЕСЬ МЕСТО ДЛЯ ВАШЕЙ ЛОГИКИ:
            // Фильтрация (Kalman/Mahony), математика, отправка по Bluetooth/WiFi.
            
            // В качестве примера: сохраняем данные в глобальный массив для монитора
            for (int i = 0; i < IMU_COUNT; i++) {
                for(int j = 0; j < 3; j++) {
                    imu_ctxs[i].acc[j] = data.acc[i][j];
                    imu_ctxs[i].gyr[j] = data.gyr[i][j];
                }
            }
        }
    }
}

K_THREAD_DEFINE(worker_tid, WORKER_STACK_SIZE, worker_thread_entry, NULL, NULL, NULL,
                WORKER_PRIORITY, 0, 0);

void init_sensors(void)
{
    struct sensor_trigger trig;

    trig.type = SENSOR_TRIG_DATA_READY;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    printf("Initializing %d sensors...\n", IMU_COUNT);

    for (int i = 0; i < IMU_COUNT; i++) {
        imu_ctxs[i].dev = imu_devs[i];
        atomic_set(&imu_ctxs[i].sample_count, 0);
        
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
        k_sleep(K_SECONDS(1));

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

            atomic_val_t count = atomic_set(&ctx->sample_count, 0);
            unsigned long fps = (unsigned long)((count * 1000) / delta_ms);

            printf("[IMU%d] %4lu Hz | A: %3d.%02d %3d.%02d %3d.%02d | G: %3d.%02d %3d.%02d %3d.%02d\n",
                   i, fps, 
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