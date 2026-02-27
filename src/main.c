#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>
#include <stdlib.h>

#define IMU_COUNT 4

#define MONITOR_STACK_SIZE 2048
#define WORKER_STACK_SIZE  2048

#define WORKER_PRIORITY    1
#define MONITOR_PRIORITY   5

#define MASTER_IMU_INDEX 0

K_SEM_DEFINE(imu_trigger_sem, 0, 1);

struct imu_context {
	const struct device *dev;
	atomic_t sample_count;
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


void init_sensors(void)
{
	printf("Initializing %d sensors...\n", IMU_COUNT);

	for (int i = 0; i < IMU_COUNT; i++) {
		imu_ctxs[i].dev = imu_devs[i];
		atomic_set(&imu_ctxs[i].sample_count, 0);
		const struct device *dev = imu_ctxs[i].dev;

		if (!device_is_ready(dev)) {
			printf("Error: Device imu%d not ready\n", i);
			continue;
		}

		printf("IMU%d ready.\n", i);
	}

}

void imu_worker_thread_entry(void *p1, void *p2, void *p3)
{
	while (1) {
		k_msleep(16); /* Опрос ~60Гц */

		for (int i = 0; i < IMU_COUNT; i++) {
			struct imu_context *ctx = &imu_ctxs[i];
			if (!ctx->dev) continue;
			if (sensor_sample_fetch(ctx->dev) == 0) {
				sensor_channel_get(ctx->dev, SENSOR_CHAN_ACCEL_XYZ, ctx->acc);
				sensor_channel_get(ctx->dev, SENSOR_CHAN_GYRO_XYZ, ctx->gyr);
                atomic_inc(&ctx->sample_count);
			}
		}
	}
}

void monitor_thread_entry(void *p1, void *p2, void *p3)
{
	int64_t last_time = k_uptime_get();

	while (1) {
		k_sleep(K_SECONDS(2));

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

K_THREAD_DEFINE(worker_tid, WORKER_STACK_SIZE, imu_worker_thread_entry, NULL, NULL, NULL,
		WORKER_PRIORITY, 0, 0);

int main(void)
{
	init_sensors();
	
	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}