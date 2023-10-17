/*****************************************************************************
 *                                                                           *
 *  Copyright 2018 Simon M. Werner                                           *
 *                                                                           *
 *  Licensed under the Apache License, Version 2.0 (the "License");          *
 *  you may not use this file except in compliance with the License.         *
 *  You may obtain a copy of the License at                                  *
 *                                                                           *
 *      http://www.apache.org/licenses/LICENSE-2.0                           *
 *                                                                           *
 *  Unless required by applicable law or agreed to in writing, software      *
 *  distributed under the License is distributed on an "AS IS" BASIS,        *
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. *
 *  See the License for the specific language governing permissions and      *
 *  limitations under the License.                                           *
 *                                                                           *
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"

#include "driver/i2c.h"

#include "ahrs.h"
#include "mpu9250.h"
#include "calibrate.h"
#include "common.h"

static const char *TAG = "main";

#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */

calibration_t cal = {
     .mag_offset = {.x = -8.230469, .y = 17.753906, .z = 13.117188},
    .mag_scale = {.x = 1.501793, .y = 0.898172, .z = 0.819164},
    .accel_offset = {.x = -0.001876, .y = 0.003407, .z = -0.071071},
    .accel_scale_lo = {.x = 1.005090, .y = 1.005386, .z = 0.997393},
    .accel_scale_hi = {.x = -0.993321, .y = -0.989180, .z = -1.015121},
    
    .gyro_bias_offset = {.x = -0.823844, .y = 0.010621, .z = -1.136183}};

/**
 * Transformation:
 *  - Rotate around Z axis 180 degrees
 *  - Rotate around X axis -90 degrees
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_accel_gyro(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -x;
  v->y = -z;
  v->z = -y;
}

/**
 * Transformation: to get magnetometer aligned
 * @param  {object} s {x,y,z} sensor
 * @return {object}   {x,y,z} transformed
 */
static void transform_mag(vector_t *v)
{
  float x = v->x;
  float y = v->y;
  float z = v->z;

  v->x = -y;
  v->y = z;
  v->z = -x;
}

void run_imu(void)
{

    i2c_mpu9250_init(&cal);
    ahrs_init(SAMPLE_FREQ_Hz, 0.8);

    uint64_t i = 0;
    while (true)
    {
        vector_t va, vg, vm;

        // Get the Accelerometer, Gyroscope, and Magnetometer values.
        ESP_ERROR_CHECK(get_accel_gyro_mag(&va, &vg, &vm));

        // Transform these values to the orientation of our device.
        transform_accel_gyro(&va);
        transform_accel_gyro(&vg);
        transform_mag(&vm);

        // Apply the AHRS algorithm
        ahrs_update(DEG2RAD(vg.x), DEG2RAD(vg.y), DEG2RAD(vg.z),
                    va.x, va.y, va.z,
                    vm.x, vm.y, vm.z);

        // Print all the data out every 10 items
        if (i++ % 10 == 0)
        {
            float temp;
            ESP_ERROR_CHECK(get_temperature_celsius(&temp));

            float heading, pitch, roll;
            ahrs_get_euler_in_degrees(&heading, &pitch, &roll);

            ESP_LOGI(TAG, "Accel: x=%2.3f, y=%2.3f, z=%2.3f, Gyro: x=%2.3f, y=%2.3f, z=%2.3f, Mag: x=%2.3f, y=%2.3f, z=%2.3f, Temp %2.3f°C, Heading %2.3f°, Pitch %2.3f°, Roll %2.3f°",
                     va.x, va.y, va.z, vg.x, vg.y, vg.z, vm.x, vm.y, vm.z, temp, heading, pitch, roll);

            // Make the WDT happy
            vTaskDelay(0);
        }

        pause();
    }
}

static void imu_task(void *arg)
{

#ifdef CONFIG_CALIBRATION_MODE
  calibrate_gyro();
  calibrate_accel();
  calibrate_mag();
#else
  run_imu();
#endif

  // Exit
  vTaskDelay(100 / portTICK_PERIOD_MS);
  i2c_driver_delete(I2C_MASTER_NUM);

  vTaskDelete(NULL);
}

void app_main(void)
{
  // start i2c task
  xTaskCreate(imu_task, "imu_task", 4096, NULL, 10, NULL);
}