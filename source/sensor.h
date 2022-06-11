/*
 * sensor.h
 *
 *  Created on: May 15, 2022
 *      Author: navin
 */

#ifndef SOURCE_SENSOR_H_
#define SOURCE_SENSOR_H_

/* FreeRTOS header file. */
#include <FreeRTOS.h>
#include <queue.h>

#include "cyabs_rtos.h"

typedef struct
{
	uint8_t* pData;
	uint32_t dataSize;
}sensor_data_t;

typedef struct __attribute__((packed)) bmi160_sensor_data_accel
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

    uint32_t timestamp;

}imu_sensor_accel_t;

typedef struct __attribute__((packed)) bmi160_sensor_data_gyro
{
    /*! X-axis sensor data */
	int16_t x;

    /*! Y-axis sensor data */
	int16_t y;

    /*! Z-axis sensor data */
	int16_t z;

	uint32_t timestamp;

}imu_sensor_gyro_t;

typedef struct __attribute__((packed)) imu_sensor_data
{
    imu_sensor_accel_t accel;
    imu_sensor_gyro_t gyro;
}imu_sensor_data_t;

extern QueueHandle_t qSensorData;
/*******************************************************************************
* Functions
*******************************************************************************/
cy_rslt_t sensor_init(void);
void sensor_task(void *arg);

#endif /* SOURCE_SENSOR_H_ */
