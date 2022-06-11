/*
 * sensor.c
 *
 *  Created on: May 15, 2022
 *      Author: navin
 */

#include "cyhal.h"
#include "cybsp.h"

#include "mtb_bmx160.h"
#include "mtb_bmi160.h"

#include "cy_fifo.h"
#include "sensor.h"
#include "cycfg_pins.h"
#include "ei_c_wrapper.h"
/*******************************************************************************
* Typedefs
*******************************************************************************/


/*******************************************************************************
* Constants
*******************************************************************************/

#define SENSOR_EVENT_BIT 1u<<2

#ifdef CY_BMX_160_IMU
#define IMU_SPI_FREQUENCY 10000000
#endif

#ifdef CY_BMI_160_IMU
    #define IMU_I2C_MASTER_DEFAULT_ADDRESS    0
    #define IMU_I2C_FREQUENCY                1000000
#endif

#define SENSOR_DATA_WIDTH 2
#define SENSOR_NUM_AXIS   6
#define SENSOR_TIMESTAMP  8
#define SENSOR_SAMPLE_SIZE ((SENSOR_DATA_WIDTH * SENSOR_NUM_AXIS) + 8)
#define SENSOR_BATCH_SIZE  64

#define SENSOR_FIFO_ITEM_SIZE (SENSOR_SAMPLE_SIZE / 2)
#define SENSOR_FIFO_POOL_SIZE (2 * SENSOR_BATCH_SIZE * sizeof(imu_sensor_data_t))

#define SENSOR_SCAN_RATE       128
#define SENSOR_TIMER_FREQUENCY 100000
#define SENSOR_TIMER_PERIOD (SENSOR_TIMER_FREQUENCY/SENSOR_SCAN_RATE)
#define SENSOR_TIMER_PRIORITY  3

#define MAX_DATA_SAMPLE 32768
#define MIN_DATA_SAMPLE -32768

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Global timer used for getting data */
cyhal_timer_t sensor_timer;

/* Event set when data is done collecting */
cy_event_t sensor_event;
uint32_t sensor_event_bits  = SENSOR_EVENT_BIT;

/* Circle buffer to store IMU data */
static cy_fifo_t sensor_fifo;
static uint8_t sensor_fifo_pool[SENSOR_FIFO_POOL_SIZE];
static float features[SENSOR_BATCH_SIZE * SENSOR_NUM_AXIS] = {};

/* BMX160 driver structures */
mtb_bmx160_data_t data;
mtb_bmx160_t sensor_bmx160;

static imu_sensor_data_t imu_data;

/* SPI object for data transmission */
cyhal_spi_t spi;

QueueHandle_t qSensorData;

static sensor_data_t sensorData;
static uint8_t* m_pSensorData = NULL;
/*******************************************************************************
* Local Functions
*******************************************************************************/
void sensor_interrupt_handler(void *callback_arg, cyhal_timer_event_t event);
cy_rslt_t sensor_timer_init(void);
static void convert_int16_to_float(void);
static int get_signal_data(size_t offset, size_t length, float *out_ptr);
/*******************************************************************************
* Function Name: sensor_init
********************************************************************************
* Summary:
*   Initialize the Neural Network based on the magic wand model. Initializes the
*   IMU to collect data, a SPI for IMU communication, a timer, and circular buffer.
*
* Parameters:
*     None
*
* Return:
*   The status of the initialization.
*******************************************************************************/
cy_rslt_t sensor_init(void)
{
    cy_rslt_t result;

    /* Setup the circle buffer for data storage */
    cy_fifo_init_static(&sensor_fifo, sensor_fifo_pool, sizeof(sensor_fifo_pool), sizeof(imu_sensor_data_t));

#ifdef CY_BMX_160_IMU
    /* Initialize SPI for IMU communication */
    result = cyhal_spi_init(&spi, CYBSP_SPI_MOSI, CYBSP_SPI_MISO, CYBSP_SPI_CLK, NC, NULL, 8, CYHAL_SPI_MODE_00_MSB, false);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set SPI frequency to 10MHz */
    result = cyhal_spi_set_frequency(&spi, IMU_SPI_FREQUENCY);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Initialize the chip select line */
    result = cyhal_gpio_init(CYBSP_SPI_CS, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, 1);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Initialize the IMU */
    result = mtb_bmx160_init_spi(&sensor_bmx160, &spi, CYBSP_SPI_CS);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Set the output data rate and range of the accelerometer */
    sensor_bmx160.sensor1.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
    sensor_bmx160.sensor1.accel_cfg.range = BMI160_ACCEL_RANGE_4G;

    /* Set the output data rate of the gyroscope */
    sensor_bmx160.sensor1.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;

    /* Set the sensor configuration */
    bmi160_set_sens_conf(&(sensor_bmx160.sensor1));
#endif

    /* Timer for data collection */
    sensor_timer_init();

    /* Create an event that will be set when data collection is done */
    cy_rtos_init_event(&sensor_event);

    return CY_RSLT_SUCCESS;
}

void sensor_task(void *arg)
{
    (void)arg;

    ei_impulse_result_t ei_result = { 0 };

    int total_length = 300;

    printf("Sensor Task started\r\n");

    sensor_data_t current_sensorData;

    for(;;)
    {
        /* Wait until there is 64 samples from the accelerometer and the
         * gyroscope in the circular buffer */

    	xQueueReceive(qSensorData, &current_sensorData, portMAX_DELAY);

    	m_pSensorData = current_sensorData.pData;

    	convert_int16_to_float();

        ei_c_wrapper_run_classifier(
            total_length,
            &get_signal_data,
            &ei_result,
            false);

        printf("\r\n=========================================\r\n");

        // Print to screen
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
        	printf("%s:\t\t%0.4f\r\n", ei_result.classification[ix].label, ei_result.classification[ix].value);
        }
    }
}

void sensor_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;
    static int location = 0;

    /* Read data from IMU sensor */
    cy_rslt_t result;
#ifdef CY_BMX_160_IMU
    result = mtb_bmx160_read(&sensor_bmx160, &data);
#endif
#ifdef CY_BMI_160_IMU
    result = mtb_bmi160_read(&sensor_bmi160, &data);
#endif
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    imu_data.accel.x 			= data.accel.x;
    imu_data.accel.y 			= data.accel.y;
    imu_data.accel.z 			= data.accel.z;
    imu_data.accel.timestamp  	= xTaskGetTickCountFromISR();


    imu_data.gyro.x 			= data.gyro.x;
    imu_data.gyro.y 			= data.gyro.y;
    imu_data.gyro.z 			= data.gyro.z;
    imu_data.gyro.timestamp 	= xTaskGetTickCountFromISR();

    /* Write data to the circle buffer */
    cy_fifo_write(&sensor_fifo, &imu_data, 1);

    /* Once there is enough data to feed the inference, run pre-processing */
    location++;
    if(location == (SENSOR_BATCH_SIZE * 2))
    {
        /* Reset the counter */
        location = 0;

        cy_fifo_clear(&sensor_fifo);

        sensorData.dataSize = SENSOR_FIFO_POOL_SIZE / 2;
        sensorData.pData = sensor_fifo_pool + (SENSOR_FIFO_POOL_SIZE / 2);

        xQueueSendFromISR(qSensorData, &sensorData, NULL);
    }
    else if(location == (SENSOR_BATCH_SIZE))
    {
        sensorData.dataSize = SENSOR_FIFO_POOL_SIZE / 2;
        sensorData.pData = sensor_fifo_pool;

        xQueueSendFromISR(qSensorData, &sensorData, NULL);
    }
}

cy_rslt_t sensor_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = SENSOR_TIMER_PERIOD,      /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run the timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&sensor_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&sensor_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&sensor_timer, SENSOR_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&sensor_timer, sensor_interrupt_handler, NULL);
    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&sensor_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, SENSOR_TIMER_PRIORITY, true);
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&sensor_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}

static void convert_int16_to_float(void)
{
	int i = 0;

    for (size_t ix = 0; ix < (SENSOR_BATCH_SIZE * SENSOR_NUM_AXIS); ix += SENSOR_NUM_AXIS)
    {
    	imu_sensor_data_t data;
    	memcpy(&data, m_pSensorData + (i * sizeof(imu_sensor_data_t)), sizeof(imu_sensor_data_t));

    	features[ix + 0] = (float)(data.accel.x) / 1.f;
    	features[ix + 1] = (float)(data.accel.y) / 1.f;
    	features[ix + 2] = (float)(data.accel.z) / 1.f;
    	features[ix + 3] = (float)(data.gyro.x) / 1.f;
    	features[ix + 4] = (float)(data.gyro.y) / 1.f;
    	features[ix + 5] = (float)(data.gyro.z) / 1.f;

        i++;
    }
}

static int get_signal_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}
