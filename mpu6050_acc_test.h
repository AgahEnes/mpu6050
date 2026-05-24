#ifndef MPU6050_ACC_TEST_H_
#define MPU6050_ACC_TEST_H_

#include <stdint.h>

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h" // IWYU pragma: keep  -- umbrella header pulls in I2C/UART HAL types and device config

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Parameters for `MPU6050_ACC_TEST` (course-style self-test, similar to `ADXL_ACC_TEST`).
 */
typedef struct
{
    uint32_t u32TimeoutMs;       /*!< Total test duration in milliseconds (loop uses 100 ms steps). */
    UART_HandleTypeDef *pxUart; /*!< Optional UART for text lines (e.g. USART2 VCP); NULL = silent. */
    I2C_HandleTypeDef *pxI2c;    /*!< CubeMX I2C handle bound to the MPU6050 bus. */
    osMutexId_t xBusMutex;      /*!< Optional shared bus mutex; NULL = test uses an internal mutex. */
    uint8_t u8I2cAddr7bit;      /*!< 7-bit I2C address; use 0 to default to AD0 low (0x68). */
} ts_Mpu6050_AccTestParams;

#if defined(MPU6050_ENABLE_ACC_TEST)
/**
 * @brief Initializes the MPU6050 via POSIX-style API, configures basic rates/range, then streams samples.
 * @param psX Test parameters (must not be NULL; `pxI2c` must be valid).
 * @return 0 on success, -1 on open/identification failure, -2 on configuration/read failure.
 */
int8_t MPU6050_ACC_TEST(const ts_Mpu6050_AccTestParams *psX);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_ACC_TEST_H_ */
