#ifndef MPU6050_STM32_HAL_PORT_H_
#define MPU6050_STM32_HAL_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050_driver.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"

typedef struct
{
    I2C_HandleTypeDef *pxI2cHandle;
    osMutexId_t xBusMutex;
    osThreadId_t xWorkerTaskHandle;
    osSemaphoreId_t xDmaSemId;
    void *vpDriverHandle;
    uint8_t au8DmaBuffer[MPU6050_RAW_FRAME_BYTE_LEN];
} ts_Mpu6050_Stm32BusContext;

/**
 * @brief STM32 HAL read callback for `ts_BusInterface`.
 * @param vpBusContext Pointer to `ts_Mpu6050_Stm32BusContext`.
 * @param u8DeviceAddr 7-bit MPU6050 I2C address.
 * @param u8RegisterAddr Register start address.
 * @param pu8ReadData Destination buffer.
 * @param u16ReadLen Number of bytes to read.
 * @param u32TimeoutMs HAL timeout in milliseconds.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_Read(uint8_t u8DeviceAddr,
                                         uint8_t u8RegisterAddr,
                                         uint8_t *pu8ReadData,
                                         uint16_t u16ReadLen,
                                         uint32_t u32TimeoutMs,
                                         void *vpBusContext);

/**
 * @brief STM32 HAL write callback for `ts_BusInterface`.
 * @param vpBusContext Pointer to `ts_Mpu6050_Stm32BusContext`.
 * @param u8DeviceAddr 7-bit MPU6050 I2C address.
 * @param u8RegisterAddr Register start address.
 * @param pu8WriteData Source buffer.
 * @param u16WriteLen Number of bytes to write.
 * @param u32TimeoutMs HAL timeout in milliseconds.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_Write(uint8_t u8DeviceAddr,
                                          uint8_t u8RegisterAddr,
                                          const uint8_t *pu8WriteData,
                                          uint16_t u16WriteLen,
                                          uint32_t u32TimeoutMs,
                                          void *vpBusContext);

/**
 * @brief Delay callback based on HAL tick.
 * @param vpBusContext Unused.
 * @param u32DelayMs Delay in milliseconds.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_DelayMs(uint32_t u32DelayMs, void *vpBusContext);

/**
 * @brief Tick callback based on HAL tick.
 * @param vpBusContext Unused.
 * @return System tick in milliseconds.
 */
uint32_t Mpu6050_Stm32Hal_GetTickMs(void *vpBusContext);

/**
 * @brief Optional bus mutex lock callback.
 * @param vpBusContext Pointer to `ts_Mpu6050_Stm32BusContext`.
 * @param u32TimeoutMs Timeout in milliseconds.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpBusContext);

/**
 * @brief Optional bus mutex unlock callback.
 * @param vpBusContext Pointer to `ts_Mpu6050_Stm32BusContext`.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_Unlock(void *vpBusContext);

/**
 * @brief Helper to populate `ts_BusInterface` with STM32 HAL callbacks.
 * @param psBusInterface Destination interface.
 * @param psBusContext Bus context that holds I2C handle and optional mutex.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_FillBusInterface(ts_BusInterface *psBusInterface,
                                                     ts_Mpu6050_Stm32BusContext *psBusContext);

/**
 * @brief Helper to populate `ts_LockInterface` with STM32 callbacks.
 * @param psLockInterface Destination interface.
 * @param psBusContext Bus context that holds optional mutex.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_FillLockInterface(ts_LockInterface *psLockInterface,
                                                      ts_Mpu6050_Stm32BusContext *psBusContext);

/**
 * @brief Helper to populate `ts_Mpu6050_TimingInterface` with HAL callbacks.
 * @param psTimingInterface Destination timing interface.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_FillTimingInterface(ts_Mpu6050_TimingInterface *psTimingInterface);

/**
 * @brief Initializes internal async worker resources for DMA read flow.
 * @param psBusContext Bus context with I2C/mutex and async storage.
 * @param vpDriverHandle Pointer to `ts_Mpu6050_Handle`.
 * @return Driver return code.
 */
te_Driver_RetCode Mpu6050_Stm32Hal_InitAsyncWorker(ts_Mpu6050_Stm32BusContext *psBusContext, void *vpDriverHandle);

/**
 * @brief EXTI ISR hook to wake internal DMA worker.
 * @param psBusContext Bus context.
 */
void Mpu6050_Stm32Hal_OnPinInterrupt(ts_Mpu6050_Stm32BusContext *psBusContext);

/**
 * @brief I2C DMA completion hook for worker synchronization.
 * @param hi2c HAL I2C handle reported by callback.
 * @param psBusContext Bus context.
 */
void Mpu6050_Stm32Hal_OnDmaComplete(I2C_HandleTypeDef *hi2c, ts_Mpu6050_Stm32BusContext *psBusContext);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_STM32_HAL_PORT_H_ */
