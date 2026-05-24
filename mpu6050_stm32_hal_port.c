#include "mpu6050_stm32_hal_port.h"

te_Driver_RetCode Mpu6050_Stm32Hal_Read(uint8_t u8DeviceAddr,
                                         uint8_t u8RegisterAddr,
                                         uint8_t *pu8ReadData,
                                         uint16_t u16ReadLen,
                                         uint32_t u32TimeoutMs,
                                         void *vpBusContext)
{
    ts_Mpu6050_Stm32BusContext *psContext = (ts_Mpu6050_Stm32BusContext *)vpBusContext;

    if ((psContext == NULL) || (psContext->pxI2cHandle == NULL) || (pu8ReadData == NULL) || (u16ReadLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (HAL_I2C_Mem_Read(psContext->pxI2cHandle,
                         (uint16_t)(u8DeviceAddr << 1U),
                         u8RegisterAddr,
                         I2C_MEMADD_SIZE_8BIT,
                         pu8ReadData,
                         u16ReadLen,
                         u32TimeoutMs) != HAL_OK)
    {
        return (psContext->pxI2cHandle->ErrorCode == HAL_I2C_ERROR_TIMEOUT) ? DRIVER_ERR_TIMEOUT : DRIVER_ERR_BUS;
    }

    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Stm32Hal_Write(uint8_t u8DeviceAddr,
                                          uint8_t u8RegisterAddr,
                                          const uint8_t *pu8WriteData,
                                          uint16_t u16WriteLen,
                                          uint32_t u32TimeoutMs,
                                          void *vpBusContext)
{
    ts_Mpu6050_Stm32BusContext *psContext = (ts_Mpu6050_Stm32BusContext *)vpBusContext;

    if ((psContext == NULL) || (psContext->pxI2cHandle == NULL) || (pu8WriteData == NULL) || (u16WriteLen == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    if (HAL_I2C_Mem_Write(psContext->pxI2cHandle,
                          (uint16_t)(u8DeviceAddr << 1U),
                          u8RegisterAddr,
                          I2C_MEMADD_SIZE_8BIT,
                          (uint8_t *)pu8WriteData,
                          u16WriteLen,
                          u32TimeoutMs) != HAL_OK)
    {
        return (psContext->pxI2cHandle->ErrorCode == HAL_I2C_ERROR_TIMEOUT) ? DRIVER_ERR_TIMEOUT : DRIVER_ERR_BUS;
    }

    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Stm32Hal_DelayMs(uint32_t u32DelayMs, void *vpBusContext)
{
    (void)vpBusContext;
    HAL_Delay(u32DelayMs);
    return DRIVER_OK;
}

uint32_t Mpu6050_Stm32Hal_GetTickMs(void *vpBusContext)
{
    (void)vpBusContext;
    return HAL_GetTick();
}

te_Driver_RetCode Mpu6050_Stm32Hal_Lock(uint32_t u32TimeoutMs, void *vpBusContext)
{
    ts_Mpu6050_Stm32BusContext *psContext = (ts_Mpu6050_Stm32BusContext *)vpBusContext;

    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }
    if (psContext->xBusMutex == NULL)
    {
        return DRIVER_OK;
    }
    if (osMutexAcquire(psContext->xBusMutex, u32TimeoutMs) != osOK)
    {
        return DRIVER_ERR_TIMEOUT;
    }

    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Stm32Hal_Unlock(void *vpBusContext)
{
    ts_Mpu6050_Stm32BusContext *psContext = (ts_Mpu6050_Stm32BusContext *)vpBusContext;

    if (psContext == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }
    if (psContext->xBusMutex == NULL)
    {
        return DRIVER_OK;
    }

    return (osMutexRelease(psContext->xBusMutex) == osOK) ? DRIVER_OK : DRIVER_ERR_STATE;
}

te_Driver_RetCode Mpu6050_Stm32Hal_FillBusInterface(ts_BusInterface *psBusInterface,
                                                     ts_Mpu6050_Stm32BusContext *psBusContext)
{
    if ((psBusInterface == NULL) || (psBusContext == NULL) || (psBusContext->pxI2cHandle == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psBusInterface->pfnRead = Mpu6050_Stm32Hal_Read;
    psBusInterface->pfnWrite = Mpu6050_Stm32Hal_Write;
    psBusInterface->vpCtx = psBusContext;
    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Stm32Hal_FillLockInterface(ts_LockInterface *psLockInterface,
                                                      ts_Mpu6050_Stm32BusContext *psBusContext)
{
    if ((psLockInterface == NULL) || (psBusContext == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psLockInterface->pfnLock = Mpu6050_Stm32Hal_Lock;
    psLockInterface->pfnUnlock = Mpu6050_Stm32Hal_Unlock;
    psLockInterface->vpCtx = psBusContext;
    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Stm32Hal_FillTimingInterface(ts_Mpu6050_TimingInterface *psTimingInterface)
{
    if (psTimingInterface == NULL)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    psTimingInterface->pfnDelayMs = Mpu6050_Stm32Hal_DelayMs;
    psTimingInterface->pfnGetTickMs = Mpu6050_Stm32Hal_GetTickMs;
    psTimingInterface->vpCtx = NULL;
    return DRIVER_OK;
}
