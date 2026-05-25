#include "mpu6050_stm32_hal_port.h"
#include "mpu6050_hal.h"

#define MPU6050_DMA_WORKER_FLAG_DATA_READY         (0x01U)
#define MPU6050_DMA_WORKER_STACK_SIZE_BYTES        (512U)

static void vMpuWorkerTask(void *vpArgument);

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

static void vMpuWorkerTask(void *vpArgument)
{
    ts_Mpu6050_Stm32BusContext *psBusContext = (ts_Mpu6050_Stm32BusContext *)vpArgument;
    ts_Mpu6050_Handle *psDriverHandle;
    uint32_t u32Flags;

    if (psBusContext == NULL)
    {
        return;
    }

    for (;;)
    {
        u32Flags = osThreadFlagsWait(MPU6050_DMA_WORKER_FLAG_DATA_READY, osFlagsWaitAny, osWaitForever);
        if ((u32Flags & osFlagsError) != 0U)
        {
            continue;
        }

        psDriverHandle = (ts_Mpu6050_Handle *)psBusContext->vpDriverHandle;
        if ((psDriverHandle == NULL) || (psBusContext->pxI2cHandle == NULL) || (psBusContext->xDmaSemId == NULL))
        {
            continue;
        }

        if (osMutexAcquire(psBusContext->xBusMutex, osWaitForever) != osOK)
        {
            continue;
        }

        if (HAL_I2C_Mem_Read_DMA(psBusContext->pxI2cHandle,
                                 (uint16_t)(psDriverHandle->u8I2cAddress << 1U),
                                 MPU6050_REG_ACCEL_XOUT_H,
                                 I2C_MEMADD_SIZE_8BIT,
                                 psBusContext->au8DmaBuffer,
                                 (uint16_t)sizeof(psBusContext->au8DmaBuffer)) == HAL_OK)
        {
            (void)osSemaphoreAcquire(psBusContext->xDmaSemId, osWaitForever);
            (void)Mpu6050_SubmitRawFrame(psDriverHandle, psBusContext->au8DmaBuffer);
        }

        (void)osMutexRelease(psBusContext->xBusMutex);
    }
}

te_Driver_RetCode Mpu6050_Stm32Hal_InitAsyncWorker(ts_Mpu6050_Stm32BusContext *psBusContext, void *vpDriverHandle)
{
    const osThreadAttr_t xWorkerAttr =
    {
        .name = "mpu_dma_wrk",
        .priority = osPriorityRealtime,
        .stack_size = MPU6050_DMA_WORKER_STACK_SIZE_BYTES
    };

    if ((psBusContext == NULL) || (psBusContext->pxI2cHandle == NULL) || (psBusContext->xBusMutex == NULL) || (vpDriverHandle == NULL))
    {
        return DRIVER_ERR_INVALID_ARG;
    }
    if ((psBusContext->xWorkerTaskHandle != NULL) || (psBusContext->xDmaSemId != NULL))
    {
        return DRIVER_ERR_STATE;
    }

    psBusContext->vpDriverHandle = vpDriverHandle;
    psBusContext->xDmaSemId = osSemaphoreNew(1U, 0U, NULL);
    if (psBusContext->xDmaSemId == NULL)
    {
        psBusContext->vpDriverHandle = NULL;
        return DRIVER_ERR_STATE;
    }

    psBusContext->xWorkerTaskHandle = osThreadNew(vMpuWorkerTask, psBusContext, &xWorkerAttr);
    if (psBusContext->xWorkerTaskHandle == NULL)
    {
        (void)osSemaphoreDelete(psBusContext->xDmaSemId);
        psBusContext->xDmaSemId = NULL;
        psBusContext->vpDriverHandle = NULL;
        return DRIVER_ERR_STATE;
    }

    return DRIVER_OK;
}

void Mpu6050_Stm32Hal_OnPinInterrupt(ts_Mpu6050_Stm32BusContext *psBusContext)
{
    if ((psBusContext == NULL) || (psBusContext->xWorkerTaskHandle == NULL))
    {
        return;
    }

    (void)osThreadFlagsSet(psBusContext->xWorkerTaskHandle, MPU6050_DMA_WORKER_FLAG_DATA_READY);
}

void Mpu6050_Stm32Hal_OnDmaComplete(I2C_HandleTypeDef *hi2c, ts_Mpu6050_Stm32BusContext *psBusContext)
{
    if ((hi2c == NULL) || (psBusContext == NULL) || (psBusContext->xDmaSemId == NULL))
    {
        return;
    }
    if (hi2c != psBusContext->pxI2cHandle)
    {
        return;
    }

    (void)osSemaphoreRelease(psBusContext->xDmaSemId);
}
