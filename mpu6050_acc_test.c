#if defined(MPU6050_ENABLE_ACC_TEST)

#include "mpu6050_acc_test.h"

#include "mpu6050_driver.h"
#include "mpu6050_hal.h"
#include "mpu6050_stm32_hal_port.h"

#define MPU6050_ACC_TEST_LOOP_DELAY_MS          (100U)

static osMutexId_t s_xMpu6050AccTest_BusMutex = NULL;

static uint16_t Mpu6050_AccTest_prvStrLen(const char *szText)
{
    uint16_t u16Len = 0U;

    if (szText == NULL)
    {
        return 0U;
    }

    while (szText[u16Len] != '\0')
    {
        ++u16Len;
    }

    return u16Len;
}

static void Mpu6050_AccTest_prvMemZero(void *vpData, uint32_t u32Size)
{
    uint8_t *pu8Data = (uint8_t *)vpData;
    uint32_t u32Idx;

    if (vpData == NULL)
    {
        return;
    }

    for (u32Idx = 0U; u32Idx < u32Size; ++u32Idx)
    {
        pu8Data[u32Idx] = 0U;
    }
}

static void Mpu6050_AccTest_prvSendText(UART_HandleTypeDef *pxUart, const char *szText)
{
    uint16_t u16Len;

    if ((pxUart == NULL) || (szText == NULL))
    {
        return;
    }

    u16Len = Mpu6050_AccTest_prvStrLen(szText);
    if (u16Len == 0U)
    {
        return;
    }

    (void)HAL_UART_Transmit(pxUart, (uint8_t *)szText, u16Len, 100U);
}

int8_t MPU6050_ACC_TEST(const ts_Mpu6050_AccTestParams *psX)
{
    ts_Mpu6050_Stm32BusContext sBusCtx;
    ts_BusInterface sBusIf;
    ts_LockInterface sLockIf;
    ts_Mpu6050_TimingInterface sTimingIf;
    ts_Mpu6050_OpenConfig sCfg;
    ts_Mpu6050_Handle sHandle;
    ts_Mpu6050_Data sData;
    ts_Mpu6050_HealthStatus sHealth;
    te_Driver_RetCode eRet;
    uint32_t u32ElapsedMs;
    te_Mpu6050_SmplrtDiv eSmplrtDiv;
    te_Mpu6050_DlpfCfg eDlpfCfg;
    te_Mpu6050_GyroFs eGyroFs;
    te_Mpu6050_AccelFs eAccelFs;
    const osMutexAttr_t xMutexAttr = {
        .name = "mpu6050_acc_test_bus",
        .attr_bits = osMutexPrioInherit,
    };

    if ((psX == NULL) || (psX->pxI2c == NULL))
    {
        return -1;
    }

    if (psX->xBusMutex != NULL)
    {
        sBusCtx.xBusMutex = psX->xBusMutex;
    }
    else
    {
        if (s_xMpu6050AccTest_BusMutex == NULL)
        {
            s_xMpu6050AccTest_BusMutex = osMutexNew(&xMutexAttr);
            if (s_xMpu6050AccTest_BusMutex == NULL)
            {
                Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: bus mutex create failed\r\n");
                return -2;
            }
        }
        sBusCtx.xBusMutex = s_xMpu6050AccTest_BusMutex;
    }

    sBusCtx.pxI2cHandle = psX->pxI2c;
    eRet = Mpu6050_Stm32Hal_FillBusInterface(&sBusIf, &sBusCtx);
    if (eRet != DRIVER_OK)
    {
        Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: bus interface fill failed\r\n");
        return -2;
    }

    Mpu6050_AccTest_prvMemZero(&sCfg, (uint32_t)sizeof(sCfg));
    sCfg.u8I2cAddress = (psX->u8I2cAddr7bit == 0U) ? MPU6050_I2C_ADDR_AD0_LOW : psX->u8I2cAddr7bit;
    sCfg.u32BusTimeoutMs = 100U;
    sCfg.u32BusLockTimeoutMs = 100U;
    sCfg.sBusInterface = sBusIf;
    (void)Mpu6050_Stm32Hal_FillLockInterface(&sLockIf, &sBusCtx);
    sCfg.sLockInterface = sLockIf;
    (void)Mpu6050_Stm32Hal_FillTimingInterface(&sTimingIf);
    sCfg.sTimingInterface = sTimingIf;

    eRet = Mpu6050_Open(&sHandle, &sCfg);
    if (eRet != DRIVER_OK)
    {
        Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: Open failed (WHO_AM_I / bus)\r\n");
        return -1;
    }

    Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: Open OK\r\n");

    eSmplrtDiv = MPU6050_SMPLRT_DIV_100HZ_DLPF_ON;
    eRet = Mpu6050_Ioctl(&sHandle, MPU6050_IOCTL_SET_SAMPLE_RATE_DIV, &eSmplrtDiv);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    eDlpfCfg = MPU6050_DLPF_CFG_44HZ;
    eRet = Mpu6050_Ioctl(&sHandle, MPU6050_IOCTL_SET_DLPF_CFG, &eDlpfCfg);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    eGyroFs = MPU6050_GYRO_FS_250_DPS;
    eRet = Mpu6050_Ioctl(&sHandle, MPU6050_IOCTL_SET_GYRO_FS, &eGyroFs);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    eAccelFs = MPU6050_ACCEL_FS_2G;
    eRet = Mpu6050_Ioctl(&sHandle, MPU6050_IOCTL_SET_ACCEL_FS, &eAccelFs);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    Mpu6050_AccTest_prvMemZero(&sHealth, (uint32_t)sizeof(sHealth));
    eRet = Mpu6050_Ioctl(&sHandle, MPU6050_IOCTL_CHECK_HEALTH, &sHealth);
    if (eRet != DRIVER_OK)
    {
        Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: CHECK_HEALTH failed\r\n");
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    eRet = Mpu6050_Read(&sHandle, &sData);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_Close(&sHandle);
        return -2;
    }

    u32ElapsedMs = 0U;
    while (u32ElapsedMs < psX->u32TimeoutMs)
    {
        eRet = Mpu6050_Read(&sHandle, &sData);
        if (eRet != DRIVER_OK)
        {
            Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: Read failed\r\n");
            (void)Mpu6050_Close(&sHandle);
            return -2;
        }

        Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: sample ok\r\n");

        (void)osDelay(MPU6050_ACC_TEST_LOOP_DELAY_MS);
        u32ElapsedMs += MPU6050_ACC_TEST_LOOP_DELAY_MS;
    }

    (void)Mpu6050_Close(&sHandle);
    Mpu6050_AccTest_prvSendText(psX->pxUart, "MPU6050_ACC_TEST: done\r\n");
    return 0;
}

#endif /* MPU6050_ENABLE_ACC_TEST */
