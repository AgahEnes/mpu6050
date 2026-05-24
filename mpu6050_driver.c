#include "mpu6050_driver.h"
#include "mpu6050_hal.h"

#define MPU6050_BUS_TIMEOUT_DEFAULT_MS            (100U)
#define MPU6050_BUS_LOCK_TIMEOUT_DEFAULT_MS       (100U)
#define MPU6050_SELF_TEST_SAMPLE_COUNT            (16U)
#define MPU6050_SELF_TEST_TRIM_LEN                (4U)   /* SELF_TEST_X..A (0x0D..0x10) */
#define MPU6050_SOFT_RESET_DELAY_MS               (50U)
#define MPU6050_STARTUP_DELAY_MS                  (50U)

#define AXIS_NUM_3D                               (3U)
#define AXIS_X                                    (0U)
#define AXIS_Y                                    (1U)
#define AXIS_Z                                    (2U)

#define MPU6050_RAW_FRAME_IDX_TEMP                (AXIS_NUM_3D)
#define MPU6050_RAW_FRAME_IDX_GYRO_BASE           (AXIS_NUM_3D + 1U)


#ifndef NULL
#define NULL                                  ((void *)0)
#endif

/* Layer 0: Pure helpers (no hardware dependency) */
static void    Mpu6050_prvMemZero(void *vpData, uint32_t u32Len);
static int16_t Mpu6050_prvParseBe16(const uint8_t *pu8Data);
static int32_t Mpu6050_prvAbs32(int32_t s32Value);
static float   Mpu6050_prvGetAccelLsbPerG(te_Mpu6050_AccelFs eFs);
static float   Mpu6050_prvGetGyroLsbPerDps(te_Mpu6050_GyroFs eFs);
static bool    Mpu6050_prvIsValidSmplrtDiv(te_Mpu6050_SmplrtDiv eSmplrtDiv);
static bool    Mpu6050_prvIsValidClockSource(te_Mpu6050_ClockSource eClockSource);
static float   Mpu6050_prvBuildSelfTestLimit(uint8_t u8Code, int32_t s32FallbackMin);

/* Layer 1: Bus / synchronization */
static te_Driver_RetCode Mpu6050_prvLock(ts_Mpu6050_Handle *psHandle);
static te_Driver_RetCode Mpu6050_prvUnlock(ts_Mpu6050_Handle *psHandle);
static te_Driver_RetCode Mpu6050_prvMarkError(ts_Mpu6050_Handle *psHandle, te_Driver_RetCode eRet);
static uint32_t          Mpu6050_prvGetBusTimeout(const ts_Mpu6050_Handle *psHandle);
static te_Driver_RetCode Mpu6050_prvReadBlock(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len);
static te_Driver_RetCode Mpu6050_prvWriteBlock(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len);

/* Layer 2: Register access */
static te_Driver_RetCode Mpu6050_prvReadRegister(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data);
static te_Driver_RetCode Mpu6050_prvWriteRegister(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data);

/* Layer 3: Device semantics (ordered by related register map) */
static te_Driver_RetCode Mpu6050_prvReadSelfTestTrim(ts_Mpu6050_Handle *psHandle, uint8_t au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN]); /* SELF_TEST_X..A (0x0D..0x10) */
static te_Driver_RetCode Mpu6050_prvRunSelfTest(ts_Mpu6050_Handle *psHandle, ts_Mpu6050_SelfTestResult *psResult);                  /* SELF_TEST_X..A (0x0D..0x10) */
static te_Driver_RetCode Mpu6050_prvSetSampleRateDiv(ts_Mpu6050_Handle *psHandle, te_Mpu6050_SmplrtDiv eSmplrtDiv);                 /* SMPLRT_DIV (0x19) */
static te_Driver_RetCode Mpu6050_prvGetSampleRateDiv(ts_Mpu6050_Handle *psHandle, te_Mpu6050_SmplrtDiv *peSmplrtDiv);               /* SMPLRT_DIV (0x19) */
static te_Driver_RetCode Mpu6050_prvSetDlpfCfg(ts_Mpu6050_Handle *psHandle, te_Mpu6050_DlpfCfg eDlpfCfg);                           /* CONFIG (0x1A) */
static te_Driver_RetCode Mpu6050_prvGetDlpfCfg(ts_Mpu6050_Handle *psHandle, te_Mpu6050_DlpfCfg *peDlpfCfg);                         /* CONFIG (0x1A) */
static te_Driver_RetCode Mpu6050_prvSetGyroFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_GyroFs eFs);                                  /* GYRO_CONFIG (0x1B) */
static te_Driver_RetCode Mpu6050_prvGetGyroFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_GyroFs *peFs);                                /* GYRO_CONFIG (0x1B) */
static te_Driver_RetCode Mpu6050_prvSetAccelFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_AccelFs eFs);                                /* ACCEL_CONFIG (0x1C) */
static te_Driver_RetCode Mpu6050_prvGetAccelFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_AccelFs *peFs);                              /* ACCEL_CONFIG (0x1C) */
static te_Driver_RetCode Mpu6050_prvSetMotThr(ts_Mpu6050_Handle *psHandle, uint8_t u8MotThr);                                       /* MOT_THR (0x1F) */
static te_Driver_RetCode Mpu6050_prvGetMotThr(ts_Mpu6050_Handle *psHandle, uint8_t *pu8MotThr);                                     /* MOT_THR (0x1F) */
static te_Driver_RetCode Mpu6050_prvSetFifoEnable(ts_Mpu6050_Handle *psHandle, uint8_t u8FifoEnable);                               /* FIFO_EN (0x23) */
static te_Driver_RetCode Mpu6050_prvGetFifoEnable(ts_Mpu6050_Handle *psHandle, uint8_t *pu8FifoEnable);                             /* FIFO_EN (0x23) */
static te_Driver_RetCode Mpu6050_prvSetIntPinCfg(ts_Mpu6050_Handle *psHandle, uint8_t u8IntPinCfg);                                 /* INT_PIN_CFG (0x37) */
static te_Driver_RetCode Mpu6050_prvGetIntPinCfg(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntPinCfg);                               /* INT_PIN_CFG (0x37) */
static te_Driver_RetCode Mpu6050_prvSetIntEnable(ts_Mpu6050_Handle *psHandle, uint8_t u8IntEnable);                                 /* INT_ENABLE (0x38) */
static te_Driver_RetCode Mpu6050_prvGetIntEnable(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntEnable);                               /* INT_ENABLE (0x38) */
static te_Driver_RetCode Mpu6050_prvGetIntStatus(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntStatus);                               /* INT_STATUS (0x3A) */
static te_Driver_RetCode Mpu6050_prvReadRawFrame(ts_Mpu6050_Handle *psHandle, int16_t as16Raw[7]);                                  /* ACCEL_XOUT_H..GYRO_ZOUT_L (0x3B..0x48) */
static te_Driver_RetCode Mpu6050_prvSignalPathReset(ts_Mpu6050_Handle *psHandle, const uint8_t *pu8ResetValue);                     /* SIGNAL_PATH_RESET (0x68) */
static te_Driver_RetCode Mpu6050_prvSetMotDetectCtrl(ts_Mpu6050_Handle *psHandle, uint8_t u8MotDetectCtrl);                         /* MOT_DETECT_CTRL (0x69) */
static te_Driver_RetCode Mpu6050_prvGetMotDetectCtrl(ts_Mpu6050_Handle *psHandle, uint8_t *pu8MotDetectCtrl);                       /* MOT_DETECT_CTRL (0x69) */
static te_Driver_RetCode Mpu6050_prvResetFifo(ts_Mpu6050_Handle *psHandle);                                                         /* USER_CTRL (0x6A) */
static te_Driver_RetCode Mpu6050_prvSoftReset(ts_Mpu6050_Handle *psHandle);                                                         /* PWR_MGMT_1 (0x6B) */
static te_Driver_RetCode Mpu6050_prvSetClockSource(ts_Mpu6050_Handle *psHandle, te_Mpu6050_ClockSource eClockSource);               /* PWR_MGMT_1 (0x6B) */
static te_Driver_RetCode Mpu6050_prvGetClockSource(ts_Mpu6050_Handle *psHandle, te_Mpu6050_ClockSource *peClockSource);             /* PWR_MGMT_1 (0x6B) */
static te_Driver_RetCode Mpu6050_prvSetSleepState(ts_Mpu6050_Handle *psHandle, bool bSleep);                                        /* PWR_MGMT_1 (0x6B) */
static te_Driver_RetCode Mpu6050_prvSetLpWakeCtrl(ts_Mpu6050_Handle *psHandle, uint8_t u8LpWakeCtrl);                               /* PWR_MGMT_2 (0x6C) */
static te_Driver_RetCode Mpu6050_prvGetLpWakeCtrl(ts_Mpu6050_Handle *psHandle, uint8_t *pu8LpWakeCtrl);                             /* PWR_MGMT_2 (0x6C) */
static te_Driver_RetCode Mpu6050_prvSetStandbyMask(ts_Mpu6050_Handle *psHandle, uint8_t u8StandbyMask);                             /* PWR_MGMT_2 (0x6C) */
static te_Driver_RetCode Mpu6050_prvGetStandbyMask(ts_Mpu6050_Handle *psHandle, uint8_t *pu8StandbyMask);                           /* PWR_MGMT_2 (0x6C) */
static te_Driver_RetCode Mpu6050_prvGetFifoCount(ts_Mpu6050_Handle *psHandle, uint16_t *pu16FifoCount);                             /* FIFO_COUNTH/L (0x72/0x73) */
static te_Driver_RetCode Mpu6050_prvReadFifo(ts_Mpu6050_Handle *psHandle, uint8_t *pu8Buffer, uint16_t u16Length);                  /* FIFO_R_W (0x74) */
static te_Driver_RetCode Mpu6050_prvCheckWhoAmI(ts_Mpu6050_Handle *psHandle, uint8_t *pu8WhoAmI);                                   /* WHO_AM_I (0x75) */


/* Fonksiyon govdeleri*/
static void Mpu6050_prvMemZero(void *vpData, uint32_t u32Len)
{
    uint32_t u32Idx;
    uint8_t *pu8Data = (uint8_t *)vpData;

    if (vpData == NULL)
    {
        return;
    }

    for (u32Idx = 0U; u32Idx < u32Len; ++u32Idx)
    {
        pu8Data[u32Idx] = 0U;
    }
}

static int16_t Mpu6050_prvParseBe16(const uint8_t *pu8Data)
{
    return (int16_t)(((int16_t)pu8Data[0] << 8) | pu8Data[1]);
}

static int32_t Mpu6050_prvAbs32(int32_t s32Value)
{
    return (s32Value < 0) ? -s32Value : s32Value;
}

static float Mpu6050_prvGetAccelLsbPerG(te_Mpu6050_AccelFs eFs)
{
    switch (eFs)
    {
    case MPU6050_ACCEL_FS_2G:
        return MPU6050_ACCEL_LSB_PER_G_2G;
    case MPU6050_ACCEL_FS_4G:
        return MPU6050_ACCEL_LSB_PER_G_4G;
    case MPU6050_ACCEL_FS_8G:
        return MPU6050_ACCEL_LSB_PER_G_8G;
    case MPU6050_ACCEL_FS_16G:
        return MPU6050_ACCEL_LSB_PER_G_16G;
    default:
        return MPU6050_ACCEL_LSB_PER_G_2G;
    }
}

static float Mpu6050_prvGetGyroLsbPerDps(te_Mpu6050_GyroFs eFs)
{
    switch (eFs)
    {
    case MPU6050_GYRO_FS_250_DPS:
        return MPU6050_GYRO_LSB_PER_DPS_250;
    case MPU6050_GYRO_FS_500_DPS:
        return MPU6050_GYRO_LSB_PER_DPS_500;
    case MPU6050_GYRO_FS_1000_DPS:
        return MPU6050_GYRO_LSB_PER_DPS_1000;
    case MPU6050_GYRO_FS_2000_DPS:
        return MPU6050_GYRO_LSB_PER_DPS_2000;
    default:
        return MPU6050_GYRO_LSB_PER_DPS_250;
    }
}

static bool Mpu6050_prvIsValidSmplrtDiv(te_Mpu6050_SmplrtDiv eSmplrtDiv)
{
    switch (eSmplrtDiv)
    {
    case MPU6050_SMPLRT_DIV_1000HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_500HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_200HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_100HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_50HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_20HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_10HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_5HZ_DLPF_ON:
    case MPU6050_SMPLRT_DIV_1000HZ_DLPF_OFF:
    case MPU6050_SMPLRT_DIV_500HZ_DLPF_OFF:
    case MPU6050_SMPLRT_DIV_200HZ_DLPF_OFF:
    case MPU6050_SMPLRT_DIV_100HZ_DLPF_OFF:
    case MPU6050_SMPLRT_DIV_50HZ_DLPF_OFF:
        return true;
    default:
        return false;
    }
}

static bool Mpu6050_prvIsValidClockSource(te_Mpu6050_ClockSource eClockSource)
{
    switch (eClockSource)
    {
    case MPU6050_CLOCK_INTERNAL:
    case MPU6050_CLOCK_PLL_X_GYRO:
    case MPU6050_CLOCK_PLL_Y_GYRO:
    case MPU6050_CLOCK_PLL_Z_GYRO:
    case MPU6050_CLOCK_PLL_EXT_32K:
    case MPU6050_CLOCK_PLL_EXT_19M:
    case MPU6050_CLOCK_STOP:
        return true;
    default:
        return false;
    }
}

static float Mpu6050_prvBuildSelfTestLimit(uint8_t u8Code, int32_t s32FallbackMin)
{
    if (u8Code == 0U)
    {
        return (float)s32FallbackMin;
    }

    return ((float)u8Code * 8.0f) + (float)s32FallbackMin;
}

static te_Driver_RetCode Mpu6050_prvLock(ts_Mpu6050_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->sLockInterface.pfnLock == NULL)
    {
        return DRIVER_OK;
    }

    return psHandle->sLockInterface.pfnLock(psHandle->u32BusLockTimeoutMs, psHandle->sLockInterface.vpCtx);
}

static te_Driver_RetCode Mpu6050_prvUnlock(ts_Mpu6050_Handle *psHandle)
{
    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    if (psHandle->sLockInterface.pfnUnlock == NULL)
    {
        return DRIVER_OK;
    }

    return psHandle->sLockInterface.pfnUnlock(psHandle->sLockInterface.vpCtx);
}

static te_Driver_RetCode Mpu6050_prvMarkError(ts_Mpu6050_Handle *psHandle, te_Driver_RetCode eRet)
{
    if ((psHandle != NULL) && (eRet != DRIVER_OK))
    {
        psHandle->eState = MPU6050_STATE_ERROR;
    }
    return eRet;
}

static uint32_t Mpu6050_prvGetBusTimeout(const ts_Mpu6050_Handle *psHandle)
{
    return (psHandle->u32BusTimeoutMs == 0U) ? MPU6050_BUS_TIMEOUT_DEFAULT_MS : psHandle->u32BusTimeoutMs;
}

static te_Driver_RetCode Mpu6050_prvReadBlock(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data, uint16_t u16Len)
{
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }
    if (psHandle->sBusInterface.pfnRead == NULL)
    {
        return DRIVER_ERR_CONFIG;
    }

    eRet = Mpu6050_prvLock(psHandle);
    if (eRet != DRIVER_OK)
    {
        return Mpu6050_prvMarkError(psHandle, eRet);
    }

    eRet = psHandle->sBusInterface.pfnRead(psHandle->u8I2cAddress,
                                           u8Reg,
                                           pu8Data,
                                           u16Len,
                                           Mpu6050_prvGetBusTimeout(psHandle),
                                           psHandle->sBusInterface.vpCtx);
    (void)Mpu6050_prvUnlock(psHandle);
    return (eRet == DRIVER_OK) ? DRIVER_OK : Mpu6050_prvMarkError(psHandle, eRet);
}

static te_Driver_RetCode Mpu6050_prvWriteBlock(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, const uint8_t *pu8Data, uint16_t u16Len)
{
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (pu8Data == NULL) || (u16Len == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }
    if (psHandle->sBusInterface.pfnWrite == NULL)
    {
        return DRIVER_ERR_CONFIG;
    }

    eRet = Mpu6050_prvLock(psHandle);
    if (eRet != DRIVER_OK)
    {
        return Mpu6050_prvMarkError(psHandle, eRet);
    }

    eRet = psHandle->sBusInterface.pfnWrite(psHandle->u8I2cAddress,
                                            u8Reg,
                                            pu8Data,
                                            u16Len,
                                            Mpu6050_prvGetBusTimeout(psHandle),
                                            psHandle->sBusInterface.vpCtx);
    (void)Mpu6050_prvUnlock(psHandle);
    return (eRet == DRIVER_OK) ? DRIVER_OK : Mpu6050_prvMarkError(psHandle, eRet);
}

static te_Driver_RetCode Mpu6050_prvReadRegister(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t *pu8Data)
{
    return Mpu6050_prvReadBlock(psHandle, u8Reg, pu8Data, 1U);
}

static te_Driver_RetCode Mpu6050_prvWriteRegister(ts_Mpu6050_Handle *psHandle, uint8_t u8Reg, uint8_t u8Data)
{
    return Mpu6050_prvWriteBlock(psHandle, u8Reg, &u8Data, 1U);
}

static te_Driver_RetCode Mpu6050_prvReadSelfTestTrim(ts_Mpu6050_Handle *psHandle, uint8_t au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN])
{
    if ((psHandle == NULL) || (au8SelfTest == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    return Mpu6050_prvReadBlock(psHandle, MPU6050_REG_SELF_TEST_X, au8SelfTest, MPU6050_SELF_TEST_TRIM_LEN);
}

static te_Driver_RetCode Mpu6050_prvRunSelfTest(ts_Mpu6050_Handle *psHandle, ts_Mpu6050_SelfTestResult *psResult)
{
    tu_Mpu6050_RegGyroConfig xGyroConfig;
    tu_Mpu6050_RegAccelConfig xAccelConfig;
    tu_Mpu6050_RegGyroConfig xGyroBackup;
    tu_Mpu6050_RegAccelConfig xAccelBackup;
    int32_t as32Baseline[6] = {0};
    int32_t as32Stimulated[6] = {0};
    int16_t as16Raw[7];
    uint8_t au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN] = {0};
    uint8_t au8AccelCode[AXIS_NUM_3D] = {0};
    uint8_t au8GyroCode[AXIS_NUM_3D] = {0};
    float af32AccelMinDelta[AXIS_NUM_3D];
    float af32GyroMinDelta[AXIS_NUM_3D];
    uint32_t u32Idx;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (psResult == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    Mpu6050_prvMemZero(psResult, (uint32_t)sizeof(*psResult));
    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_GYRO_CONFIG, &xGyroBackup.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, &xAccelBackup.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvReadSelfTestTrim(psHandle, au8SelfTest);
    if (eRet == DRIVER_OK)
    {
        au8GyroCode[AXIS_X] = au8SelfTest[AXIS_X] & 0x1FU;
        au8GyroCode[AXIS_Y] = au8SelfTest[AXIS_Y] & 0x1FU;
        au8GyroCode[AXIS_Z] = au8SelfTest[AXIS_Z] & 0x1FU;
        au8AccelCode[AXIS_X] = (uint8_t)(((au8SelfTest[AXIS_X] >> 5U) & 0x07U) |
                                          ((au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN - 1U] >> 4U) & 0x03U));
        au8AccelCode[AXIS_Y] = (uint8_t)(((au8SelfTest[AXIS_Y] >> 5U) & 0x07U) |
                                          ((au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN - 1U] >> 2U) & 0x03U));
        au8AccelCode[AXIS_Z] = (uint8_t)(((au8SelfTest[AXIS_Z] >> 5U) & 0x07U) |
                                          (au8SelfTest[MPU6050_SELF_TEST_TRIM_LEN - 1U] & 0x03U));
    }
    else
    {
        for (u32Idx = 0U; u32Idx < AXIS_NUM_3D; ++u32Idx)
        {
            au8AccelCode[u32Idx] = 0U;
            au8GyroCode[u32Idx] = 0U;
        }
    }

    for (u32Idx = 0U; u32Idx < AXIS_NUM_3D; ++u32Idx)
    {
        af32AccelMinDelta[u32Idx] = Mpu6050_prvBuildSelfTestLimit(au8AccelCode[u32Idx], MPU6050_SELF_TEST_ACCEL_MIN_DELTA);
        af32GyroMinDelta[u32Idx] = Mpu6050_prvBuildSelfTestLimit(au8GyroCode[u32Idx], MPU6050_SELF_TEST_GYRO_MIN_DELTA);
    }

    for (u32Idx = 0U; u32Idx < MPU6050_SELF_TEST_SAMPLE_COUNT; ++u32Idx)
    {
        eRet = Mpu6050_prvReadRawFrame(psHandle, as16Raw);
        if (eRet != DRIVER_OK)
        {
            return eRet;
        }
        as32Baseline[AXIS_X] += as16Raw[AXIS_X];
        as32Baseline[AXIS_Y] += as16Raw[AXIS_Y];
        as32Baseline[AXIS_Z] += as16Raw[AXIS_Z];
        as32Baseline[AXIS_X + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_X];
        as32Baseline[AXIS_Y + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Y];
        as32Baseline[AXIS_Z + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Z];
    }

    xGyroConfig = xGyroBackup;
    xGyroConfig.sBits.u8FsSel = MPU6050_GYRO_FS_250_DPS;
    xGyroConfig.sBits.u8XgSt = 1U;
    xGyroConfig.sBits.u8YgSt = 1U;
    xGyroConfig.sBits.u8ZgSt = 1U;
    eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_GYRO_CONFIG, xGyroConfig.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    xAccelConfig = xAccelBackup;
    xAccelConfig.sBits.u8AfsSel = MPU6050_ACCEL_FS_8G;
    xAccelConfig.sBits.u8XaSt = 1U;
    xAccelConfig.sBits.u8YaSt = 1U;
    xAccelConfig.sBits.u8ZaSt = 1U;
    eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, xAccelConfig.u8Value);
    if (eRet != DRIVER_OK)
    {
        (void)Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_GYRO_CONFIG, xGyroBackup.u8Value);
        return eRet;
    }

    if (psHandle->sTimingInterface.pfnDelayMs != NULL)
    {
        (void)psHandle->sTimingInterface.pfnDelayMs(20U, psHandle->sTimingInterface.vpCtx);
    }

    for (u32Idx = 0U; u32Idx < MPU6050_SELF_TEST_SAMPLE_COUNT; ++u32Idx)
    {
        eRet = Mpu6050_prvReadRawFrame(psHandle, as16Raw);
        if (eRet != DRIVER_OK)
        {
            (void)Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_GYRO_CONFIG, xGyroBackup.u8Value);
            (void)Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, xAccelBackup.u8Value);
            return eRet;
        }
        as32Stimulated[AXIS_X] += as16Raw[AXIS_X];
        as32Stimulated[AXIS_Y] += as16Raw[AXIS_Y];
        as32Stimulated[AXIS_Z] += as16Raw[AXIS_Z];
        as32Stimulated[AXIS_X + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_X];
        as32Stimulated[AXIS_Y + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Y];
        as32Stimulated[AXIS_Z + AXIS_NUM_3D] += as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Z];
    }

    (void)Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_GYRO_CONFIG, xGyroBackup.u8Value);
    (void)Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, xAccelBackup.u8Value);

    for (u32Idx = 0U; u32Idx < AXIS_NUM_3D; ++u32Idx)
    {
        psResult->s32AccelDelta[u32Idx] =
            (as32Stimulated[u32Idx] / (int32_t)MPU6050_SELF_TEST_SAMPLE_COUNT) -
            (as32Baseline[u32Idx] / (int32_t)MPU6050_SELF_TEST_SAMPLE_COUNT);

        psResult->s32GyroDelta[u32Idx] =
            (as32Stimulated[u32Idx + AXIS_NUM_3D] / (int32_t)MPU6050_SELF_TEST_SAMPLE_COUNT) -
            (as32Baseline[u32Idx + AXIS_NUM_3D] / (int32_t)MPU6050_SELF_TEST_SAMPLE_COUNT);
    }

    psResult->bAccelPass = ((float)Mpu6050_prvAbs32(psResult->s32AccelDelta[AXIS_X]) >= af32AccelMinDelta[AXIS_X]) &&
                           ((float)Mpu6050_prvAbs32(psResult->s32AccelDelta[AXIS_Y]) >= af32AccelMinDelta[AXIS_Y]) &&
                           ((float)Mpu6050_prvAbs32(psResult->s32AccelDelta[AXIS_Z]) >= af32AccelMinDelta[AXIS_Z]);
    psResult->bGyroPass = ((float)Mpu6050_prvAbs32(psResult->s32GyroDelta[AXIS_X]) >= af32GyroMinDelta[AXIS_X]) &&
                          ((float)Mpu6050_prvAbs32(psResult->s32GyroDelta[AXIS_Y]) >= af32GyroMinDelta[AXIS_Y]) &&
                          ((float)Mpu6050_prvAbs32(psResult->s32GyroDelta[AXIS_Z]) >= af32GyroMinDelta[AXIS_Z]);

    return DRIVER_OK;
}

static te_Driver_RetCode Mpu6050_prvSetSampleRateDiv(ts_Mpu6050_Handle *psHandle, te_Mpu6050_SmplrtDiv eSmplrtDiv)
{
    if (Mpu6050_prvIsValidSmplrtDiv(eSmplrtDiv) == false)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_SMPLRT_DIV, (uint8_t)eSmplrtDiv);
}

static te_Driver_RetCode Mpu6050_prvGetSampleRateDiv(ts_Mpu6050_Handle *psHandle, te_Mpu6050_SmplrtDiv *peSmplrtDiv)
{
    uint8_t u8SmplrtDiv;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (peSmplrtDiv == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_SMPLRT_DIV, &u8SmplrtDiv);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    if (Mpu6050_prvIsValidSmplrtDiv((te_Mpu6050_SmplrtDiv)u8SmplrtDiv) == false)
    {
        return DRIVER_ERR_IO;
    }

    *peSmplrtDiv = (te_Mpu6050_SmplrtDiv)u8SmplrtDiv;
    return DRIVER_OK;
}

static te_Driver_RetCode Mpu6050_prvSetDlpfCfg(ts_Mpu6050_Handle *psHandle, te_Mpu6050_DlpfCfg eDlpfCfg)
{
    tu_Mpu6050_RegConfig xConfig;
    te_Driver_RetCode eRet;

    if ((uint8_t)eDlpfCfg > (uint8_t)MPU6050_DLPF_CFG_5HZ)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_CONFIG, &xConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        xConfig.sBits.u8DlpfCfg = (uint8_t)eDlpfCfg;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_CONFIG, xConfig.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetDlpfCfg(ts_Mpu6050_Handle *psHandle, te_Mpu6050_DlpfCfg *peDlpfCfg)
{
    tu_Mpu6050_RegConfig xConfig;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_CONFIG, &xConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        if (xConfig.sBits.u8DlpfCfg > (uint8_t)MPU6050_DLPF_CFG_5HZ)
        {
            eRet = DRIVER_ERR_IO;
        }
        else
        {
            *peDlpfCfg = (te_Mpu6050_DlpfCfg)xConfig.sBits.u8DlpfCfg;
        }
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetGyroFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_GyroFs eFs)
{
    tu_Mpu6050_RegGyroConfig xGyroConfig;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_GYRO_CONFIG, &xGyroConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        xGyroConfig.sBits.u8FsSel = (uint8_t)eFs;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_GYRO_CONFIG, xGyroConfig.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetGyroFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_GyroFs *peFs)
{
    tu_Mpu6050_RegGyroConfig xGyroConfig;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_GYRO_CONFIG, &xGyroConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        *peFs = (te_Mpu6050_GyroFs)xGyroConfig.sBits.u8FsSel;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetAccelFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_AccelFs eFs)
{
    tu_Mpu6050_RegAccelConfig xAccelConfig;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, &xAccelConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        xAccelConfig.sBits.u8AfsSel = (uint8_t)eFs;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, xAccelConfig.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetAccelFs(ts_Mpu6050_Handle *psHandle, te_Mpu6050_AccelFs *peFs)
{
    tu_Mpu6050_RegAccelConfig xAccelConfig;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, &xAccelConfig.u8Value);
    if (eRet == DRIVER_OK)
    {
        *peFs = (te_Mpu6050_AccelFs)xAccelConfig.sBits.u8AfsSel;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetMotThr(ts_Mpu6050_Handle *psHandle, uint8_t u8MotThr)
{
    if ((u8MotThr < MPU6050_MOT_THR_MIN) || (u8MotThr > MPU6050_MOT_THR_MAX))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_MOT_THR, u8MotThr);
}

static te_Driver_RetCode Mpu6050_prvGetMotThr(ts_Mpu6050_Handle *psHandle, uint8_t *pu8MotThr)
{
    if ((psHandle == NULL) || (pu8MotThr == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    return Mpu6050_prvReadRegister(psHandle, MPU6050_REG_MOT_THR, pu8MotThr);
}

static te_Driver_RetCode Mpu6050_prvSetFifoEnable(ts_Mpu6050_Handle *psHandle, uint8_t u8FifoEnable)
{
    tu_Mpu6050_RegFifoEn xFifoEnable;

    xFifoEnable.u8Value = u8FifoEnable;
    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_FIFO_EN, xFifoEnable.u8Value);
}

static te_Driver_RetCode Mpu6050_prvGetFifoEnable(ts_Mpu6050_Handle *psHandle, uint8_t *pu8FifoEnable)
{
    tu_Mpu6050_RegFifoEn xFifoEnable;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_FIFO_EN, &xFifoEnable.u8Value);
    if (eRet == DRIVER_OK)
    {
        *pu8FifoEnable = xFifoEnable.u8Value;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetIntPinCfg(ts_Mpu6050_Handle *psHandle, uint8_t u8IntPinCfg)
{
    tu_Mpu6050_RegIntPinCfg xIntPinCfg;

    xIntPinCfg.u8Value = u8IntPinCfg;
    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_INT_PIN_CFG, xIntPinCfg.u8Value);
}

static te_Driver_RetCode Mpu6050_prvGetIntPinCfg(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntPinCfg)
{
    tu_Mpu6050_RegIntPinCfg xIntPinCfg;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_INT_PIN_CFG, &xIntPinCfg.u8Value);
    if (eRet == DRIVER_OK)
    {
        *pu8IntPinCfg = xIntPinCfg.u8Value;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetIntEnable(ts_Mpu6050_Handle *psHandle, uint8_t u8IntEnable)
{
    tu_Mpu6050_RegIntEnable xIntEnable;

    xIntEnable.u8Value = u8IntEnable;
    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_INT_ENABLE, xIntEnable.u8Value);
}

static te_Driver_RetCode Mpu6050_prvGetIntEnable(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntEnable)
{
    tu_Mpu6050_RegIntEnable xIntEnable;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_INT_ENABLE, &xIntEnable.u8Value);
    if (eRet == DRIVER_OK)
    {
        *pu8IntEnable = xIntEnable.u8Value;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetIntStatus(ts_Mpu6050_Handle *psHandle, uint8_t *pu8IntStatus)
{
    tu_Mpu6050_RegIntStatus xIntStatus;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_INT_STATUS, &xIntStatus.u8Value);
    if (eRet == DRIVER_OK)
    {
        *pu8IntStatus = xIntStatus.u8Value;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvReadRawFrame(ts_Mpu6050_Handle *psHandle, int16_t as16Raw[7])
{
    uint8_t au8Frame[MPU6050_REG_BURST_DATA_LEN];
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (as16Raw == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_prvReadBlock(psHandle, MPU6050_REG_ACCEL_XOUT_H, au8Frame, MPU6050_REG_BURST_DATA_LEN);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    as16Raw[AXIS_X] = Mpu6050_prvParseBe16(&au8Frame[0]);
    as16Raw[AXIS_Y] = Mpu6050_prvParseBe16(&au8Frame[2]);
    as16Raw[AXIS_Z] = Mpu6050_prvParseBe16(&au8Frame[4]);
    as16Raw[MPU6050_RAW_FRAME_IDX_TEMP] = Mpu6050_prvParseBe16(&au8Frame[6]);
    as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_X] = Mpu6050_prvParseBe16(&au8Frame[8]);
    as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Y] = Mpu6050_prvParseBe16(&au8Frame[10]);
    as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Z] = Mpu6050_prvParseBe16(&au8Frame[12]);
    return DRIVER_OK;
}

static te_Driver_RetCode Mpu6050_prvSignalPathReset(ts_Mpu6050_Handle *psHandle, const uint8_t *pu8ResetValue)
{
    tu_Mpu6050_RegSignalPathReset xReset;

    if (pu8ResetValue != NULL)
    {
        xReset.u8Value = *pu8ResetValue;
    }
    else
    {
        xReset.u8Value = 0U;
        xReset.sBits.u8TempReset = 1U;
        xReset.sBits.u8AccelReset = 1U;
        xReset.sBits.u8GyroReset = 1U;
    }

    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_SIGNAL_PATH_RESET, xReset.u8Value);
}

static te_Driver_RetCode Mpu6050_prvSetMotDetectCtrl(ts_Mpu6050_Handle *psHandle, uint8_t u8MotDetectCtrl)
{
    tu_Mpu6050_RegMotDetectCtrl xMotDetectCtrl;

    xMotDetectCtrl.u8Value = u8MotDetectCtrl;
    if ((xMotDetectCtrl.sBits.u8AccelOnDelay > MPU6050_MOT_ACCEL_ON_DELAY_3MS) ||
        (xMotDetectCtrl.sBits.u8MotCount > MPU6050_MOT_COUNT_4))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_MOT_DETECT_CTRL, xMotDetectCtrl.u8Value);
}

static te_Driver_RetCode Mpu6050_prvGetMotDetectCtrl(ts_Mpu6050_Handle *psHandle, uint8_t *pu8MotDetectCtrl)
{
    if ((psHandle == NULL) || (pu8MotDetectCtrl == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    return Mpu6050_prvReadRegister(psHandle, MPU6050_REG_MOT_DETECT_CTRL, pu8MotDetectCtrl);
}

static te_Driver_RetCode Mpu6050_prvResetFifo(ts_Mpu6050_Handle *psHandle)
{
    tu_Mpu6050_RegUserCtrl xUserCtrl;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_USER_CTRL, &xUserCtrl.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    xUserCtrl.sBits.u8FifoEn = 1U;
    xUserCtrl.sBits.u8FifoReset = 1U;
    return Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_USER_CTRL, xUserCtrl.u8Value);
}

static te_Driver_RetCode Mpu6050_prvSoftReset(ts_Mpu6050_Handle *psHandle)
{
    tu_Mpu6050_RegPwrMgmt1 xPwrMgmt1;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_1, &xPwrMgmt1.u8Value);
    if (eRet == DRIVER_OK)
    {
        xPwrMgmt1.sBits.u8DeviceReset = 1U;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_PWR_MGMT_1, xPwrMgmt1.u8Value);
    }
    if ((eRet == DRIVER_OK) && (psHandle->sTimingInterface.pfnDelayMs != NULL))
    {
        (void)psHandle->sTimingInterface.pfnDelayMs(MPU6050_SOFT_RESET_DELAY_MS, psHandle->sTimingInterface.vpCtx);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetClockSource(ts_Mpu6050_Handle *psHandle, te_Mpu6050_ClockSource eClockSource)
{
    tu_Mpu6050_RegPwrMgmt1 xPwrMgmt1;
    te_Driver_RetCode eRet;

    if (Mpu6050_prvIsValidClockSource(eClockSource) == false)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_1, &xPwrMgmt1.u8Value);
    if (eRet == DRIVER_OK)
    {
        xPwrMgmt1.sBits.u8ClkSel = (uint8_t)eClockSource;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_PWR_MGMT_1, xPwrMgmt1.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetClockSource(ts_Mpu6050_Handle *psHandle, te_Mpu6050_ClockSource *peClockSource)
{
    tu_Mpu6050_RegPwrMgmt1 xPwrMgmt1;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_1, &xPwrMgmt1.u8Value);
    if (eRet == DRIVER_OK)
    {
        *peClockSource = (te_Mpu6050_ClockSource)xPwrMgmt1.sBits.u8ClkSel;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetSleepState(ts_Mpu6050_Handle *psHandle, bool bSleep)
{
    tu_Mpu6050_RegPwrMgmt1 xPwrMgmt1;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_1, &xPwrMgmt1.u8Value);
    if (eRet == DRIVER_OK)
    {
        xPwrMgmt1.sBits.u8Sleep = bSleep ? 1U : 0U;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_PWR_MGMT_1, xPwrMgmt1.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetLpWakeCtrl(ts_Mpu6050_Handle *psHandle, uint8_t u8LpWakeCtrl)
{
    tu_Mpu6050_RegPwrMgmt2 xPwrMgmt2;
    te_Driver_RetCode eRet;

    if (u8LpWakeCtrl > MPU6050_LP_WAKE_CTRL_40HZ)
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_2, &xPwrMgmt2.u8Value);
    if (eRet == DRIVER_OK)
    {
        xPwrMgmt2.sBits.u8LpWakeCtrl = u8LpWakeCtrl;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_PWR_MGMT_2, xPwrMgmt2.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetLpWakeCtrl(ts_Mpu6050_Handle *psHandle, uint8_t *pu8LpWakeCtrl)
{
    tu_Mpu6050_RegPwrMgmt2 xPwrMgmt2;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (pu8LpWakeCtrl == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_2, &xPwrMgmt2.u8Value);
    if (eRet == DRIVER_OK)
    {
        *pu8LpWakeCtrl = xPwrMgmt2.sBits.u8LpWakeCtrl;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvSetStandbyMask(ts_Mpu6050_Handle *psHandle, uint8_t u8StandbyMask)
{
    tu_Mpu6050_RegPwrMgmt2 xPwrMgmt2;
    tu_Mpu6050_RegPwrMgmt2 xStandbyRequest;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_2, &xPwrMgmt2.u8Value);
    if (eRet == DRIVER_OK)
    {
        xStandbyRequest.u8Value = u8StandbyMask;
        xPwrMgmt2.sBits.u8StbyXa = xStandbyRequest.sBits.u8StbyXa;
        xPwrMgmt2.sBits.u8StbyYa = xStandbyRequest.sBits.u8StbyYa;
        xPwrMgmt2.sBits.u8StbyZa = xStandbyRequest.sBits.u8StbyZa;
        xPwrMgmt2.sBits.u8StbyXg = xStandbyRequest.sBits.u8StbyXg;
        xPwrMgmt2.sBits.u8StbyYg = xStandbyRequest.sBits.u8StbyYg;
        xPwrMgmt2.sBits.u8StbyZg = xStandbyRequest.sBits.u8StbyZg;
        eRet = Mpu6050_prvWriteRegister(psHandle, MPU6050_REG_PWR_MGMT_2, xPwrMgmt2.u8Value);
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetStandbyMask(ts_Mpu6050_Handle *psHandle, uint8_t *pu8StandbyMask)
{
    tu_Mpu6050_RegPwrMgmt2 xPwrMgmt2;
    tu_Mpu6050_RegPwrMgmt2 xStandbyMask;
    te_Driver_RetCode eRet;

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_2, &xPwrMgmt2.u8Value);
    if (eRet == DRIVER_OK)
    {
        xStandbyMask.u8Value = 0U;
        xStandbyMask.sBits.u8StbyXa = xPwrMgmt2.sBits.u8StbyXa;
        xStandbyMask.sBits.u8StbyYa = xPwrMgmt2.sBits.u8StbyYa;
        xStandbyMask.sBits.u8StbyZa = xPwrMgmt2.sBits.u8StbyZa;
        xStandbyMask.sBits.u8StbyXg = xPwrMgmt2.sBits.u8StbyXg;
        xStandbyMask.sBits.u8StbyYg = xPwrMgmt2.sBits.u8StbyYg;
        xStandbyMask.sBits.u8StbyZg = xPwrMgmt2.sBits.u8StbyZg;
        *pu8StandbyMask = xStandbyMask.u8Value;
    }
    return eRet;
}

static te_Driver_RetCode Mpu6050_prvGetFifoCount(ts_Mpu6050_Handle *psHandle, uint16_t *pu16FifoCount)
{
    uint8_t au8FifoCount[2];
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (pu16FifoCount == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_FIFO_COUNTH, &au8FifoCount[0]);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_FIFO_COUNTL, &au8FifoCount[1]);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    *pu16FifoCount = (uint16_t)(((uint16_t)au8FifoCount[0] << 8) | au8FifoCount[1]);
    return DRIVER_OK;
}

static te_Driver_RetCode Mpu6050_prvReadFifo(ts_Mpu6050_Handle *psHandle, uint8_t *pu8Buffer, uint16_t u16Length)
{
    if ((psHandle == NULL) || (pu8Buffer == NULL) || (u16Length == 0U))
    {
        return DRIVER_ERR_INVALID_ARG;
    }

    return Mpu6050_prvReadBlock(psHandle, MPU6050_REG_FIFO_R_W, pu8Buffer, u16Length);
}

static te_Driver_RetCode Mpu6050_prvCheckWhoAmI(ts_Mpu6050_Handle *psHandle, uint8_t *pu8WhoAmI)
{
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (pu8WhoAmI == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_WHO_AM_I, pu8WhoAmI);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    if ((*pu8WhoAmI != MPU6050_WHO_AM_I_EXPECTED_LOW) && (*pu8WhoAmI != MPU6050_WHO_AM_I_EXPECTED_HIGH))
    {
        return Mpu6050_prvMarkError(psHandle, DRIVER_ERR_WHOAMI);
    }

    return DRIVER_OK;
}


/* Layer 4: Public API */
te_Driver_RetCode Mpu6050_Open(ts_Mpu6050_Handle *psHandle, const ts_Mpu6050_OpenConfig *psConfig)
{
    tu_Mpu6050_RegPwrMgmt1 xPwrMgmt1;
    tu_Mpu6050_RegIntPinCfg xIntPinCfg;
    tu_Mpu6050_RegMotDetectCtrl xMotDetectCtrl;
    te_Mpu6050_SmplrtDiv eSmplrtDivReadback;
    uint8_t u8MotThrReadback;
    uint8_t u8MotDetectReadback;
    uint8_t u8LpWakeReadback;
    uint8_t u8WhoAmI = 0U;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (psConfig == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }
    if ((psConfig->sBusInterface.pfnRead == NULL) || (psConfig->sBusInterface.pfnWrite == NULL))
    {
        return DRIVER_ERR_CONFIG;
    }

    Mpu6050_prvMemZero(psHandle, (uint32_t)sizeof(*psHandle));
    psHandle->eState = MPU6050_STATE_UNINIT;
    psHandle->u8I2cAddress = (psConfig->u8I2cAddress == 0U) ? MPU6050_I2C_ADDR_AD0_LOW : psConfig->u8I2cAddress;
    psHandle->u32BusTimeoutMs = (psConfig->u32BusTimeoutMs == 0U) ? MPU6050_BUS_TIMEOUT_DEFAULT_MS : psConfig->u32BusTimeoutMs;
    psHandle->u32BusLockTimeoutMs =
        (psConfig->u32BusLockTimeoutMs == 0U) ? MPU6050_BUS_LOCK_TIMEOUT_DEFAULT_MS : psConfig->u32BusLockTimeoutMs;
    psHandle->sBusInterface = psConfig->sBusInterface;
    psHandle->sLockInterface = psConfig->sLockInterface;
    psHandle->sTimingInterface = psConfig->sTimingInterface;
    psHandle->pfnInterruptPinControl = psConfig->pfnInterruptPinControl;
    psHandle->vpInterruptCtx = psConfig->vpInterruptCtx;

    if (psHandle->sTimingInterface.pfnDelayMs != NULL)
    {
        (void)psHandle->sTimingInterface.pfnDelayMs(MPU6050_STARTUP_DELAY_MS, psHandle->sTimingInterface.vpCtx);
    }

    eRet = Mpu6050_prvCheckWhoAmI(psHandle, &u8WhoAmI);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSoftReset(psHandle);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_PWR_MGMT_1, &xPwrMgmt1.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    if ((xPwrMgmt1.u8Value != MPU6050_REG_PWR_MGMT_1_DEFAULT) && (xPwrMgmt1.u8Value != MPU6050_REG_DEFAULT_ZERO))
    {
        return DRIVER_ERR_IO;
    }

    eRet = Mpu6050_prvSetClockSource(psHandle, MPU6050_CLOCK_PLL_X_GYRO);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetSleepState(psHandle, false);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetSampleRateDiv(psHandle, MPU6050_SMPLRT_DIV_100HZ_DLPF_ON);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetDlpfCfg(psHandle, MPU6050_DLPF_CFG_44HZ);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetGyroFs(psHandle, MPU6050_GYRO_FS_250_DPS);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetAccelFs(psHandle, MPU6050_ACCEL_FS_2G);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSignalPathReset(psHandle, NULL);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetMotThr(psHandle, MPU6050_MOT_THR_MIN);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    xMotDetectCtrl.u8Value = 0U;
    xMotDetectCtrl.sBits.u8AccelOnDelay = MPU6050_MOT_ACCEL_ON_DELAY_0MS;
    xMotDetectCtrl.sBits.u8MotCount = MPU6050_MOT_COUNT_1;
    eRet = Mpu6050_prvSetMotDetectCtrl(psHandle, xMotDetectCtrl.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvSetLpWakeCtrl(psHandle, MPU6050_LP_WAKE_CTRL_1_25HZ);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    xIntPinCfg.u8Value = 0U;
    xIntPinCfg.sBits.u8IntLevel = MPU6050_INT_PIN_LEVEL_ACTIVE_LOW;
    xIntPinCfg.sBits.u8IntOpen = MPU6050_INT_PIN_OPEN_PUSH_PULL;
    xIntPinCfg.sBits.u8LatchIntEn = MPU6050_INT_PIN_LATCH_PULSE_50US;
    xIntPinCfg.sBits.u8IntAnyrd2Clear = MPU6050_INT_PIN_CLEAR_ON_ANY_READ;
    eRet = Mpu6050_prvSetIntPinCfg(psHandle, xIntPinCfg.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_prvGetSampleRateDiv(psHandle, &eSmplrtDivReadback);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvGetMotThr(psHandle, &u8MotThrReadback);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvGetMotDetectCtrl(psHandle, &u8MotDetectReadback);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvGetLpWakeCtrl(psHandle, &u8LpWakeReadback);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    if ((eSmplrtDivReadback != MPU6050_SMPLRT_DIV_100HZ_DLPF_ON) ||
        (u8MotThrReadback != MPU6050_MOT_THR_MIN) ||
        (u8MotDetectReadback != xMotDetectCtrl.u8Value) ||
        (u8LpWakeReadback != MPU6050_LP_WAKE_CTRL_1_25HZ))
    {
        return DRIVER_ERR_IO;
    }

    eRet = Mpu6050_prvSetIntEnable(psHandle, 0U);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    psHandle->eState = MPU6050_STATE_READY;
    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Close(ts_Mpu6050_Handle *psHandle)
{
    te_Driver_RetCode eRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }
    if (psHandle->eState == MPU6050_STATE_UNINIT)
    {
        return DRIVER_ERR_STATE;
    }

    eRet = Mpu6050_prvSetSleepState(psHandle, true);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    psHandle->eState = MPU6050_STATE_UNINIT;
    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Read(ts_Mpu6050_Handle *psHandle, ts_Mpu6050_Data *psOutData)
{
    tu_Mpu6050_RegGyroConfig xGyroCfg;
    tu_Mpu6050_RegAccelConfig xAccelCfg;
    int16_t as16Raw[7];
    float f32AccelScale;
    float f32GyroScale;
    te_Driver_RetCode eRet;

    if ((psHandle == NULL) || (psOutData == NULL))
    {
        return DRIVER_ERR_NULL_PTR;
    }
    if (psHandle->eState != MPU6050_STATE_READY)
    {
        return DRIVER_ERR_STATE;
    }

    eRet = Mpu6050_prvReadRawFrame(psHandle, as16Raw);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_GYRO_CONFIG, &xGyroCfg.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }
    eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_ACCEL_CONFIG, &xAccelCfg.u8Value);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    f32AccelScale = MPU6050_STD_GRAVITY_MPS2 / Mpu6050_prvGetAccelLsbPerG((te_Mpu6050_AccelFs)xAccelCfg.sBits.u8AfsSel);
    f32GyroScale = MPU6050_DEG_TO_RAD / Mpu6050_prvGetGyroLsbPerDps((te_Mpu6050_GyroFs)xGyroCfg.sBits.u8FsSel);

    psOutData->sAccelMps2.f32X = ((float)as16Raw[AXIS_X] * f32AccelScale) - psHandle->sCalibration.sAccelBiasMps2.f32X;
    psOutData->sAccelMps2.f32Y = ((float)as16Raw[AXIS_Y] * f32AccelScale) - psHandle->sCalibration.sAccelBiasMps2.f32Y;
    psOutData->sAccelMps2.f32Z = ((float)as16Raw[AXIS_Z] * f32AccelScale) - psHandle->sCalibration.sAccelBiasMps2.f32Z;
    psOutData->f32TempC        = ((float)as16Raw[MPU6050_RAW_FRAME_IDX_TEMP] / MPU6050_TEMP_SENS_LSB_PER_C) + MPU6050_TEMP_OFFSET_C - psHandle->sCalibration.f32TempBiasC;
    psOutData->sGyroRadS.f32X  = ((float)as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_X] * f32GyroScale) - psHandle->sCalibration.sGyroBiasRadS.f32X;
    psOutData->sGyroRadS.f32Y  = ((float)as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Y] * f32GyroScale) - psHandle->sCalibration.sGyroBiasRadS.f32Y;
    psOutData->sGyroRadS.f32Z  = ((float)as16Raw[MPU6050_RAW_FRAME_IDX_GYRO_BASE + AXIS_Z] * f32GyroScale) - psHandle->sCalibration.sGyroBiasRadS.f32Z;
    psOutData->u32TimestampMs  = (psHandle->sTimingInterface.pfnGetTickMs != NULL) ? psHandle->sTimingInterface.pfnGetTickMs(psHandle->sTimingInterface.vpCtx) : 0U;
    psOutData->bValid          = true;
    return DRIVER_OK;
}

te_Driver_RetCode Mpu6050_Write(ts_Mpu6050_Handle *psHandle, const void *vpInData)
{
    (void)psHandle;
    (void)vpInData;
    return DRIVER_ERR_NOT_SUPPORTED;
}

te_Driver_RetCode Mpu6050_Ioctl(ts_Mpu6050_Handle *psHandle, te_Mpu6050_IoctlCmd eCmd, void *vpArg)
{
    te_Driver_RetCode eRet = DRIVER_OK;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }
    if ((psHandle->eState == MPU6050_STATE_UNINIT) && (eCmd != MPU6050_IOCTL_GET_STATE))
    {
        return DRIVER_ERR_STATE;
    }

    switch (eCmd)
    {
    case MPU6050_IOCTL_GET_VERSION:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : DRIVER_OK;
        if (eRet == DRIVER_OK)
        {
            *(uint16_t *)vpArg = MPU6050_DRIVER_API_VERSION;
        }
        break;

    case MPU6050_IOCTL_GET_STATE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : DRIVER_OK;
        if (eRet == DRIVER_OK)
        {
            *(te_Mpu6050_State *)vpArg = psHandle->eState;
        }
        break;

    case MPU6050_IOCTL_CHECK_HEALTH:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : DRIVER_OK;
        if (eRet == DRIVER_OK)
        {
            ts_Mpu6050_HealthStatus *psHealth = (ts_Mpu6050_HealthStatus *)vpArg;

            Mpu6050_prvMemZero(psHealth, (uint32_t)sizeof(*psHealth));
            psHealth->eState = psHandle->eState;
            eRet = Mpu6050_prvReadRegister(psHandle, MPU6050_REG_WHO_AM_I, &psHealth->u8WhoAmIValue);
            if (eRet == DRIVER_OK)
            {
                psHealth->bBusOk = true;
                psHealth->bWhoAmIOk = (psHealth->u8WhoAmIValue == MPU6050_WHO_AM_I_EXPECTED_LOW) ||
                                      (psHealth->u8WhoAmIValue == MPU6050_WHO_AM_I_EXPECTED_HIGH);
                eRet = psHealth->bWhoAmIOk ? DRIVER_OK : DRIVER_ERR_WHOAMI;
            }
            else
            {
                psHealth->bBusOk = false;
                psHealth->bWhoAmIOk = false;
            }
        }
        break;

    case MPU6050_IOCTL_SOFT_RESET:
        eRet = Mpu6050_prvSoftReset(psHandle);
        break;

    case MPU6050_IOCTL_SIGNAL_PATH_RESET:
        eRet = Mpu6050_prvSignalPathReset(psHandle, (const uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_SET_CLOCK_SOURCE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetClockSource(psHandle, *(te_Mpu6050_ClockSource *)vpArg);
        break;

    case MPU6050_IOCTL_GET_CLOCK_SOURCE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetClockSource(psHandle, (te_Mpu6050_ClockSource *)vpArg);
        break;

    case MPU6050_IOCTL_SET_SAMPLE_RATE_DIV:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetSampleRateDiv(psHandle, *(te_Mpu6050_SmplrtDiv *)vpArg);
        break;

    case MPU6050_IOCTL_GET_SAMPLE_RATE_DIV:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetSampleRateDiv(psHandle, (te_Mpu6050_SmplrtDiv *)vpArg);
        break;

    case MPU6050_IOCTL_SET_DLPF_CFG:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetDlpfCfg(psHandle, *(te_Mpu6050_DlpfCfg *)vpArg);
        break;

    case MPU6050_IOCTL_GET_DLPF_CFG:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetDlpfCfg(psHandle, (te_Mpu6050_DlpfCfg *)vpArg);
        break;

    case MPU6050_IOCTL_SET_GYRO_FS:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetGyroFs(psHandle, *(te_Mpu6050_GyroFs *)vpArg);
        break;

    case MPU6050_IOCTL_GET_GYRO_FS:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetGyroFs(psHandle, (te_Mpu6050_GyroFs *)vpArg);
        break;

    case MPU6050_IOCTL_SET_ACCEL_FS:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetAccelFs(psHandle, *(te_Mpu6050_AccelFs *)vpArg);
        break;

    case MPU6050_IOCTL_GET_ACCEL_FS:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetAccelFs(psHandle, (te_Mpu6050_AccelFs *)vpArg);
        break;

    case MPU6050_IOCTL_SLEEP:
        eRet = Mpu6050_prvSetSleepState(psHandle, true);
        if (eRet == DRIVER_OK)
        {
            psHandle->eState = MPU6050_STATE_SLEEP;
        }
        break;

    case MPU6050_IOCTL_WAKE:
        eRet = Mpu6050_prvSetSleepState(psHandle, false);
        if (eRet == DRIVER_OK)
        {
            psHandle->eState = MPU6050_STATE_READY;
        }
        break;

    case MPU6050_IOCTL_SET_STANDBY_MASK:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetStandbyMask(psHandle, *(uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_GET_STANDBY_MASK:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetStandbyMask(psHandle, (uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_SET_INT_PIN_CFG:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetIntPinCfg(psHandle, *(uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_GET_INT_PIN_CFG:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetIntPinCfg(psHandle, (uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_SET_INT_ENABLE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetIntEnable(psHandle, *(uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_GET_INT_ENABLE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetIntEnable(psHandle, (uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_GET_INT_STATUS:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetIntStatus(psHandle, (uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_ENABLE_INTERRUPT_PIN:
        if (psHandle->pfnInterruptPinControl == NULL)
        {
            eRet = DRIVER_ERR_NOT_SUPPORTED;
            break;
        }
        eRet = psHandle->pfnInterruptPinControl(psHandle->vpInterruptCtx, true);
        break;

    case MPU6050_IOCTL_DISABLE_INTERRUPT_PIN:
        if (psHandle->pfnInterruptPinControl == NULL)
        {
            eRet = DRIVER_ERR_NOT_SUPPORTED;
            break;
        }
        eRet = psHandle->pfnInterruptPinControl(psHandle->vpInterruptCtx, false);
        break;

    case MPU6050_IOCTL_SET_FIFO_ENABLE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvSetFifoEnable(psHandle, *(uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_GET_FIFO_ENABLE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetFifoEnable(psHandle, (uint8_t *)vpArg);
        break;

    case MPU6050_IOCTL_RESET_FIFO:
        eRet = Mpu6050_prvResetFifo(psHandle);
        break;

    case MPU6050_IOCTL_GET_FIFO_COUNT:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvGetFifoCount(psHandle, (uint16_t *)vpArg);
        break;

    case MPU6050_IOCTL_READ_FIFO:
        eRet = ((vpArg == NULL) ||
                (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) ||
                (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length == 0U)) ?
               DRIVER_ERR_INVALID_ARG :
               Mpu6050_prvReadFifo(psHandle,
                                   ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                   ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length);
        break;

    case MPU6050_IOCTL_SET_CALIBRATION:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : DRIVER_OK;
        if (eRet == DRIVER_OK)
        {
            psHandle->sCalibration = *(ts_Mpu6050_Calibration *)vpArg;
        }
        break;

    case MPU6050_IOCTL_GET_CALIBRATION:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : DRIVER_OK;
        if (eRet == DRIVER_OK)
        {
            *(ts_Mpu6050_Calibration *)vpArg = psHandle->sCalibration;
        }
        break;

    case MPU6050_IOCTL_CLEAR_CALIBRATION:
        Mpu6050_prvMemZero(&psHandle->sCalibration, (uint32_t)sizeof(psHandle->sCalibration));
        break;

    case MPU6050_IOCTL_RUN_SELF_TEST:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvRunSelfTest(psHandle, (ts_Mpu6050_SelfTestResult *)vpArg);
        break;

    case MPU6050_IOCTL_REG_READ:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvReadRegister(psHandle, ((ts_Mpu6050_RegisterAccess *)vpArg)->u8RegisterAddr, &((ts_Mpu6050_RegisterAccess *)vpArg)->u8Value);
        break;

    case MPU6050_IOCTL_REG_WRITE:
        eRet = (vpArg == NULL) ? DRIVER_ERR_NULL_PTR : Mpu6050_prvWriteRegister(psHandle,((ts_Mpu6050_RegisterAccess *)vpArg)->u8RegisterAddr,((ts_Mpu6050_RegisterAccess *)vpArg)->u8Value);
        break;

    case MPU6050_IOCTL_REG_READ_BLOCK:
        if ((vpArg == NULL) || (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) || (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length == 0U))
        {
            eRet = DRIVER_ERR_INVALID_ARG;
        }
        else
        {
            eRet = Mpu6050_prvReadBlock(psHandle,
                                        ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u8RegisterAddr,
                                        ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                        ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length);
        }
        break;

    case MPU6050_IOCTL_REG_WRITE_BLOCK:
        if ((vpArg == NULL) || (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer == NULL) || (((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length == 0U))
        {
            eRet = DRIVER_ERR_INVALID_ARG;
        }
        else
        {
            eRet = Mpu6050_prvWriteBlock(psHandle,
                                         ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u8RegisterAddr,
                                         ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->pu8Buffer,
                                         ((ts_Mpu6050_RegisterBlockAccess *)vpArg)->u16Length);
        }
        break;

    default:
        eRet = DRIVER_ERR_NOT_SUPPORTED;
        break;
    }

    if ((eRet == DRIVER_ERR_BUS) || (eRet == DRIVER_ERR_TIMEOUT) || (eRet == DRIVER_ERR_WHOAMI) || (eRet == DRIVER_ERR_IO))
    {
        return Mpu6050_prvMarkError(psHandle, eRet);
    }

    return eRet;
}

te_Driver_RetCode Mpu6050_Test(ts_Mpu6050_Handle *psHandle, uint32_t u32TimeoutMs)
{
    ts_Mpu6050_HealthStatus sHealth;
    ts_Mpu6050_SelfTestResult sSelfTest;
    te_Driver_RetCode eRet;

    if (psHandle == NULL)
    {
        return DRIVER_ERR_NULL_PTR;
    }

    eRet = Mpu6050_Ioctl(psHandle, MPU6050_IOCTL_CHECK_HEALTH, &sHealth);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    eRet = Mpu6050_Ioctl(psHandle, MPU6050_IOCTL_RUN_SELF_TEST, &sSelfTest);
    if (eRet != DRIVER_OK)
    {
        return eRet;
    }

    if ((!sSelfTest.bAccelPass) || (!sSelfTest.bGyroPass))
    {
        return Mpu6050_prvMarkError(psHandle, DRIVER_ERR_IO);
    }

    if ((u32TimeoutMs > 0U) && (psHandle->sTimingInterface.pfnDelayMs != NULL))
    {
        (void)psHandle->sTimingInterface.pfnDelayMs(u32TimeoutMs, psHandle->sTimingInterface.vpCtx);
    }

    return DRIVER_OK;
}
