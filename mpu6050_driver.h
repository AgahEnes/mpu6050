#ifndef MPU6050_DRIVER_H_
#define MPU6050_DRIVER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MPU6050_DRIVER_API_VERSION                (0x0201U)

typedef enum
{
    DRIVER_OK = 0,
    DRIVER_ERR_NULL_PTR,
    DRIVER_ERR_INVALID_ARG,
    DRIVER_ERR_STATE,
    DRIVER_ERR_BUS,
    DRIVER_ERR_TIMEOUT,
    DRIVER_ERR_CONFIG,
    DRIVER_ERR_NOT_SUPPORTED,
    DRIVER_ERR_WHOAMI,
    DRIVER_ERR_IO
} te_Driver_RetCode;

typedef enum
{
    MPU6050_STATE_UNINIT = 0,
    MPU6050_STATE_READY,
    MPU6050_STATE_SLEEP,
    MPU6050_STATE_ERROR
} te_Mpu6050_State;

typedef te_Driver_RetCode (*tpfn_Mpu6050BusRead)(uint8_t u8DeviceAddr,
                                                  uint8_t u8RegisterAddr,
                                                  uint8_t *pu8ReadData,
                                                  uint16_t u16ReadLen,
                                                  uint32_t u32TimeoutMs,
                                                  void *vpCtx);
typedef te_Driver_RetCode (*tpfn_Mpu6050BusWrite)(uint8_t u8DeviceAddr,
                                                   uint8_t u8RegisterAddr,
                                                   const uint8_t *pu8WriteData,
                                                   uint16_t u16WriteLen,
                                                   uint32_t u32TimeoutMs,
                                                   void *vpCtx);
typedef te_Driver_RetCode (*tpfn_Mpu6050DelayMs)(uint32_t u32DelayMs, void *vpCtx);
typedef uint32_t (*tpfn_Mpu6050GetTickMs)(void *vpCtx);
typedef te_Driver_RetCode (*tpfn_Mpu6050Lock)(uint32_t u32TimeoutMs, void *vpCtx);
typedef te_Driver_RetCode (*tpfn_Mpu6050Unlock)(void *vpCtx);
typedef te_Driver_RetCode (*tpfn_Mpu6050InterruptPinControl)(void *vpCtx, bool bEnable);

typedef struct
{
    tpfn_Mpu6050BusRead pfnRead;
    tpfn_Mpu6050BusWrite pfnWrite;
    void *vpCtx;
} ts_BusInterface;

typedef struct
{
    tpfn_Mpu6050Lock pfnLock;
    tpfn_Mpu6050Unlock pfnUnlock;
    void *vpCtx;
} ts_LockInterface;

typedef struct
{
    tpfn_Mpu6050DelayMs pfnDelayMs;
    tpfn_Mpu6050GetTickMs pfnGetTickMs;
    void *vpCtx;
} ts_Mpu6050_TimingInterface;

typedef struct
{
    uint8_t u8I2cAddress;
    uint32_t u32BusTimeoutMs;
    uint32_t u32BusLockTimeoutMs;
    ts_BusInterface sBusInterface;
    ts_LockInterface sLockInterface;
    ts_Mpu6050_TimingInterface sTimingInterface;
    tpfn_Mpu6050InterruptPinControl pfnInterruptPinControl;
    void *vpInterruptCtx;
} ts_Mpu6050_OpenConfig;

typedef struct
{
    float f32X;
    float f32Y;
    float f32Z;
} ts_Mpu6050_Vector3f;

typedef struct
{
    ts_Mpu6050_Vector3f sAccelMps2;
    ts_Mpu6050_Vector3f sGyroRadS;
    float f32TempC;
    uint32_t u32TimestampMs;
    bool bValid;
} ts_Mpu6050_Data;

typedef struct
{
    ts_Mpu6050_Vector3f sAccelBiasMps2;
    ts_Mpu6050_Vector3f sGyroBiasRadS;
    float f32TempBiasC;
} ts_Mpu6050_Calibration;

typedef struct
{
    bool bBusOk;
    bool bWhoAmIOk;
    uint8_t u8WhoAmIValue;
    te_Mpu6050_State eState;
} ts_Mpu6050_HealthStatus;

typedef struct
{
    int32_t s32AccelDelta[3];
    int32_t s32GyroDelta[3];
    bool bAccelPass;
    bool bGyroPass;
} ts_Mpu6050_SelfTestResult;

typedef struct
{
    uint8_t u8RegisterAddr;
    uint8_t u8Value;
} ts_Mpu6050_RegisterAccess;

typedef struct
{
    uint8_t u8RegisterAddr;
    uint8_t *pu8Buffer;
    uint16_t u16Length;
} ts_Mpu6050_RegisterBlockAccess;

typedef struct
{
    te_Mpu6050_State eState;
    uint8_t u8I2cAddress;
    uint32_t u32BusTimeoutMs;
    uint32_t u32BusLockTimeoutMs;
    ts_BusInterface sBusInterface;
    ts_LockInterface sLockInterface;
    ts_Mpu6050_TimingInterface sTimingInterface;
    ts_Mpu6050_Calibration sCalibration;
    tpfn_Mpu6050InterruptPinControl pfnInterruptPinControl;
    void *vpInterruptCtx;
} ts_Mpu6050_Handle;

typedef enum
{
    MPU6050_IOCTL_GET_VERSION = 0x00U,
    MPU6050_IOCTL_GET_STATE = 0x01U,
    MPU6050_IOCTL_CHECK_HEALTH = 0x10U,
    MPU6050_IOCTL_SOFT_RESET = 0x11U,
    MPU6050_IOCTL_SIGNAL_PATH_RESET = 0x12U,
    MPU6050_IOCTL_SET_CLOCK_SOURCE = 0x20U,
    MPU6050_IOCTL_GET_CLOCK_SOURCE = 0x21U,
    MPU6050_IOCTL_SET_SAMPLE_RATE_DIV = 0x22U,
    MPU6050_IOCTL_GET_SAMPLE_RATE_DIV = 0x23U,
    MPU6050_IOCTL_SET_DLPF_CFG = 0x24U,
    MPU6050_IOCTL_GET_DLPF_CFG = 0x25U,
    MPU6050_IOCTL_SET_GYRO_FS = 0x26U,
    MPU6050_IOCTL_GET_GYRO_FS = 0x27U,
    MPU6050_IOCTL_SET_ACCEL_FS = 0x28U,
    MPU6050_IOCTL_GET_ACCEL_FS = 0x29U,
    MPU6050_IOCTL_SLEEP = 0x2AU,
    MPU6050_IOCTL_WAKE = 0x2BU,
    MPU6050_IOCTL_SET_STANDBY_MASK = 0x2CU,
    MPU6050_IOCTL_GET_STANDBY_MASK = 0x2DU,
    MPU6050_IOCTL_SET_INT_PIN_CFG = 0x30U,
    MPU6050_IOCTL_GET_INT_PIN_CFG = 0x31U,
    MPU6050_IOCTL_SET_INT_ENABLE = 0x32U,
    MPU6050_IOCTL_GET_INT_ENABLE = 0x33U,
    MPU6050_IOCTL_GET_INT_STATUS = 0x34U,
    MPU6050_IOCTL_ENABLE_INTERRUPT_PIN = 0x35U,
    MPU6050_IOCTL_DISABLE_INTERRUPT_PIN = 0x36U,
    MPU6050_IOCTL_SET_FIFO_ENABLE = 0x40U,
    MPU6050_IOCTL_GET_FIFO_ENABLE = 0x41U,
    MPU6050_IOCTL_RESET_FIFO = 0x42U,
    MPU6050_IOCTL_GET_FIFO_COUNT = 0x43U,
    MPU6050_IOCTL_READ_FIFO = 0x44U,
    MPU6050_IOCTL_SET_CALIBRATION = 0x50U,
    MPU6050_IOCTL_GET_CALIBRATION = 0x51U,
    MPU6050_IOCTL_CLEAR_CALIBRATION = 0x52U,
    MPU6050_IOCTL_RUN_SELF_TEST = 0x60U,
    MPU6050_IOCTL_REG_READ = 0x70U,
    MPU6050_IOCTL_REG_WRITE = 0x71U,
    MPU6050_IOCTL_REG_READ_BLOCK = 0x72U,
    MPU6050_IOCTL_REG_WRITE_BLOCK = 0x73U
} te_Mpu6050_IoctlCmd;

/**
 * @brief Initializes a specific MPU6050 instance and validates WHO_AM_I.
 * @param psHandle Sensor instance handle (must not be NULL).
 * @param psConfig Open configuration with injected interfaces.
 * @return DRIVER_OK on success, otherwise te_Driver_RetCode.
 */
te_Driver_RetCode Mpu6050_Open(ts_Mpu6050_Handle *psHandle, const ts_Mpu6050_OpenConfig *psConfig);

/**
 * @brief Safely closes a specific MPU6050 instance.
 * @param psHandle Sensor instance handle.
 * @return DRIVER_OK on success, otherwise te_Driver_RetCode.
 */
te_Driver_RetCode Mpu6050_Close(ts_Mpu6050_Handle *psHandle);

/**
 * @brief Reads sensor outputs as SI scaled values.
 * @param psHandle Sensor instance handle.
 * @param psOutData Output data pointer.
 * @return DRIVER_OK on success, otherwise te_Driver_RetCode.
 */
te_Driver_RetCode Mpu6050_Read(ts_Mpu6050_Handle *psHandle, ts_Mpu6050_Data *psOutData);

/**
 * @brief Reserved write entry for API completeness.
 * @param psHandle Sensor instance handle.
 * @param vpInData Input pointer (unused for MPU6050).
 * @return DRIVER_ERR_NOT_SUPPORTED for unsupported write model.
 */
te_Driver_RetCode Mpu6050_Write(ts_Mpu6050_Handle *psHandle, const void *vpInData);

/**
 * @brief Executes command-based configuration and service operations.
 * @param psHandle Sensor instance handle.
 * @param eCmd Ioctl command.
 * @param vpArg Command argument pointer.
 * @return DRIVER_OK on success, otherwise te_Driver_RetCode.
 */
te_Driver_RetCode Mpu6050_Ioctl(ts_Mpu6050_Handle *psHandle, te_Mpu6050_IoctlCmd eCmd, void *vpArg);

/**
 * @brief Runs a basic device verification scenario.
 * @param psHandle Sensor instance handle.
 * @param u32TimeoutMs Timeout budget for the test routine.
 * @return DRIVER_OK on success, otherwise te_Driver_RetCode.
 */
te_Driver_RetCode Mpu6050_Test(ts_Mpu6050_Handle *psHandle, uint32_t u32TimeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_DRIVER_H_ */
