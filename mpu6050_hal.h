#ifndef MPU6050_HAL_H_
#define MPU6050_HAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Device identification */
#define MPU6050_WHO_AM_I_EXPECTED_LOW        (0x68U)
#define MPU6050_WHO_AM_I_EXPECTED_HIGH       (0x69U)

/* I2C addresses (7-bit) */
#define MPU6050_I2C_ADDR_AD0_LOW             (0x68U)
#define MPU6050_I2C_ADDR_AD0_HIGH            (0x69U)


/****************************************************
            Register map (RM-MPU-6000A) 
*****************************************************/
#define MPU6050_REG_SELF_TEST_X              (0x0DU)
#define MPU6050_REG_SELF_TEST_Y              (0x0EU)
#define MPU6050_REG_SELF_TEST_Z              (0x0FU)
#define MPU6050_REG_SELF_TEST_A              (0x10U)
#define MPU6050_REG_SMPLRT_DIV               (0x19U)
#define MPU6050_REG_CONFIG                   (0x1AU)
#define MPU6050_REG_GYRO_CONFIG              (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG             (0x1CU)
#define MPU6050_REG_MOT_THR                  (0x1FU)
#define MPU6050_REG_FIFO_EN                  (0x23U)
/* I2C_Ctrl Cikartilmistir */
#define MPU6050_REG_INT_PIN_CFG              (0x37U)
#define MPU6050_REG_INT_ENABLE               (0x38U)
#define MPU6050_REG_INT_STATUS               (0x3AU)
#define MPU6050_REG_ACCEL_XOUT_H             (0x3BU)
#define MPU6050_REG_ACCEL_XOUT_L             (0x3CU)
#define MPU6050_REG_ACCEL_YOUT_H             (0x3DU)
#define MPU6050_REG_ACCEL_YOUT_L             (0x3EU)
#define MPU6050_REG_ACCEL_ZOUT_H             (0x3FU)
#define MPU6050_REG_ACCEL_ZOUT_L             (0x40U)
#define MPU6050_REG_TEMP_OUT_H               (0x41U)
#define MPU6050_REG_TEMP_OUT_L               (0x42U)
#define MPU6050_REG_GYRO_XOUT_H              (0x43U)
#define MPU6050_REG_GYRO_XOUT_L              (0x44U)
#define MPU6050_REG_GYRO_YOUT_H              (0x45U)
#define MPU6050_REG_GYRO_YOUT_L              (0x46U)
#define MPU6050_REG_GYRO_ZOUT_H              (0x47U)
#define MPU6050_REG_GYRO_ZOUT_L              (0x48U)
/* EXT_SENS_DATA Cikartilmistir */
#define MPU6050_REG_SIGNAL_PATH_RESET        (0x68U)
#define MPU6050_REG_MOT_DETECT_CTRL          (0x69U)
#define MPU6050_REG_USER_CTRL                (0x6AU)
#define MPU6050_REG_PWR_MGMT_1               (0x6BU)
#define MPU6050_REG_PWR_MGMT_2               (0x6CU)
#define MPU6050_REG_FIFO_COUNTH              (0x72U)
#define MPU6050_REG_FIFO_COUNTL              (0x73U)
#define MPU6050_REG_FIFO_R_W                 (0x74U)
#define MPU6050_REG_WHO_AM_I                 (0x75U)


/****************************************************
             Register configuration values 
****************************************************/
/* Reset/default values */
#define MPU6050_REG_DEFAULT_ZERO             (0x00U)
#define MPU6050_REG_PWR_MGMT_1_DEFAULT       (0x40U)
#define MPU6050_REG_WHO_AM_I_DEFAULT         (0x68U)

/* Physical conversion constants */
#define MPU6050_STD_GRAVITY_MPS2             (9.80665f)
#define MPU6050_PI_F                         (3.14159265358979323846f)
#define MPU6050_DEG_TO_RAD                   (MPU6050_PI_F / 180.0f)

/* Self test thresholds (conservative generic limits in raw LSB delta) */
#define MPU6050_SELF_TEST_GYRO_MIN_DELTA     (10)
#define MPU6050_SELF_TEST_ACCEL_MIN_DELTA    (200)

/* Sample rate divider values for different DLPF configurations */
/* Sample_Rate = Gyro_Out / (1 + SMPLRT_DIV); Gyro_Out: 1kHz (DLPF on), 8kHz (DLPF off) */
typedef enum
{
    MPU6050_SMPLRT_DIV_1000HZ_DLPF_ON  = 0U,
    MPU6050_SMPLRT_DIV_500HZ_DLPF_ON   = 1U,
    MPU6050_SMPLRT_DIV_200HZ_DLPF_ON   = 4U,
    MPU6050_SMPLRT_DIV_100HZ_DLPF_ON   = 9U,
    MPU6050_SMPLRT_DIV_50HZ_DLPF_ON    = 19U,
    MPU6050_SMPLRT_DIV_20HZ_DLPF_ON    = 49U,
    MPU6050_SMPLRT_DIV_10HZ_DLPF_ON    = 99U,
    MPU6050_SMPLRT_DIV_5HZ_DLPF_ON     = 199U,
    MPU6050_SMPLRT_DIV_1000HZ_DLPF_OFF = 7U,
    MPU6050_SMPLRT_DIV_500HZ_DLPF_OFF  = 15U,
    MPU6050_SMPLRT_DIV_200HZ_DLPF_OFF  = 39U,
    MPU6050_SMPLRT_DIV_100HZ_DLPF_OFF  = 79U,
    MPU6050_SMPLRT_DIV_50HZ_DLPF_OFF   = 159U
} te_Mpu6050_SmplrtDiv;

/* DLPF configuration values */
typedef enum
{
    MPU6050_DLPF_CFG_260HZ = 0,
    MPU6050_DLPF_CFG_184HZ = 1,
    MPU6050_DLPF_CFG_94HZ = 2,
    MPU6050_DLPF_CFG_44HZ = 3,
    MPU6050_DLPF_CFG_21HZ = 4,
    MPU6050_DLPF_CFG_10HZ = 5,
    MPU6050_DLPF_CFG_5HZ = 6
} te_Mpu6050_DlpfCfg;

/* Gyroscope full scale values */
typedef enum
{
    MPU6050_GYRO_FS_250_DPS = 0,
    MPU6050_GYRO_FS_500_DPS,
    MPU6050_GYRO_FS_1000_DPS,
    MPU6050_GYRO_FS_2000_DPS
} te_Mpu6050_GyroFs;

/* Accelerometer full scale values */
typedef enum
{
    MPU6050_ACCEL_FS_2G = 0,
    MPU6050_ACCEL_FS_4G,
    MPU6050_ACCEL_FS_8G,
    MPU6050_ACCEL_FS_16G
} te_Mpu6050_AccelFs;

/* INT_PIN_CFG (Register 55) bit values — use with ts_Mpu6050_RegIntPinCfgBits */
#define MPU6050_INT_PIN_LEVEL_ACTIVE_HIGH        (0U)
#define MPU6050_INT_PIN_LEVEL_ACTIVE_LOW         (1U)
#define MPU6050_INT_PIN_OPEN_PUSH_PULL           (0U)
#define MPU6050_INT_PIN_OPEN_OPEN_DRAIN          (1U)
#define MPU6050_INT_PIN_LATCH_PULSE_50US         (0U)
#define MPU6050_INT_PIN_LATCH_HOLD_UNTIL_CLEAR   (1U)
#define MPU6050_INT_PIN_CLEAR_ON_STATUS_READ     (0U)
#define MPU6050_INT_PIN_CLEAR_ON_ANY_READ        (1U)

/* IMU raw frame: ACCEL_XOUT_H through GYRO_ZOUT_L (14 bytes) */
#define MPU6050_RAW_FRAME_BYTE_LEN           (14U)

/* Accelerometer sensitivity (LSB/g) */
#define MPU6050_ACCEL_LSB_PER_G_2G           (16384.0f)
#define MPU6050_ACCEL_LSB_PER_G_4G           (8192.0f)
#define MPU6050_ACCEL_LSB_PER_G_8G           (4096.0f)
#define MPU6050_ACCEL_LSB_PER_G_16G          (2048.0f)

/* Temperature conversion */
#define MPU6050_TEMP_SENS_LSB_PER_C          (340.0f)
#define MPU6050_TEMP_OFFSET_C                (36.53f)

/* Gyroscope sensitivity (LSB/(deg/s)) */
#define MPU6050_GYRO_LSB_PER_DPS_250         (131.0f)
#define MPU6050_GYRO_LSB_PER_DPS_500         (65.5f)
#define MPU6050_GYRO_LSB_PER_DPS_1000        (32.8f)
#define MPU6050_GYRO_LSB_PER_DPS_2000        (16.4f)

/* MOT_THR (Register 31) range */
#define MPU6050_MOT_THR_MIN                       (0U)
#define MPU6050_MOT_THR_MAX                       (255U)

/* MOT_DETECT_CTRL (Register 105) field values */
#define MPU6050_MOT_COUNT_1                       (0U)
#define MPU6050_MOT_COUNT_2                       (1U)
#define MPU6050_MOT_COUNT_3                       (2U)
#define MPU6050_MOT_COUNT_4                       (3U)

#define MPU6050_MOT_ACCEL_ON_DELAY_0MS            (0U)
#define MPU6050_MOT_ACCEL_ON_DELAY_1MS            (1U)
#define MPU6050_MOT_ACCEL_ON_DELAY_2MS            (2U)
#define MPU6050_MOT_ACCEL_ON_DELAY_3MS            (3U)

/* Clock source values */
typedef enum
{
    MPU6050_CLOCK_INTERNAL = 0,
    MPU6050_CLOCK_PLL_X_GYRO = 1,
    MPU6050_CLOCK_PLL_Y_GYRO = 2,
    MPU6050_CLOCK_PLL_Z_GYRO = 3,
    MPU6050_CLOCK_PLL_EXT_32K = 4,
    MPU6050_CLOCK_PLL_EXT_19M = 5,
    MPU6050_CLOCK_STOP = 7
} te_Mpu6050_ClockSource;

/* PWR_MGMT_2 (Register 108) LP_WAKE_CTRL — wake-up frequency in cycle mode
The MPU-60X0 can be put into Accelerometer Only Low Power Mode using the following steps:
(i)   Set CYCLE bit to 1
(ii)  Set SLEEP bit to O
(iii) Set TEMP_DIS bit to 1
(iv)  Set STBY_XG, STBY_YG, STBY_ZG bits to 1*/
#define MPU6050_LP_WAKE_CTRL_1_25HZ              (0U)
#define MPU6050_LP_WAKE_CTRL_5HZ                 (1U)
#define MPU6050_LP_WAKE_CTRL_20HZ                (2U)
#define MPU6050_LP_WAKE_CTRL_40HZ                (3U)


/****************************************************
                  Register bit fields 
****************************************************/

/*Register 26: CONFIG */
typedef struct
{
    uint8_t u8DlpfCfg : 3;
    uint8_t u8ExtSyncSet : 3;
    uint8_t u8Reserved0 : 2;
} ts_Mpu6050_RegConfigBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegConfigBits sBits;
} tu_Mpu6050_RegConfig;

/*Register 27: GYRO_CONFIG */
typedef struct
{
    uint8_t u8Reserved0 : 3;
    uint8_t u8FsSel : 2;
    uint8_t u8ZgSt : 1;
    uint8_t u8YgSt : 1;
    uint8_t u8XgSt : 1;
} ts_Mpu6050_RegGyroConfigBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegGyroConfigBits sBits;
} tu_Mpu6050_RegGyroConfig;

/*Register 28: ACCEL_CONFIG */
typedef struct
{
    uint8_t u8Reserved0 : 3;
    uint8_t u8AfsSel : 2;
    uint8_t u8ZaSt : 1;
    uint8_t u8YaSt : 1;
    uint8_t u8XaSt : 1;
} ts_Mpu6050_RegAccelConfigBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegAccelConfigBits sBits;
} tu_Mpu6050_RegAccelConfig;

/*Register 35: FIFO_EN */
typedef struct
{
    uint8_t u8Slv0 : 1;
    uint8_t u8Slv1 : 1;
    uint8_t u8Slv2 : 1;
    uint8_t u8Accel : 1;
    uint8_t u8Zgyro : 1;
    uint8_t u8Ygyro : 1;
    uint8_t u8Xgyro : 1;
    uint8_t u8Temp : 1;
} ts_Mpu6050_RegFifoEnBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegFifoEnBits sBits;
} tu_Mpu6050_RegFifoEn;

/*Register 55: INT_PIN_CFG */
typedef struct
{
    uint8_t u8ClkOutEn : 1;
    uint8_t u8I2cBypassEn : 1;
    uint8_t u8FsyncIntEn : 1;
    uint8_t u8FsyncIntLevel : 1;
    uint8_t u8IntAnyrd2Clear : 1;
    uint8_t u8LatchIntEn : 1;
    uint8_t u8IntOpen : 1;
    uint8_t u8IntLevel : 1;
} ts_Mpu6050_RegIntPinCfgBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegIntPinCfgBits sBits;
} tu_Mpu6050_RegIntPinCfg;


/*Register 56: INT_ENABLE */
typedef struct
{
    uint8_t u8DataReadyEn : 1;
    uint8_t u8Reserved0 : 2;
    uint8_t u8I2cMstIntEn : 1;
    uint8_t u8FifoOverflowEn : 1;
    uint8_t u8Reserved1 : 1;
    uint8_t u8MotEn : 1;
    uint8_t u8Reserved2 : 1;
} ts_Mpu6050_RegIntEnableBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegIntEnableBits sBits;
} tu_Mpu6050_RegIntEnable;

/*Register 58: INT_STATUS */
typedef struct
{
    uint8_t u8DataReadyEn : 1;
    uint8_t u8Reserved0 : 2;
    uint8_t u8I2cMstIntEn : 1;
    uint8_t u8FifoOverflowEn : 1;
    uint8_t u8Reserved1 : 1;
    uint8_t u8MotEn : 1;
    uint8_t u8Reserved2 : 1;
} ts_Mpu6050_RegIntStatusBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegIntStatusBits sBits;
} tu_Mpu6050_RegIntStatus;

/*Register 104: SIGNAL_PATH_RESET */
typedef struct
{
    uint8_t u8TempReset : 1;
    uint8_t u8AccelReset : 1;
    uint8_t u8GyroReset : 1;
    uint8_t u8Reserved0 : 5;
} ts_Mpu6050_RegSignalPathResetBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegSignalPathResetBits sBits;
} tu_Mpu6050_RegSignalPathReset;

/*Register 105: MOT_DETECT_CTRL */
typedef struct
{
    uint8_t u8AccelOnDelay : 2;
    uint8_t u8MotCount : 2;
    uint8_t u8Reserved0 : 4;
} ts_Mpu6050_RegMotDetectCtrlBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegMotDetectCtrlBits sBits;
} tu_Mpu6050_RegMotDetectCtrl;

/*Register 106: USER_CTRL */
typedef struct
{
    uint8_t u8SigCondReset : 1;
    uint8_t u8I2cMstReset : 1;
    uint8_t u8FifoReset : 1;
    uint8_t u8Reserved0 : 1;
    uint8_t u8I2cIfDis : 1;
    uint8_t u8I2cMstEn : 1;
    uint8_t u8FifoEn : 1;
    uint8_t u8Reserved1 : 1;
} ts_Mpu6050_RegUserCtrlBits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegUserCtrlBits sBits;
} tu_Mpu6050_RegUserCtrl;

/*Register 107: PWR_MGMT_1 */
typedef struct
{
    uint8_t u8ClkSel : 3;
    uint8_t u8TempDisable : 1;
    uint8_t u8Reserved0 : 1;
    uint8_t u8Cycle : 1;
    uint8_t u8Sleep : 1;
    uint8_t u8DeviceReset : 1;
} ts_Mpu6050_RegPwrMgmt1Bits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegPwrMgmt1Bits sBits;
} tu_Mpu6050_RegPwrMgmt1;

/*Register 108: PWR_MGMT_2 */
typedef struct
{
    uint8_t u8StbyZg : 1;
    uint8_t u8StbyYg : 1;
    uint8_t u8StbyXg : 1;
    uint8_t u8StbyZa : 1;
    uint8_t u8StbyYa : 1;
    uint8_t u8StbyXa : 1;
    uint8_t u8LpWakeCtrl : 2;
} ts_Mpu6050_RegPwrMgmt2Bits;
typedef union
{
    uint8_t u8Value;
    ts_Mpu6050_RegPwrMgmt2Bits sBits;
} tu_Mpu6050_RegPwrMgmt2;


#ifdef __cplusplus
}
#endif

#endif /* MPU6050_HAL_H_ */
