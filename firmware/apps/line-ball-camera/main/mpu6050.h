#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#ifdef __cplusplus
extern "C" {
#endif

//默认不变
#define MPU6050_I2C_PORT              I2C_NUM_0

#define MPU6050_SDA_PIN               GPIO_NUM_5
#define MPU6050_SCL_PIN               GPIO_NUM_4


/*
 *
 * AD0 = LOW -> 0x68
 * AD0 = HIGH -> 0x69
 */
#define MPU6050_I2C_ADDRESS           0x68


/*
 * MPU6050 閺€顖涘瘮 Fast Mode I2C閵?
 */
#define MPU6050_I2C_FREQ_HZ           100000U

#define MPU6050_I2C_TIMEOUT_MS        50


/*
 * 婵″倹鐏?MPU6050 濡€虫健瀹歌尙绮￠張?I2C 娑撳﹥濯洪悽鐢告▎閿?
 * 閹恒劏宕樻穱婵囧瘮 0閵?
 *
 * 鐟佹瓕濮遍悧?閺冪姳绗傞幏澶嬆侀崸妤呮付鐟曚礁顦婚柈銊ょ瑐閹峰鈧?
 */
#define MPU6050_ENABLE_INTERNAL_PULLUP 1


/* ============================================================
 * 2. MPU6050 鐎靛嫬鐡ㄩ崳?
 * ============================================================ */

#define MPU6050_REG_SMPLRT_DIV        0x19
#define MPU6050_REG_CONFIG            0x1A
#define MPU6050_REG_GYRO_CONFIG       0x1B
#define MPU6050_REG_ACCEL_CONFIG      0x1C

#define MPU6050_REG_ACCEL_XOUT_H      0x3B

#define MPU6050_REG_PWR_MGMT_1        0x6B
#define MPU6050_REG_PWR_MGMT_2        0x6C

#define MPU6050_REG_WHO_AM_I          0x75


#define MPU6050_WHO_AM_I_VALUE        0x68


/* ============================================================
 * 3. MPU6050 绾兛娆㈠銉ょ稊閸欏倹鏆?
 * ============================================================ */

/*
 * 娴ｈ法鏁?DLPF 閺冭泛鍞撮柈銊╁櫚閺嶅嘲鐔€绾偓妫版垹宸肩痪锔胯礋 1kHz閵?
 *
 * sample_rate =
 *
 *      1000 / (1 + SMPLRT_DIV)
 *
 * SMPLRT_DIV = 4
 *
 * -> 200Hz
 */
#define MPU6050_SMPLRT_DIV_VALUE      4U

#define MPU6050_SAMPLE_RATE_HZ        200U

#define MPU6050_SAMPLE_PERIOD_MS      5U

#define MPU6050_SAMPLE_DT             0.005f


/*
 * CONFIG.DLPF_CFG = 3
 *
 * 缁楊兛绔撮悧鍫モ偓鍌氭値鐏忓繗婧呴幐顖氬З閻滎垰顣ㄩ妴?
 */
#define MPU6050_DLPF_CFG              3U


/*
 * Gyroscope:
 *
 * FS_SEL = 1
 *
 * 鍗?00 deg/s
 *
 * sensitivity:
 *
 * 65.5 LSB / (deg/s)
 */
#define MPU6050_GYRO_CONFIG_VALUE     0x08U
#define MPU6050_GYRO_SCALE            65.5f


/*
 * Accelerometer:
 *
 * AFS_SEL = 1
 *
 * 鍗?g
 *
 * sensitivity:
 *
 * 8192 LSB/g
 */
#define MPU6050_ACCEL_CONFIG_VALUE    0x08U
#define MPU6050_ACCEL_SCALE           8192.0f


/* ============================================================
 * 4. 鏉烆垯娆㈠銈嗗皾閸欏倹鏆?
 * ============================================================ */

/*
 * 娑撯偓闂冩湹缍嗛柅姘剧窗
 *
 * filtered =
 *
 *      alpha * previous
 *      +
 *      (1-alpha) * new
 *
 * alpha 鐡掑﹤銇囩搾濠傞挬濠婃埊绱濇担鍡楁惙鎼存棁绉洪幈顫偓?
 */
#define MPU6050_ACCEL_LPF_ALPHA       0.80f
#define MPU6050_GYRO_LPF_ALPHA        0.65f


/*
 * Roll / Pitch 娴滄帟藟濠娿倖灏濋妴?
 *
 * gyro 閸?98%
 * accel 閸?2%
 */
#define MPU6050_COMPLEMENTARY_ALPHA   0.98f


/* ============================================================
 * 5. 閼奉亜濮╅弽鈥冲櫙閸欏倹鏆?
 * ============================================================ */

/*
 * 閸掓繂顫愰崠鏍ㄦ閼奉亜濮╅幍褑顢戝鏉戦挬閺嶁€冲櫙閿?
 *
 *      Gyro X/Y/Z -> 0 dps
 *      Accel X    -> 0 g
 *      Accel Y    -> 0 g
 *      Accel Z    -> 1 g
 *
 * 濞夈劍鍓伴敍?
 * 閼奉亜濮╅弽鈥冲櫙閺堢喖妫跨亸蹇氭簠韫囧懘銆忔穱婵囧瘮闂堟瑦顒涢妴浣芥簠娴ｆ挻鎸夐獮绛圭礉
 * 楠炴湹绗?MPU6050 閻?Z 鏉炴潙绻€妞ょ粯婀炴稉濞库偓?
 *
 * 娑撹桨绻氶幐浣稿斧閺堝鍘ょ純顔藉复閸欙絽鍚嬬€圭櫢绱濈紒褏鐢绘担璺ㄦ暏閸樼喐娼甸惃?
 * MPU6050_AUTO_GYRO_CALIBRATION 鐎瑰繋缍旀稉楦垮殰閸斻劍鐗庨崙鍡楃磻閸忕偨鈧?
 */
#define MPU6050_AUTO_GYRO_CALIBRATION     1

/*
 * 600 samples @ 200Hz
 *
 * About 3 seconds.
 */
#define MPU6050_CALIBRATION_SAMPLES       600U


/*
 * Startup / calibration robustness.
 *
 * These are configuration macros only; public API is unchanged.
 *
 * The MPU6050 gyro zero offset changes most rapidly just after power-up.
 * Waiting briefly before calibration gives a more repeatable yaw bias.
 */
#ifndef MPU6050_STARTUP_SETTLE_MS
#define MPU6050_STARTUP_SETTLE_MS               1000U
#endif

/*
 * Reject calibration if the sensor is moving too much.
 * Threshold is per gyro axis, based on raw calibration samples.
 */
#ifndef MPU6050_CALIBRATION_GYRO_STD_MAX_DPS
#define MPU6050_CALIBRATION_GYRO_STD_MAX_DPS     1.50f
#endif


#ifndef MPU6050_CALIBRATION_ACCEL_STD_MAX_G
#define MPU6050_CALIBRATION_ACCEL_STD_MAX_G      0.050f
#endif

/*
 * Raw acceleration magnitude must remain physically plausible.
 * This detects motion / bad wiring / an invalid calibration pose.
 */
#ifndef MPU6050_CALIBRATION_ACCEL_MAG_MIN_G
#define MPU6050_CALIBRATION_ACCEL_MAG_MIN_G      0.75f
#endif

#ifndef MPU6050_CALIBRATION_ACCEL_MAG_MAX_G
#define MPU6050_CALIBRATION_ACCEL_MAG_MAX_G      1.25f
#endif

/*
 * Full level calibration additionally requires Z-up and reasonably level.
 * This prevents accidentally "calibrating away" a large real tilt.
 */
#ifndef MPU6050_CALIBRATION_LEVEL_MAX_XY_G
#define MPU6050_CALIBRATION_LEVEL_MAX_XY_G       0.30f
#endif

#ifndef MPU6050_CALIBRATION_LEVEL_MIN_Z_G
#define MPU6050_CALIBRATION_LEVEL_MIN_Z_G        0.70f
#endif


/* ============================================================
 * 6. 闂堟瑦顒涘Λ鈧ù瀣棘閺?
 * ============================================================ */

/*
 * 娑撳閰遍幀鏄忣潡闁喎瀹虫担搴濈艾鏉╂瑤閲滈崐纭风礉
 * 閸氬本妞傞崝鐘烩偓鐔峰濡繝鏆遍幒銉ㄧ箮 1g閿?
 * 閸掋倖鏌囨稉娲饯濮濐潿鈧?
 */
#define MPU6050_STATIONARY_GYRO_DPS       2.0f

#define MPU6050_STATIONARY_ACCEL_TOL      0.08f


/* ============================================================
 * 7. Task 閸欏倹鏆?
 * ============================================================ */

#define MPU6050_TASK_STACK_SIZE           4096

#define MPU6050_TASK_PRIORITY             7


/* ============================================================
 * 8. Debug
 * ============================================================ */

#define MPU6050_DEBUG_DEFAULT_ENABLE      0

#define MPU6050_DEBUG_DEFAULT_PERIOD_MS   200U


/* ============================================================
 * 9. 閺佹澘顒熺敮鎼佸櫤
 * ============================================================ */

#define MPU6050_RAD_TO_DEG                57.2957795131f


/* ============================================================
 * 10. 閸樼喎顫愰弫鐗堝祦
 * ============================================================ */

typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t temperature;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} mpu6050_raw_t;


/* ============================================================
 * 11. 閻椻晝鎮婇柌蹇旀殶閹?
 * ============================================================ */

typedef struct
{
    /*
     * 閸楁洑缍呴敍?
     *
     * g
     */
    float ax;
    float ay;
    float az;


    /*
     * 閸楁洑缍呴敍?
     *
     * degree / second
     */
    float gx;
    float gy;
    float gz;


    /*
     * 閹藉嫭鐨惔?
     */
    float temperature;

} mpu6050_data_t;


/* ============================================================
 * 12. 婵寧鈧?
 * ============================================================ */

typedef struct
{
    /*
     * degree
     */
    float roll;

    float pitch;

    /*
     * 閻╃顕?yaw閵?
     *
     * 娑撳秴浠?鍗?80 闂勬劕鍩楅敍?
     * 閸欘垯浜掓潻鐐电敾閿?
     *
     * 0
     * 90
     * 180
     * 270
     * ...
     */
    float yaw;

} mpu6050_attitude_t;


/* ============================================================
 * 13. 闂嗚泛浜?
 * ============================================================ */

typedef struct
{
    /*
     * g
     */
    float ax;
    float ay;
    float az;


    /*
     * deg/s
     */
    float gx;
    float gy;
    float gz;

} mpu6050_bias_t;


/* ============================================================
 * 14. 閻樿埖鈧?
 * ============================================================ */

typedef struct
{
    bool initialized;

    bool calibrated;

    bool stationary;

    bool sampling;


    uint8_t who_am_i;


    uint32_t sample_count;

    uint32_t read_error_count;

} mpu6050_status_t;


/* ============================================================
 * 15. 娑撯偓濞嗏€崇暚閺佹潙鎻╅悡?
 *
 * 婵″倹鐏夐棁鈧憰浣告倱閺冩儼顕伴崣鏍ь樋娑擃亝鏆熼幑顕嗙礉
 * 閹恒劏宕樻担璺ㄦ暏鏉╂瑤閲滈幒銉ュ經閵?
 *
 * ============================================================ */

typedef struct
{
    mpu6050_raw_t raw;

    mpu6050_data_t data;

    mpu6050_attitude_t attitude;

    bool stationary;

    uint32_t sample_count;

    int64_t timestamp_us;

} mpu6050_snapshot_t;


/* ============================================================
 * 16. 鐎电懓顦?API
 * ============================================================ */

/**
 * @brief MPU6050 閼奉亜绻侀崚娑樼紦 I2C Bus 楠炶泛鍨垫慨瀣
 *
 * 娴ｈ法鏁ら敍?
 *
 *      MPU6050_SDA_PIN
 *      MPU6050_SCL_PIN
 *      MPU6050_I2C_PORT
 *
 * 閸掓繂顫愰崠鏍ф倵閼奉亜濮╅崚娑樼紦閸氬骸褰撮柌鍥ㄧ壉 Task閵?
 */
esp_err_t mpu6050_init(void);


/**
 * @brief 娴ｈ法鏁ゅ鑼病鐎涙ê婀惃鍕煀閻?I2C Bus
 *
 * 閹恒劏宕樻禒銉ユ倵婢舵矮閲滅拋鎯ь槵閸忓彉闊?I2C 閺冩湹濞囬悽銊ｂ偓?
 *
 * 閺堫剚膩閸фぞ绗夋导姘灩闂勩倛绻栨稉顏勵樆闁?bus閵?
 */
esp_err_t mpu6050_init_with_bus(
    i2c_master_bus_handle_t bus_handle
);


/**
 * @brief 闁插﹥鏂?MPU6050
 */
esp_err_t mpu6050_deinit(void);


/**
 * @brief 閼惧嘲褰囬崥搴″酱閺堚偓閺傛壆澧块悶鍡涘櫤閺佺増宓?
 */
void mpu6050_get_data(
    mpu6050_data_t *data
);


/**
 * @brief 閼惧嘲褰囬崥搴″酱閺堚偓閺傛澘甯慨瀣殶閹?
 */
void mpu6050_get_raw(
    mpu6050_raw_t *raw
);


/**
 * @brief 閻╁瓨甯存潻娑滎攽娑撯偓濞?I2C 閸樼喎顫愮拠璇插絿
 *
 * 娑撳秶绮℃潻鍥ф倵閸欐壆绱︾€涙ǜ鈧?
 */
esp_err_t mpu6050_read_raw_now(
    mpu6050_raw_t *raw
);


/**
 * @brief 閼惧嘲褰?Roll/Pitch/Yaw
 */
void mpu6050_get_attitude(
    mpu6050_attitude_t *attitude
);


/**
 * @brief 閼惧嘲褰囨稉鈧▎鈥崇暚閺佸濮搁幀浣告彥閻?
 */
void mpu6050_get_snapshot(
    mpu6050_snapshot_t *snapshot
);


/**
 * @brief 閼惧嘲褰囧Ο鈥虫健閻樿埖鈧?
 */
void mpu6050_get_status(
    mpu6050_status_t *status
);


/**
 * @brief 閼惧嘲褰囬梿璺轰焊
 */
void mpu6050_get_bias(
    mpu6050_bias_t *bias
);


/* ============================================================
 * 韫囶偅宓?getter
 * ============================================================ */

float mpu6050_get_roll(void);

float mpu6050_get_pitch(void);

float mpu6050_get_yaw(void);


/**
 * @brief 鐏?yaw 鏉烆剙鍩?[-180,180]
 */
float mpu6050_get_yaw_wrapped(void);


float mpu6050_get_gyro_x(void);

float mpu6050_get_gyro_y(void);

float mpu6050_get_gyro_z(void);


/* ============================================================
 * 婵寧鈧?Reset
 * ============================================================ */

/**
 * @brief 瑜版挸澧犻弬鐟版倻闁插秵鏌婄€规矮绠熸稉?yaw = 0
 */
void mpu6050_reset_yaw(void);


/**
 * @brief 闁插秵鏌婇崚婵嗩潗閸栨牗鏆ｆ稉顏勑幀浣规姢濞夈垹娅?
 *
 * Roll/Pitch 娴兼艾婀稉瀣╃闁插洦鐗遍崨銊︽埂闁插秵鏌婃禒?
 * 閸旂娀鈧喎瀹崇拋鈥崇紦缁斿鍨垫慨瀣幀浣碘偓?
 *
 * yaw = 0閵?
 */
void mpu6050_reset_attitude(void);


/* ============================================================
 * Calibration
 * ============================================================ */

/**
 * @brief 閸欘亝鐗庨崙鍡涙閾昏桨鍗庨梿璺轰焊
 *
 * 娴肩姵鍔呴崳銊ョ箑妞ゅ娼ゅ顫偓?
 */
esp_err_t mpu6050_calibrate_gyro(
    uint16_t samples
);


/**
 * @brief 濮樻潙閽╅弽鈥冲櫙
 *
 * 閸嬪洩顔曢敍?
 *
 *      娴肩姵鍔呴崳銊ょ箽閹镐線娼ゅ?
 *      PCB 濮樻潙閽?
 *      Z 鏉炴潙鎮滄稉?
 *
 * 閺嶁€冲櫙閿?
 *
 *      Gyro XYZ -> 0
 *
 *      Accel X -> 0g
 *      Accel Y -> 0g
 *      Accel Z -> 1g
 */
esp_err_t mpu6050_calibrate_level(
    uint16_t samples
);


/* ============================================================
 * 鏉堝懎濮崝鐔诲厴
 * ============================================================ */

bool mpu6050_is_stationary(void);


/**
 * @brief 閺嗗倸浠犻崥搴″酱闁插洦鐗?
 */
void mpu6050_pause(void);


/**
 * @brief 閹垹顦查崥搴″酱闁插洦鐗?
 */
void mpu6050_resume(void);


/* ============================================================
 * Debug
 * ============================================================ */

/**
 * @brief 瀵偓閸忓啿鎮楅崣?IMU Debug 鏉堟挸鍤?
 *
 * period_ms 娑撳秴缂撶拋顔肩毈娴?50ms閵?
 */
void mpu6050_set_debug(
    bool enable,
    uint32_t period_ms
);


/**
 * @brief 缁斿宓嗛幍鎾冲祪娑撯偓濞嗏€崇秼閸?IMU 閻樿埖鈧?
 */
void mpu6050_print_now(void);


#ifdef __cplusplus
}
#endif



/* ============================================================
 *
 * IMPLEMENTATION
 *
 * 閸滃奔缍橀惃?motor_control.h / line_tracker.h 娑撯偓閺嶅嚖绱?
 *
 * 閺佺繝閲滃銉р柤閸欘亣鍏橀崷銊ょ娑?.c 閺傚洣娆㈢€规矮绠熼敍?
 *
 *      #define MPU6050_IMPLEMENTATION
 *      #include "mpu6050.h"
 *
 * 閸忔湹绮?.c 閺傚洣娆㈤敍?
 *
 *      #include "mpu6050.h"
 *
 * ============================================================ */


#ifdef MPU6050_IMPLEMENTATION

#include "freertos/semphr.h"

/* ============================================================
 * 17. 閸愬懘鍎寸敮鎼佸櫤
 * ============================================================ */

#define MPU6050_SAMPLE_PERIOD_US \
    ((uint64_t)MPU6050_SAMPLE_PERIOD_MS * 1000ULL)

#define MPU6050_ALT_I2C_ADDRESS \
    ((MPU6050_I2C_ADDRESS == 0x68U) ? 0x69U : 0x68U)

#define MPU6050_DEINIT_WAIT_MS          1000U
#define MPU6050_TIMER_QUIESCE_TICKS    20U


/* ============================================================
 * 18. 閸愬懘鍎撮悩鑸碘偓?
 * ============================================================ */

static const char *MPU6050_TAG = "MPU6050";

static i2c_master_bus_handle_t g_mpu6050_bus = NULL;
static i2c_master_dev_handle_t g_mpu6050_dev = NULL;

static bool g_mpu6050_owns_bus = false;

static TaskHandle_t g_mpu6050_task_handle = NULL;

static esp_timer_handle_t g_mpu6050_sample_timer = NULL;
static esp_timer_handle_t g_mpu6050_delay_timer = NULL;

static SemaphoreHandle_t g_mpu6050_control_mutex = NULL;
static SemaphoreHandle_t g_mpu6050_i2c_mutex = NULL;
static SemaphoreHandle_t g_mpu6050_task_exit_sem = NULL;
static SemaphoreHandle_t g_mpu6050_pause_ack_sem = NULL;
static SemaphoreHandle_t g_mpu6050_delay_sem = NULL;

static bool g_mpu6050_initialized = false;
static bool g_mpu6050_calibrated = false;
static bool g_mpu6050_sampling = false;
static bool g_mpu6050_pause_sampling = false;
static bool g_mpu6050_filter_initialized = false;
static bool g_mpu6050_attitude_initialized = false;
static bool g_mpu6050_timebase_reset_requested = true;

/*
 * Private yaw integrator state.
 *
 * Public API remains unchanged.  Keeping the previous bias-corrected,
 * filtered Z-rate allows trapezoidal integration instead of a simple
 * forward-Euler step.
 */
static bool g_mpu6050_yaw_rate_initialized = false;
static float g_mpu6050_prev_yaw_rate_dps = 0.0f;

static uint8_t g_mpu6050_who_am_i = 0;

static mpu6050_raw_t g_mpu6050_raw;
static mpu6050_data_t g_mpu6050_data;
static mpu6050_attitude_t g_mpu6050_attitude;
static mpu6050_bias_t g_mpu6050_bias;

static bool g_mpu6050_stationary = false;

static uint32_t g_mpu6050_sample_count = 0;
static uint32_t g_mpu6050_read_error_count = 0;

static int64_t g_mpu6050_timestamp_us = 0;

static bool g_mpu6050_debug_enable =
    MPU6050_DEBUG_DEFAULT_ENABLE;

static uint32_t g_mpu6050_debug_period_ms =
    MPU6050_DEBUG_DEFAULT_PERIOD_MS;

static int64_t g_mpu6050_debug_last_print_us = 0;

/*
 * 閹碘偓閺堝鍙曞鈧悩鑸碘偓浣峰▏閻劌鎮撴稉鈧幎?spinlock閵?
 * I2C/闂冭顢ｇ粵澶婄窡缂佹繀绗夐弨鎹愮箻 critical section閵?
 */
static portMUX_TYPE g_mpu6050_lock =
    portMUX_INITIALIZER_UNLOCKED;


/* ============================================================
 * 19. 閺佹澘顒熷銉ュ徔
 * ============================================================ */

static float mpu6050_wrap_180(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}


static int16_t mpu6050_make_int16(
    uint8_t high,
    uint8_t low
)
{
    return (int16_t)(
        ((uint16_t)high << 8) |
        (uint16_t)low
    );
}


/* ============================================================
 * 20. Control / runtime resource
 * ============================================================ */

static esp_err_t mpu6050_ensure_control_mutex(void)
{
    if (g_mpu6050_control_mutex != NULL)
    {
        return ESP_OK;
    }

    g_mpu6050_control_mutex =
        xSemaphoreCreateMutex();

    if (g_mpu6050_control_mutex == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}


static esp_err_t mpu6050_control_take(void)
{
    esp_err_t ret =
        mpu6050_ensure_control_mutex();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(
            g_mpu6050_control_mutex,
            portMAX_DELAY
        ) != pdTRUE)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}


static void mpu6050_control_give(void)
{
    if (g_mpu6050_control_mutex != NULL)
    {
        xSemaphoreGive(
            g_mpu6050_control_mutex
        );
    }
}


static void mpu6050_sample_timer_callback(void *arg)
{
    (void)arg;

    /*
     * Callback 閸欘亣绀嬬拹锝呮暅闁辨帡鍣伴弽?Task閵?
     * 娑撳秴婀?esp_timer task 娑擃厼浠涙禒璁崇秿 I2C 閹存牗璇為悙纭咁吀缁犳ぜ鈧?
     *
     * 闁氨鐓℃稉?task handle 閻ㄥ嫯顕伴崘娆愭杹閸︺劌鎮撴稉鈧?critical section閿?
     * 闁灝鍘?deinit 閺冭埖瀣侀崚鏉垮嚒缂佸繐銇戦弫鍫㈡畱 task handle閵?
     */
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    if (g_mpu6050_sampling &&
        !g_mpu6050_pause_sampling &&
        (g_mpu6050_task_handle != NULL))
    {
        xTaskNotifyGive(
            g_mpu6050_task_handle
        );
    }

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


static void mpu6050_delay_timer_callback(void *arg)
{
    (void)arg;

    if (g_mpu6050_delay_sem != NULL)
    {
        xSemaphoreGive(
            g_mpu6050_delay_sem
        );
    }
}


static esp_err_t mpu6050_create_runtime_resources(void)
{
    esp_timer_create_args_t timer_args = {0};

    if (g_mpu6050_i2c_mutex == NULL)
    {
        g_mpu6050_i2c_mutex =
            xSemaphoreCreateMutex();

        if (g_mpu6050_i2c_mutex == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_mpu6050_task_exit_sem == NULL)
    {
        g_mpu6050_task_exit_sem =
            xSemaphoreCreateBinary();

        if (g_mpu6050_task_exit_sem == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_mpu6050_pause_ack_sem == NULL)
    {
        g_mpu6050_pause_ack_sem =
            xSemaphoreCreateBinary();

        if (g_mpu6050_pause_ack_sem == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_mpu6050_delay_sem == NULL)
    {
        g_mpu6050_delay_sem =
            xSemaphoreCreateBinary();

        if (g_mpu6050_delay_sem == NULL)
        {
            return ESP_ERR_NO_MEM;
        }
    }

    if (g_mpu6050_sample_timer == NULL)
    {
        timer_args.callback =
            mpu6050_sample_timer_callback;

        timer_args.arg = NULL;
        timer_args.dispatch_method =
            ESP_TIMER_TASK;

        timer_args.name =
            "mpu6050_sample";

        esp_err_t ret =
            esp_timer_create(
                &timer_args,
                &g_mpu6050_sample_timer
            );

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    if (g_mpu6050_delay_timer == NULL)
    {
        timer_args =
            (esp_timer_create_args_t){0};

        timer_args.callback =
            mpu6050_delay_timer_callback;

        timer_args.arg = NULL;
        timer_args.dispatch_method =
            ESP_TIMER_TASK;

        timer_args.name =
            "mpu6050_delay";

        esp_err_t ret =
            esp_timer_create(
                &timer_args,
                &g_mpu6050_delay_timer
            );

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    return ESP_OK;
}


static esp_err_t mpu6050_stop_timer(
    esp_timer_handle_t timer
)
{
    if (timer == NULL)
    {
        return ESP_OK;
    }

    if (esp_timer_is_active(timer))
    {
        esp_err_t ret =
            esp_timer_stop(timer);

        if ((ret != ESP_OK) &&
            (ret != ESP_ERR_INVALID_STATE))
        {
            return ret;
        }
    }

    /*
     * 鏉╂瑩鍣烽崣顏冨▏閻劍妲戠涵顔炬畱 1 tick 鐠佲晜顒炵粵澶婄窡閿?
     * 娑撳秳绱伴崙铏瑰箛 pdMS_TO_TICKS(5) == 0 閻ㄥ嫰妫舵０妯糕偓?
     */
    for (uint32_t i = 0;
         i < MPU6050_TIMER_QUIESCE_TICKS;
         i++)
    {
        if (!esp_timer_is_active(timer))
        {
            return ESP_OK;
        }

        vTaskDelay(1);
    }

    return ESP_ERR_TIMEOUT;
}


static void mpu6050_destroy_runtime_resources(void)
{
    if (g_mpu6050_sample_timer != NULL)
    {
        (void)mpu6050_stop_timer(
            g_mpu6050_sample_timer
        );

        (void)esp_timer_delete(
            g_mpu6050_sample_timer
        );

        g_mpu6050_sample_timer =
            NULL;
    }

    if (g_mpu6050_delay_timer != NULL)
    {
        (void)mpu6050_stop_timer(
            g_mpu6050_delay_timer
        );

        (void)esp_timer_delete(
            g_mpu6050_delay_timer
        );

        g_mpu6050_delay_timer =
            NULL;
    }

    if (g_mpu6050_delay_sem != NULL)
    {
        vSemaphoreDelete(
            g_mpu6050_delay_sem
        );

        g_mpu6050_delay_sem =
            NULL;
    }

    if (g_mpu6050_pause_ack_sem != NULL)
    {
        vSemaphoreDelete(
            g_mpu6050_pause_ack_sem
        );

        g_mpu6050_pause_ack_sem =
            NULL;
    }

    if (g_mpu6050_task_exit_sem != NULL)
    {
        vSemaphoreDelete(
            g_mpu6050_task_exit_sem
        );

        g_mpu6050_task_exit_sem =
            NULL;
    }

    if (g_mpu6050_i2c_mutex != NULL)
    {
        vSemaphoreDelete(
            g_mpu6050_i2c_mutex
        );

        g_mpu6050_i2c_mutex =
            NULL;
    }
}


/* ============================================================
 * 21. 妤傛绨挎惔锔剧搼瀵?
 *
 * 閸忔娊鏁悙鐧哥窗
 * 娑撳秳濞囬悽?pdMS_TO_TICKS(5)閵?
 * 閸ョ姾鈧?CONFIG_FREERTOS_HZ=100 閺冩湹绡冩稉宥勭窗瀵版鍩?0 tick閵?
 * ============================================================ */

static esp_err_t mpu6050_wait_us(
    uint64_t delay_us
)
{
    esp_err_t ret;

    if (delay_us == 0U)
    {
        return ESP_OK;
    }

    if ((g_mpu6050_delay_timer == NULL) ||
        (g_mpu6050_delay_sem == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(
               g_mpu6050_delay_sem,
               0
           ) == pdTRUE)
    {
        /* drain */
    }

    ret =
        esp_timer_start_once(
            g_mpu6050_delay_timer,
            delay_us
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (xSemaphoreTake(
            g_mpu6050_delay_sem,
            portMAX_DELAY
        ) != pdTRUE)
    {
        (void)mpu6050_stop_timer(
            g_mpu6050_delay_timer
        );

        return ESP_FAIL;
    }

    return ESP_OK;
}


static esp_err_t mpu6050_wait_until_us(
    int64_t target_us
)
{
    int64_t now_us =
        esp_timer_get_time();

    if (target_us <= now_us)
    {
        return ESP_OK;
    }

    return mpu6050_wait_us(
        (uint64_t)(target_us - now_us)
    );
}


/*
 * 閸氬本顒為弳鍌氫粻閸氬骸褰撮柌鍥ㄧ壉閵?
 *
 * 鏉╂柨娲?ESP_OK 閺冩湹绻氱拠渚婄窗
 * 1. 閸涖劍婀?timer 瀹告彃浠犲顫幢
 * 2. 闁插洦鐗?Task 瀹歌尙绮＄憴鍌氱檪閸?pause=true閿?
 * 3. 娑斿澧犲鑼病瀵偓婵娈?sample processing 瀹告彃鐣幋鎰┾偓?
 *
 * 閸ョ姾鈧?calibration / pause() 娑撳秹娓剁憰渚€娼垾婊呭娑撯偓娑?delay閳ユ繃娼甸柆璺ㄧ彽閹降鈧?
 */
static esp_err_t mpu6050_pause_sampler_sync(
    bool *timer_was_active
)
{
    bool active = false;
    TaskHandle_t task = NULL;

    if (timer_was_active != NULL)
    {
        *timer_was_active =
            false;
    }

    if (g_mpu6050_sample_timer != NULL)
    {
        active =
            esp_timer_is_active(
                g_mpu6050_sample_timer
            );
    }

    if (timer_was_active != NULL)
    {
        *timer_was_active =
            active;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_pause_sampling =
        true;

    g_mpu6050_timebase_reset_requested =
        true;

    task =
        g_mpu6050_task_handle;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (active)
    {
        esp_err_t ret =
            mpu6050_stop_timer(
                g_mpu6050_sample_timer
            );

        if (ret != ESP_OK)
        {
            return ret;
        }
    }

    if (task == NULL)
    {
        return ESP_OK;
    }

    if (g_mpu6050_pause_ack_sem == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(
               g_mpu6050_pause_ack_sem,
               0
           ) == pdTRUE)
    {
        /* drain stale ACK */
    }

    /*
     * timer 瀹告彃浠犻敍灞芥礈濮濄倛绻栨稉鈧柅姘辩叀閸欘亞鏁ゆ禍搴ゎ唨 Task 鏉╂稑鍙?paused 閸掑棙鏁妴?
     * 婵″倹鐏?Task 濮濓絽婀径鍕倞娑撳﹣绔寸粭?sample閿涘矂鈧氨鐓℃导姘箽閻ｆ瑥鍩屾稉瀣╃濞嗏€虫儕閻滎垬鈧?
     */
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    if (g_mpu6050_task_handle != NULL)
    {
        xTaskNotifyGive(
            g_mpu6050_task_handle
        );
    }

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (xSemaphoreTake(
            g_mpu6050_pause_ack_sem,
            pdMS_TO_TICKS(
                MPU6050_DEINIT_WAIT_MS
            )
        ) != pdTRUE)
    {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}


/* ============================================================
 * 22. I2C 鎼存洖鐪?
 * ============================================================ */

static esp_err_t mpu6050_write_register_nolock(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t tx[2] =
    {
        reg,
        value
    };

    if (g_mpu6050_dev == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return i2c_master_transmit(
        g_mpu6050_dev,
        tx,
        sizeof(tx),
        MPU6050_I2C_TIMEOUT_MS
    );
}


static esp_err_t mpu6050_read_registers_nolock(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    if ((g_mpu6050_dev == NULL) ||
        (data == NULL) ||
        (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_master_transmit_receive(
        g_mpu6050_dev,
        &reg,
        1,
        data,
        length,
        MPU6050_I2C_TIMEOUT_MS
    );
}


static esp_err_t mpu6050_write_register(
    uint8_t reg,
    uint8_t value
)
{
    esp_err_t ret;

    if (g_mpu6050_i2c_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(
            g_mpu6050_i2c_mutex,
            portMAX_DELAY
        ) != pdTRUE)
    {
        return ESP_FAIL;
    }

    ret =
        mpu6050_write_register_nolock(
            reg,
            value
        );

    xSemaphoreGive(
        g_mpu6050_i2c_mutex
    );

    return ret;
}


static esp_err_t mpu6050_read_registers(
    uint8_t reg,
    uint8_t *data,
    size_t length
)
{
    esp_err_t ret;

    if ((data == NULL) ||
        (length == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_mpu6050_i2c_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(
            g_mpu6050_i2c_mutex,
            portMAX_DELAY
        ) != pdTRUE)
    {
        return ESP_FAIL;
    }

    ret =
        mpu6050_read_registers_nolock(
            reg,
            data,
            length
        );

    xSemaphoreGive(
        g_mpu6050_i2c_mutex
    );

    return ret;
}


static esp_err_t mpu6050_read_register(
    uint8_t reg,
    uint8_t *value
)
{
    return mpu6050_read_registers(
        reg,
        value,
        1
    );
}


/* ============================================================
 * 23. Raw 閺佺増宓佺拠璇插絿
 * ============================================================ */

static esp_err_t mpu6050_read_raw_nolock(
    mpu6050_raw_t *raw
)
{
    uint8_t buffer[14];

    if (raw == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret =
        mpu6050_read_registers_nolock(
            MPU6050_REG_ACCEL_XOUT_H,
            buffer,
            sizeof(buffer)
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    raw->ax =
        mpu6050_make_int16(
            buffer[0],
            buffer[1]
        );

    raw->ay =
        mpu6050_make_int16(
            buffer[2],
            buffer[3]
        );

    raw->az =
        mpu6050_make_int16(
            buffer[4],
            buffer[5]
        );

    raw->temperature =
        mpu6050_make_int16(
            buffer[6],
            buffer[7]
        );

    raw->gx =
        mpu6050_make_int16(
            buffer[8],
            buffer[9]
        );

    raw->gy =
        mpu6050_make_int16(
            buffer[10],
            buffer[11]
        );

    raw->gz =
        mpu6050_make_int16(
            buffer[12],
            buffer[13]
        );

    return ESP_OK;
}


static esp_err_t mpu6050_read_raw_internal(
    mpu6050_raw_t *raw
)
{
    esp_err_t ret;

    if (raw == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_mpu6050_i2c_mutex == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (xSemaphoreTake(
            g_mpu6050_i2c_mutex,
            portMAX_DELAY
        ) != pdTRUE)
    {
        return ESP_FAIL;
    }

    ret =
        mpu6050_read_raw_nolock(raw);

    xSemaphoreGive(
        g_mpu6050_i2c_mutex
    );

    return ret;
}


/* ============================================================
 * 24. Raw -> 閻椻晝鎮婇崡鏇氱秴
 * ============================================================ */

static void mpu6050_convert_raw(
    const mpu6050_raw_t *raw,
    mpu6050_data_t *data
)
{
    mpu6050_bias_t bias;

    if ((raw == NULL) ||
        (data == NULL))
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    bias =
        g_mpu6050_bias;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    data->ax =
        ((float)raw->ax /
         MPU6050_ACCEL_SCALE)
        -
        bias.ax;

    data->ay =
        ((float)raw->ay /
         MPU6050_ACCEL_SCALE)
        -
        bias.ay;

    data->az =
        ((float)raw->az /
         MPU6050_ACCEL_SCALE)
        -
        bias.az;

    data->gx =
        ((float)raw->gx /
         MPU6050_GYRO_SCALE)
        -
        bias.gx;

    data->gy =
        ((float)raw->gy /
         MPU6050_GYRO_SCALE)
        -
        bias.gy;

    data->gz =
        ((float)raw->gz /
         MPU6050_GYRO_SCALE)
        -
        bias.gz;

    data->temperature =
        ((float)raw->temperature /
         340.0f)
        +
        36.53f;
}


/* ============================================================
 * 25. 濠娿倖灏?/ 闂堟瑦顒?/ 婵寧鈧?
 * ============================================================ */

static void mpu6050_filter_from_previous(
    const mpu6050_data_t *input,
    const mpu6050_data_t *previous,
    mpu6050_data_t *output
)
{
    const float accel_alpha =
        MPU6050_ACCEL_LPF_ALPHA;

    const float gyro_alpha =
        MPU6050_GYRO_LPF_ALPHA;

    output->ax =
        accel_alpha * previous->ax +
        (1.0f - accel_alpha) * input->ax;

    output->ay =
        accel_alpha * previous->ay +
        (1.0f - accel_alpha) * input->ay;

    output->az =
        accel_alpha * previous->az +
        (1.0f - accel_alpha) * input->az;

    output->gx =
        gyro_alpha * previous->gx +
        (1.0f - gyro_alpha) * input->gx;

    output->gy =
        gyro_alpha * previous->gy +
        (1.0f - gyro_alpha) * input->gy;

    output->gz =
        gyro_alpha * previous->gz +
        (1.0f - gyro_alpha) * input->gz;

    output->temperature =
        input->temperature;
}


static bool mpu6050_calculate_stationary(
    const mpu6050_data_t *data
)
{
    const float gyro_mag =
        sqrtf(
            data->gx * data->gx +
            data->gy * data->gy +
            data->gz * data->gz
        );

    const float accel_mag =
        sqrtf(
            data->ax * data->ax +
            data->ay * data->ay +
            data->az * data->az
        );

    return
        (gyro_mag <
         MPU6050_STATIONARY_GYRO_DPS)
        &&
        (fabsf(accel_mag - 1.0f) <
         MPU6050_STATIONARY_ACCEL_TOL);
}


static void mpu6050_accel_angles(
    const mpu6050_data_t *data,
    float *roll_acc,
    float *pitch_acc
)
{
    *roll_acc =
        atan2f(
            data->ay,
            data->az
        )
        *
        MPU6050_RAD_TO_DEG;

    *pitch_acc =
        atan2f(
            -data->ax,
            sqrtf(
                data->ay * data->ay +
                data->az * data->az
            )
        )
        *
        MPU6050_RAD_TO_DEG;
}


static void mpu6050_process_sample(
    const mpu6050_raw_t *raw,
    float dt,
    int64_t timestamp_us
)
{
    mpu6050_data_t converted;
    mpu6050_data_t previous;
    mpu6050_data_t filtered;

    bool filter_initialized;

    float filtered_roll_acc;
    float filtered_pitch_acc;
    float converted_roll_acc;
    float converted_pitch_acc;

    bool filtered_stationary;
    bool converted_stationary;

    if (raw == NULL)
    {
        return;
    }

    /*
     * dt == 0 is intentional after a timebase reset (startup after
     * calibration, pause/resume, attitude reset).  In that case the
     * current sample refreshes filter state but must not create a
     * fictitious yaw increment.
     *
     * Very small non-zero or excessively large dt values are treated as
     * timing anomalies and fall back to the nominal sample period.
     */
    if (dt < 0.0f)
    {
        dt = 0.0f;
    }
    else if (((dt > 0.0f) && (dt < 0.001f)) ||
             (dt > 0.050f))
    {
        dt =
            MPU6050_SAMPLE_DT;
    }

    mpu6050_convert_raw(
        raw,
        &converted
    );

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    previous =
        g_mpu6050_data;

    filter_initialized =
        g_mpu6050_filter_initialized;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (filter_initialized)
    {
        mpu6050_filter_from_previous(
            &converted,
            &previous,
            &filtered
        );
    }
    else
    {
        filtered =
            converted;
    }

    filtered_stationary =
        mpu6050_calculate_stationary(
            &filtered
        );

    converted_stationary =
        mpu6050_calculate_stationary(
            &converted
        );

    mpu6050_accel_angles(
        &filtered,
        &filtered_roll_acc,
        &filtered_pitch_acc
    );

    mpu6050_accel_angles(
        &converted,
        &converted_roll_acc,
        &converted_pitch_acc
    );

    /*
     * 閸︺劋绔村▎锛勭叚 critical section 閸愬懎鎮撻弮鑸靛絹娴溿倧绱?
     * raw/data/attitude/count/timestamp閵?
     */
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    if (!g_mpu6050_filter_initialized)
    {
        filtered =
            converted;

        filtered_roll_acc =
            converted_roll_acc;

        filtered_pitch_acc =
            converted_pitch_acc;

        filtered_stationary =
            converted_stationary;

        g_mpu6050_filter_initialized =
            true;
    }

    if (!g_mpu6050_attitude_initialized)
    {
        g_mpu6050_attitude.roll =
            filtered_roll_acc;

        g_mpu6050_attitude.pitch =
            filtered_pitch_acc;

        g_mpu6050_attitude.yaw =
            0.0f;

        g_mpu6050_prev_yaw_rate_dps =
            filtered.gz;

        g_mpu6050_yaw_rate_initialized =
            true;

        g_mpu6050_attitude_initialized =
            true;
    }
    else
    {
        const float alpha =
            MPU6050_COMPLEMENTARY_ALPHA;

        g_mpu6050_attitude.roll =
            alpha *
            (
                g_mpu6050_attitude.roll +
                filtered.gx * dt
            )
            +
            (1.0f - alpha) *
            filtered_roll_acc;

        g_mpu6050_attitude.pitch =
            alpha *
            (
                g_mpu6050_attitude.pitch +
                filtered.gy * dt
            )
            +
            (1.0f - alpha) *
            filtered_pitch_acc;

        /*
         * MPU6050 has no absolute yaw reference, so yaw is still a
         * relative gyro-Z integration and will exhibit long-term drift.
         *
         * Use trapezoidal integration to reduce finite-sample integration
         * error and phase-related angle bias during start/stop motion.
         * dt == 0 is used after a timebase reset and intentionally does
         * not integrate an angle.
         */
        if (dt > 0.0f)
        {
            if (g_mpu6050_yaw_rate_initialized)
            {
                g_mpu6050_attitude.yaw +=
                    0.5f *
                    (g_mpu6050_prev_yaw_rate_dps +
                     filtered.gz) *
                    dt;
            }
            else
            {
                g_mpu6050_attitude.yaw +=
                    filtered.gz * dt;
            }
        }

        g_mpu6050_prev_yaw_rate_dps =
            filtered.gz;

        g_mpu6050_yaw_rate_initialized =
            true;
    }

    g_mpu6050_raw =
        *raw;

    g_mpu6050_data =
        filtered;

    g_mpu6050_stationary =
        filtered_stationary;

    g_mpu6050_timestamp_us =
        timestamp_us;

    g_mpu6050_sample_count++;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


/* ============================================================
 * 26. Calibration
 * ============================================================ */

static esp_err_t mpu6050_calibrate_internal(
    uint16_t samples,
    bool calibrate_accel
)
{
    mpu6050_raw_t raw;

    int64_t sum_ax = 0;
    int64_t sum_ay = 0;
    int64_t sum_az = 0;

    int64_t sum_gx = 0;
    int64_t sum_gy = 0;
    int64_t sum_gz = 0;

    /*
     * Sum of squares for calibration-quality validation.
     * double is used only during calibration, not in the 200 Hz runtime
     * path.
     */
    double sum_sq_ax = 0.0;
    double sum_sq_ay = 0.0;
    double sum_sq_az = 0.0;

    double sum_sq_gx = 0.0;
    double sum_sq_gy = 0.0;
    double sum_sq_gz = 0.0;

    mpu6050_bias_t previous_bias;
    mpu6050_bias_t new_bias = {0};

    bool previous_pause;
    bool timer_was_active;

    esp_err_t ret = ESP_OK;

    if (samples < 50U)
    {
        samples =
            50U;
    }

    if ((g_mpu6050_i2c_mutex == NULL) ||
        (g_mpu6050_delay_timer == NULL) ||
        (g_mpu6050_delay_sem == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    previous_pause =
        g_mpu6050_pause_sampling;

    previous_bias =
        g_mpu6050_bias;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    ret =
        mpu6050_pause_sampler_sync(
            &timer_was_active
        );

    if (ret != ESP_OK)
    {
        goto restore_sampling;
    }

    if (xSemaphoreTake(
            g_mpu6050_i2c_mutex,
            portMAX_DELAY
        ) != pdTRUE)
    {
        ret = ESP_FAIL;
        goto restore_sampling;
    }

    ESP_LOGI(
        MPU6050_TAG,
        "Calibration started: %u samples @ %u Hz. Keep sensor stationary.",
        (unsigned)samples,
        (unsigned)MPU6050_SAMPLE_RATE_HZ
    );

    /*
     * 娑撱垹绱旀稉鈧▎鈥冲讲閼宠棄浜搁弮褏娈戦弽閿嬫拱閵?
     */
    ret =
        mpu6050_read_raw_nolock(
            &raw
        );

    if (ret == ESP_OK)
    {
        ret =
            mpu6050_wait_us(
                MPU6050_SAMPLE_PERIOD_US
            );
    }

    int64_t next_sample_us =
        esp_timer_get_time();

    for (uint16_t i = 0;
         (i < samples) &&
         (ret == ESP_OK);
         i++)
    {
        ret =
            mpu6050_read_raw_nolock(
                &raw
            );

        if (ret != ESP_OK)
        {
            break;
        }

        sum_ax += raw.ax;
        sum_ay += raw.ay;
        sum_az += raw.az;

        sum_gx += raw.gx;
        sum_gy += raw.gy;
        sum_gz += raw.gz;

        sum_sq_ax +=
            (double)raw.ax * (double)raw.ax;
        sum_sq_ay +=
            (double)raw.ay * (double)raw.ay;
        sum_sq_az +=
            (double)raw.az * (double)raw.az;

        sum_sq_gx +=
            (double)raw.gx * (double)raw.gx;
        sum_sq_gy +=
            (double)raw.gy * (double)raw.gy;
        sum_sq_gz +=
            (double)raw.gz * (double)raw.gz;

        if ((i + 1U) < samples)
        {
            next_sample_us +=
                (int64_t)MPU6050_SAMPLE_PERIOD_US;

            ret =
                mpu6050_wait_until_us(
                    next_sample_us
                );
        }
    }

    xSemaphoreGive(
        g_mpu6050_i2c_mutex
    );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Calibration I2C/timing failed: %s",
            esp_err_to_name(ret)
        );

        goto restore_sampling;
    }

    /*
     * Validate the calibration data before accepting a new bias.
     *
     * A moving sensor can otherwise produce a perfectly valid-looking
     * arithmetic average but a catastrophically wrong zero-rate bias.
     */
    const double inv_samples =
        1.0 / (double)samples;

    const double mean_ax_raw =
        (double)sum_ax * inv_samples;
    const double mean_ay_raw =
        (double)sum_ay * inv_samples;
    const double mean_az_raw =
        (double)sum_az * inv_samples;

    const double mean_gx_raw =
        (double)sum_gx * inv_samples;
    const double mean_gy_raw =
        (double)sum_gy * inv_samples;
    const double mean_gz_raw =
        (double)sum_gz * inv_samples;

    double var_ax_raw =
        sum_sq_ax * inv_samples -
        mean_ax_raw * mean_ax_raw;

    double var_ay_raw =
        sum_sq_ay * inv_samples -
        mean_ay_raw * mean_ay_raw;

    double var_az_raw =
        sum_sq_az * inv_samples -
        mean_az_raw * mean_az_raw;

    double var_gx_raw =
        sum_sq_gx * inv_samples -
        mean_gx_raw * mean_gx_raw;

    double var_gy_raw =
        sum_sq_gy * inv_samples -
        mean_gy_raw * mean_gy_raw;

    double var_gz_raw =
        sum_sq_gz * inv_samples -
        mean_gz_raw * mean_gz_raw;

    if (var_ax_raw < 0.0) var_ax_raw = 0.0;
    if (var_ay_raw < 0.0) var_ay_raw = 0.0;
    if (var_az_raw < 0.0) var_az_raw = 0.0;

    if (var_gx_raw < 0.0) var_gx_raw = 0.0;
    if (var_gy_raw < 0.0) var_gy_raw = 0.0;
    if (var_gz_raw < 0.0) var_gz_raw = 0.0;

    const float std_ax_g =
        (float)(sqrt(var_ax_raw) /
                (double)MPU6050_ACCEL_SCALE);

    const float std_ay_g =
        (float)(sqrt(var_ay_raw) /
                (double)MPU6050_ACCEL_SCALE);

    const float std_az_g =
        (float)(sqrt(var_az_raw) /
                (double)MPU6050_ACCEL_SCALE);

    const float std_gx_dps =
        (float)(sqrt(var_gx_raw) /
                (double)MPU6050_GYRO_SCALE);

    const float std_gy_dps =
        (float)(sqrt(var_gy_raw) /
                (double)MPU6050_GYRO_SCALE);

    const float std_gz_dps =
        (float)(sqrt(var_gz_raw) /
                (double)MPU6050_GYRO_SCALE);

    const float mean_ax_g =
        (float)(mean_ax_raw /
                (double)MPU6050_ACCEL_SCALE);

    const float mean_ay_g =
        (float)(mean_ay_raw /
                (double)MPU6050_ACCEL_SCALE);

    const float mean_az_g =
        (float)(mean_az_raw /
                (double)MPU6050_ACCEL_SCALE);

    const float mean_acc_mag_g =
        sqrtf(
            mean_ax_g * mean_ax_g +
            mean_ay_g * mean_ay_g +
            mean_az_g * mean_az_g
        );

    if ((std_ax_g >
         MPU6050_CALIBRATION_ACCEL_STD_MAX_G) ||
        (std_ay_g >
         MPU6050_CALIBRATION_ACCEL_STD_MAX_G) ||
        (std_az_g >
         MPU6050_CALIBRATION_ACCEL_STD_MAX_G))
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Calibration rejected: accel vibration | "
            "accel std [%.4f %.4f %.4f] g",
            std_ax_g,
            std_ay_g,
            std_az_g
        );

        ret = ESP_ERR_INVALID_STATE;
        goto restore_sampling;
    }

    if ((std_gx_dps >
         MPU6050_CALIBRATION_GYRO_STD_MAX_DPS) ||
        (std_gy_dps >
         MPU6050_CALIBRATION_GYRO_STD_MAX_DPS) ||
        (std_gz_dps >
         MPU6050_CALIBRATION_GYRO_STD_MAX_DPS))
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Calibration rejected: sensor moving | "
            "gyro std [%.3f %.3f %.3f] dps",
            std_gx_dps,
            std_gy_dps,
            std_gz_dps
        );

        ret = ESP_ERR_INVALID_STATE;
        goto restore_sampling;
    }

    if ((mean_acc_mag_g <
         MPU6050_CALIBRATION_ACCEL_MAG_MIN_G) ||
        (mean_acc_mag_g >
         MPU6050_CALIBRATION_ACCEL_MAG_MAX_G))
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Calibration rejected: accel magnitude %.3fg outside [%.2f, %.2f]g",
            mean_acc_mag_g,
            (double)MPU6050_CALIBRATION_ACCEL_MAG_MIN_G,
            (double)MPU6050_CALIBRATION_ACCEL_MAG_MAX_G
        );

        ret = ESP_ERR_INVALID_STATE;
        goto restore_sampling;
    }

    if (calibrate_accel &&
        ((fabsf(mean_ax_g) >
          MPU6050_CALIBRATION_LEVEL_MAX_XY_G) ||
         (fabsf(mean_ay_g) >
          MPU6050_CALIBRATION_LEVEL_MAX_XY_G) ||
         (mean_az_g <
          MPU6050_CALIBRATION_LEVEL_MIN_Z_G)))
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Level calibration rejected: pose not level/Z-up | "
            "mean accel [%.3f %.3f %.3f]g",
            mean_ax_g,
            mean_ay_g,
            mean_az_g
        );

        ret = ESP_ERR_INVALID_STATE;
        goto restore_sampling;
    }

    ESP_LOGI(
        MPU6050_TAG,
        "Calibration quality | gyro std [%.3f %.3f %.3f] dps | "
        "accel std [%.4f %.4f %.4f]g | "
        "mean accel [%.3f %.3f %.3f]g | |a|=%.3fg",
        std_gx_dps,
        std_gy_dps,
        std_gz_dps,
        std_ax_g,
        std_ay_g,
        std_az_g,
        mean_ax_g,
        mean_ay_g,
        mean_az_g,
        mean_acc_mag_g
    );

    new_bias.gx =
        ((float)sum_gx /
         (float)samples)
        /
        MPU6050_GYRO_SCALE;

    new_bias.gy =
        ((float)sum_gy /
         (float)samples)
        /
        MPU6050_GYRO_SCALE;

    new_bias.gz =
        ((float)sum_gz /
         (float)samples)
        /
        MPU6050_GYRO_SCALE;

    if (calibrate_accel)
    {
        new_bias.ax =
            ((float)sum_ax /
             (float)samples)
            /
            MPU6050_ACCEL_SCALE;

        new_bias.ay =
            ((float)sum_ay /
             (float)samples)
            /
            MPU6050_ACCEL_SCALE;

        new_bias.az =
            (
                ((float)sum_az /
                 (float)samples)
                /
                MPU6050_ACCEL_SCALE
            )
            -
            1.0f;
    }
    else
    {
        new_bias.ax =
            previous_bias.ax;

        new_bias.ay =
            previous_bias.ay;

        new_bias.az =
            previous_bias.az;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_bias =
        new_bias;

    g_mpu6050_calibrated =
        true;

    g_mpu6050_filter_initialized =
        false;

    g_mpu6050_attitude_initialized =
        false;

    g_mpu6050_attitude =
        (mpu6050_attitude_t){0};

    g_mpu6050_yaw_rate_initialized =
        false;

    g_mpu6050_prev_yaw_rate_dps =
        0.0f;

    g_mpu6050_timebase_reset_requested =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    ESP_LOGI(
        MPU6050_TAG,
        "Calibration finished | "
        "Gyro bias [%.3f %.3f %.3f] dps | "
        "Accel bias [%.4f %.4f %.4f] g",
        new_bias.gx,
        new_bias.gy,
        new_bias.gz,
        new_bias.ax,
        new_bias.ay,
        new_bias.az
    );

restore_sampling:

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_pause_sampling =
        previous_pause;

    g_mpu6050_timebase_reset_requested =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (timer_was_active &&
        !previous_pause &&
        (g_mpu6050_sample_timer != NULL))
    {
        esp_err_t start_ret =
            esp_timer_start_periodic(
                g_mpu6050_sample_timer,
                MPU6050_SAMPLE_PERIOD_US
            );

        if ((ret == ESP_OK) &&
            (start_ret != ESP_OK))
        {
            ret =
                start_ret;
        }
    }

    return ret;
}


/* ============================================================
 * 27. MPU6050 绾兛娆㈤崚婵嗩潗閸?
 * ============================================================ */

static esp_err_t mpu6050_verify_register(
    uint8_t reg,
    uint8_t mask,
    uint8_t expected,
    const char *name
)
{
    uint8_t value = 0;

    esp_err_t ret =
        mpu6050_read_register(
            reg,
            &value
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Read-back %s failed: %s",
            name,
            esp_err_to_name(ret)
        );

        return ret;
    }

    if ((value & mask) !=
        (expected & mask))
    {
        ESP_LOGE(
            MPU6050_TAG,
            "%s verify failed: got 0x%02X, expected 0x%02X (mask 0x%02X)",
            name,
            value,
            expected,
            mask
        );

        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}


static esp_err_t mpu6050_configure_sensor(void)
{
    uint8_t who_am_i = 0;

    esp_err_t ret =
        mpu6050_read_register(
            MPU6050_REG_WHO_AM_I,
            &who_am_i
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "WHO_AM_I read failed: %s",
            esp_err_to_name(ret)
        );

        return ret;
    }

    if (who_am_i !=
        MPU6050_WHO_AM_I_VALUE)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Unexpected WHO_AM_I: 0x%02X, expected 0x%02X",
            who_am_i,
            MPU6050_WHO_AM_I_VALUE
        );

        return ESP_ERR_NOT_FOUND;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_who_am_i =
        who_am_i;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    ESP_LOGI(
        MPU6050_TAG,
        "MPU6050 found, WHO_AM_I=0x%02X",
        who_am_i
    );

    ret =
        mpu6050_write_register(
            MPU6050_REG_PWR_MGMT_1,
            0x80U
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(
        pdMS_TO_TICKS(100)
    );

    ret =
        mpu6050_write_register(
            MPU6050_REG_PWR_MGMT_1,
            0x01U
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_write_register(
            MPU6050_REG_PWR_MGMT_2,
            0x00U
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_write_register(
            MPU6050_REG_CONFIG,
            MPU6050_DLPF_CFG
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_write_register(
            MPU6050_REG_SMPLRT_DIV,
            MPU6050_SMPLRT_DIV_VALUE
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_write_register(
            MPU6050_REG_GYRO_CONFIG,
            MPU6050_GYRO_CONFIG_VALUE
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_write_register(
            MPU6050_REG_ACCEL_CONFIG,
            MPU6050_ACCEL_CONFIG_VALUE
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    vTaskDelay(
        pdMS_TO_TICKS(50)
    );

    /*
     * 鐎靛嫬鐡ㄩ崳銊ユ礀鐠囧鐛欑拠浣碘偓?
     */
    ret =
        mpu6050_verify_register(
            MPU6050_REG_PWR_MGMT_1,
            0x7FU,
            0x01U,
            "PWR_MGMT_1"
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_verify_register(
            MPU6050_REG_CONFIG,
            0x07U,
            MPU6050_DLPF_CFG,
            "CONFIG"
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_verify_register(
            MPU6050_REG_SMPLRT_DIV,
            0xFFU,
            MPU6050_SMPLRT_DIV_VALUE,
            "SMPLRT_DIV"
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_verify_register(
            MPU6050_REG_GYRO_CONFIG,
            0x18U,
            MPU6050_GYRO_CONFIG_VALUE,
            "GYRO_CONFIG"
        );

    if (ret != ESP_OK)
    {
        return ret;
    }

    ret =
        mpu6050_verify_register(
            MPU6050_REG_ACCEL_CONFIG,
            0x18U,
            MPU6050_ACCEL_CONFIG_VALUE,
            "ACCEL_CONFIG"
        );

    return ret;
}


/* ============================================================
 * 28. Debug
 * ============================================================ */

void mpu6050_print_now(void)
{
    mpu6050_snapshot_t snapshot;
    mpu6050_status_t status;

    mpu6050_get_snapshot(
        &snapshot
    );

    mpu6050_get_status(
        &status
    );

    ESP_LOGI(
        MPU6050_TAG,
        "ACC[g]=[%+.3f %+.3f %+.3f] "
        "GYRO[dps]=[%+.2f %+.2f %+.2f] "
        "ATT[deg]=[R:%+.1f P:%+.1f Y:%+.1f] "
        "TEMP=%.1fC %s | samples=%lu errors=%lu",
        snapshot.data.ax,
        snapshot.data.ay,
        snapshot.data.az,
        snapshot.data.gx,
        snapshot.data.gy,
        snapshot.data.gz,
        snapshot.attitude.roll,
        snapshot.attitude.pitch,
        snapshot.attitude.yaw,
        snapshot.data.temperature,
        snapshot.stationary
            ? "STILL"
            : "MOVE",
        (unsigned long)status.sample_count,
        (unsigned long)status.read_error_count
    );
}


static void mpu6050_debug_update(
    int64_t timestamp_us
)
{
    bool enable;
    uint32_t period_ms;
    int64_t last_print_us;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    enable =
        g_mpu6050_debug_enable;

    period_ms =
        g_mpu6050_debug_period_ms;

    last_print_us =
        g_mpu6050_debug_last_print_us;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (!enable)
    {
        return;
    }

    if ((last_print_us != 0) &&
        ((timestamp_us - last_print_us) <
         ((int64_t)period_ms * 1000LL)))
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_debug_last_print_us =
        timestamp_us;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    mpu6050_print_now();
}


/* ============================================================
 * 29. 閸氬骸褰?IMU Task
 *
 * 200Hz 閼哄倹濯块弶銉ㄥ殰 esp_timer(5000us)閿?
 * 娑撳秳绶风挧?CONFIG_FREERTOS_HZ閵?
 * ============================================================ */

static void mpu6050_task(void *arg)
{
    (void)arg;

    int64_t previous_time_us =
        esp_timer_get_time();

    for (;;)
    {
        /*
         * pdTRUE閿涙艾顩ч弸婊冾槱閻炲棗浼撶亸鏃€鍙冩禍?5ms閿涘奔娑鍐柈鐠?tick閿?
         * 娑撳秷绻樼悰宀兯夌拠鎯х础鏉╃偟鐢?I2C burst閵?
         */
        (void)ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );

        bool sampling;
        bool paused;
        bool reset_timebase;

        taskENTER_CRITICAL(
            &g_mpu6050_lock
        );

        sampling =
            g_mpu6050_sampling;

        paused =
            g_mpu6050_pause_sampling;

        reset_timebase =
            g_mpu6050_timebase_reset_requested;

        if (reset_timebase)
        {
            g_mpu6050_timebase_reset_requested =
                false;
        }

        taskEXIT_CRITICAL(
            &g_mpu6050_lock
        );

        if (!sampling)
        {
            break;
        }

        if (paused)
        {
            previous_time_us =
                esp_timer_get_time();

            if (g_mpu6050_pause_ack_sem != NULL)
            {
                xSemaphoreGive(
                    g_mpu6050_pause_ack_sem
                );
            }

            continue;
        }

        mpu6050_raw_t raw;

        esp_err_t ret =
            mpu6050_read_raw_internal(
                &raw
            );

        const int64_t current_time_us =
            esp_timer_get_time();

        if (ret == ESP_OK)
        {
            float dt;

            if (reset_timebase ||
                (previous_time_us <= 0))
            {
                /*
                 * Fresh timebase: update filter state, but do not invent
                 * one nominal sample of yaw motion.
                 */
                dt =
                    0.0f;
            }
            else
            {
                dt =
                    (float)(
                        current_time_us -
                        previous_time_us
                    )
                    /
                    1000000.0f;
            }

            previous_time_us =
                current_time_us;

            mpu6050_process_sample(
                &raw,
                dt,
                current_time_us
            );

            mpu6050_debug_update(
                current_time_us
            );
        }
        else
        {
            /*
             * Keep previous_time_us unchanged.  On the next successful
             * read, dt spans the missed sample interval so a transient
             * I2C error does not silently discard elapsed yaw time.
             */
            taskENTER_CRITICAL(
                &g_mpu6050_lock
            );

            g_mpu6050_read_error_count++;

            taskEXIT_CRITICAL(
                &g_mpu6050_lock
            );
        }
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_task_handle =
        NULL;

    g_mpu6050_sampling =
        false;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (g_mpu6050_task_exit_sem != NULL)
    {
        xSemaphoreGive(
            g_mpu6050_task_exit_sem
        );
    }

    vTaskDelete(NULL);
}


/* ============================================================
 * 30. 閸掓稑缂撻柌鍥ㄧ壉 Task
 * ============================================================ */

static esp_err_t mpu6050_create_task(void)
{
    if (g_mpu6050_task_handle != NULL)
    {
        return ESP_OK;
    }

    if ((g_mpu6050_sample_timer == NULL) ||
        (g_mpu6050_task_exit_sem == NULL) ||
        (g_mpu6050_pause_ack_sem == NULL))
    {
        return ESP_ERR_INVALID_STATE;
    }

    while (xSemaphoreTake(
               g_mpu6050_task_exit_sem,
               0
           ) == pdTRUE)
    {
        /* drain */
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_sampling =
        true;

    g_mpu6050_pause_sampling =
        false;

    g_mpu6050_timebase_reset_requested =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    BaseType_t result =
        xTaskCreate(
            mpu6050_task,
            "mpu6050",
            MPU6050_TASK_STACK_SIZE,
            NULL,
            MPU6050_TASK_PRIORITY,
            &g_mpu6050_task_handle
        );

    if (result != pdPASS)
    {
        taskENTER_CRITICAL(
            &g_mpu6050_lock
        );

        g_mpu6050_sampling =
            false;

        g_mpu6050_task_handle =
            NULL;

        taskEXIT_CRITICAL(
            &g_mpu6050_lock
        );

        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret =
        esp_timer_start_periodic(
            g_mpu6050_sample_timer,
            MPU6050_SAMPLE_PERIOD_US
        );

    if (ret != ESP_OK)
    {
        taskENTER_CRITICAL(
            &g_mpu6050_lock
        );

        g_mpu6050_sampling =
            false;

        if (g_mpu6050_task_handle != NULL)
        {
            xTaskNotifyGive(
                g_mpu6050_task_handle
            );
        }

        taskEXIT_CRITICAL(
            &g_mpu6050_lock
        );

        (void)xSemaphoreTake(
            g_mpu6050_task_exit_sem,
            pdMS_TO_TICKS(
                MPU6050_DEINIT_WAIT_MS
            )
        );

        return ret;
    }

    return ESP_OK;
}


/* ============================================================
 * 31. 閸掓繂顫愰崠鏍у彆閸忚鲸鐗宠箛?
 * ============================================================ */

static void mpu6050_clear_public_state(void)
{
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_initialized =
        false;

    g_mpu6050_calibrated =
        false;

    g_mpu6050_sampling =
        false;

    g_mpu6050_pause_sampling =
        false;

    g_mpu6050_filter_initialized =
        false;

    g_mpu6050_attitude_initialized =
        false;

    g_mpu6050_timebase_reset_requested =
        true;

    g_mpu6050_yaw_rate_initialized =
        false;

    g_mpu6050_prev_yaw_rate_dps =
        0.0f;

    g_mpu6050_who_am_i =
        0;

    g_mpu6050_raw =
        (mpu6050_raw_t){0};

    g_mpu6050_data =
        (mpu6050_data_t){0};

    g_mpu6050_attitude =
        (mpu6050_attitude_t){0};

    g_mpu6050_bias =
        (mpu6050_bias_t){0};

    g_mpu6050_stationary =
        false;

    g_mpu6050_sample_count =
        0;

    g_mpu6050_read_error_count =
        0;

    g_mpu6050_timestamp_us =
        0;

    g_mpu6050_debug_last_print_us =
        0;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


static esp_err_t mpu6050_probe_sensor(
    i2c_master_bus_handle_t bus_handle
)
{
    esp_err_t ret =
        i2c_master_probe(
            bus_handle,
            MPU6050_I2C_ADDRESS,
            MPU6050_I2C_TIMEOUT_MS
        );

    if (ret == ESP_OK)
    {
        return ESP_OK;
    }

    ESP_LOGE(
        MPU6050_TAG,
        "No ACK from configured address 0x%02X: %s",
        MPU6050_I2C_ADDRESS,
        esp_err_to_name(ret)
    );

    /*
     * 妫版繂顦婚幒銏＄ゴ閸欙缚绔存稉顏勬値濞?AD0 閸︽澘娼冮敍灞肩矌閻劋绨拠濠冩焽閿?
     * 娑撳秷鍤滈崝銊ヤ紨閸嬭渹鎱ㄩ弨鍦暏閹寸兘鍘ょ純顔衡偓?
     */
    esp_err_t alt_ret =
        i2c_master_probe(
            bus_handle,
            MPU6050_ALT_I2C_ADDRESS,
            MPU6050_I2C_TIMEOUT_MS
        );

    if (alt_ret == ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "MPU6050 ACKs at 0x%02X instead. Check AD0 or MPU6050_I2C_ADDRESS.",
            MPU6050_ALT_I2C_ADDRESS
        );

        return ESP_ERR_NOT_FOUND;
    }

    return ret;
}


static esp_err_t mpu6050_init_common(
    i2c_master_bus_handle_t bus_handle,
    bool owns_bus
)
{
    i2c_device_config_t dev_cfg = {0};

    esp_err_t ret;

    if (bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    mpu6050_clear_public_state();

    g_mpu6050_bus =
        bus_handle;

    g_mpu6050_owns_bus =
        owns_bus;

    ret =
        mpu6050_probe_sensor(
            bus_handle
        );

    if (ret != ESP_OK)
    {
        goto fail;
    }

    ret =
        mpu6050_create_runtime_resources();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Runtime resource creation failed: %s",
            esp_err_to_name(ret)
        );

        goto fail;
    }

    dev_cfg.dev_addr_length =
        I2C_ADDR_BIT_LEN_7;

    dev_cfg.device_address =
        MPU6050_I2C_ADDRESS;

    dev_cfg.scl_speed_hz =
        MPU6050_I2C_FREQ_HZ;

    ESP_LOGI(
        MPU6050_TAG,
        "Adding device address 0x%02X",
        MPU6050_I2C_ADDRESS
    );

    ret =
        i2c_master_bus_add_device(
            g_mpu6050_bus,
            &dev_cfg,
            &g_mpu6050_dev
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "i2c_master_bus_add_device failed: %s",
            esp_err_to_name(ret)
        );

        goto fail;
    }

    ret =
        mpu6050_configure_sensor();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Sensor configuration failed: %s",
            esp_err_to_name(ret)
        );

        goto fail;
    }

#if MPU6050_AUTO_GYRO_CALIBRATION
    /*
     * Give the gyro a short power-up settling interval before estimating
     * zero-rate bias.  This improves repeatability for yaw/odometry.
     */
    ESP_LOGI(
        MPU6050_TAG,
        "Startup settling for %u ms before calibration. Keep sensor still.",
        (unsigned)MPU6050_STARTUP_SETTLE_MS
    );

    vTaskDelay(
        pdMS_TO_TICKS(
            MPU6050_STARTUP_SETTLE_MS
        )
    );

    /*
     * 閸氼垰濮╅弮鎯板殰閸斻劌浠涚€瑰本鏆ｅ鏉戦挬閺嶁€冲櫙閵?
     *
     * calibrate_accel = true閿?
     *
     *      Gyro X/Y/Z -> 0 dps
     *      Accel X    -> 0 g
     *      Accel Y    -> 0 g
     *      Accel Z    -> 1 g
     *
     * 閸ョ姵顒?main() 閸欘亪娓剁憰浣界殶閻?mpu6050_init()閿?
     * 娑撳秹娓剁憰浣稿晙妫版繂顦荤拫鍐暏 mpu6050_calibrate_level()閵?
     */
    ret =
        mpu6050_calibrate_internal(
            MPU6050_CALIBRATION_SAMPLES,
            true
        );

    if (ret != ESP_OK)
    {
        goto fail;
    }
#endif

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_initialized =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    ret =
        mpu6050_create_task();

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Sampling task/timer creation failed: %s",
            esp_err_to_name(ret)
        );

        taskENTER_CRITICAL(
            &g_mpu6050_lock
        );

        g_mpu6050_initialized =
            false;

        taskEXIT_CRITICAL(
            &g_mpu6050_lock
        );

        goto fail;
    }

    ESP_LOGI(
        MPU6050_TAG,
        "MPU6050 ready: %uHz via esp_timer, accel +/-4g, gyro +/-500dps, RTOS tick=%uHz",
        (unsigned)MPU6050_SAMPLE_RATE_HZ,
        (unsigned)configTICK_RATE_HZ
    );

    return ESP_OK;

fail:

    if (g_mpu6050_dev != NULL)
    {
        (void)i2c_master_bus_rm_device(
            g_mpu6050_dev
        );

        g_mpu6050_dev =
            NULL;
    }

    mpu6050_destroy_runtime_resources();

    g_mpu6050_bus =
        NULL;

    g_mpu6050_owns_bus =
        false;

    mpu6050_clear_public_state();

    return ret;
}


/* ============================================================
 * 32. 鐎电懓顦婚敍姘冲殰瀹稿崬鍨卞?I2C Bus
 * ============================================================ */

esp_err_t mpu6050_init(void)
{
    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg =
        {0};

    i2c_master_bus_handle_t bus_handle =
        NULL;

    bus_cfg.clk_source =
        I2C_CLK_SRC_DEFAULT;

    bus_cfg.i2c_port =
        MPU6050_I2C_PORT;

    bus_cfg.sda_io_num =
        MPU6050_SDA_PIN;

    bus_cfg.scl_io_num =
        MPU6050_SCL_PIN;

    bus_cfg.glitch_ignore_cnt =
        7;

    bus_cfg.flags.enable_internal_pullup =
        MPU6050_ENABLE_INTERNAL_PULLUP;

    ESP_LOGI(
        MPU6050_TAG,
        "Creating I2C bus: port=%d SDA=%d SCL=%d pullup=%d freq=%u",
        (int)MPU6050_I2C_PORT,
        (int)MPU6050_SDA_PIN,
        (int)MPU6050_SCL_PIN,
        (int)MPU6050_ENABLE_INTERNAL_PULLUP,
        (unsigned)MPU6050_I2C_FREQ_HZ
    );

    ret =
        i2c_new_master_bus(
            &bus_cfg,
            &bus_handle
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "i2c_new_master_bus failed: %s",
            esp_err_to_name(ret)
        );

        mpu6050_control_give();
        return ret;
    }

    ret =
        mpu6050_init_common(
            bus_handle,
            true
        );

    if (ret != ESP_OK)
    {
        /*
         * init_common 婢惰精瑙﹂弮鏈电瑝娴兼艾鍨归梽?caller 閸掓稑缂撻惃?bus閵?
         */
        (void)i2c_del_master_bus(
            bus_handle
        );
    }

    mpu6050_control_give();

    return ret;
}


/* ============================================================
 * 33. 鐎电懓顦婚敍姘▏閻劌鍑￠張?I2C Bus
 * ============================================================ */

esp_err_t mpu6050_init_with_bus(
    i2c_master_bus_handle_t bus_handle
)
{
    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_OK;
    }

    ret =
        mpu6050_init_common(
            bus_handle,
            false
        );

    mpu6050_control_give();

    return ret;
}


/* ============================================================
 * 34. Deinit
 * ============================================================ */

esp_err_t mpu6050_deinit(void)
{
    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_OK;
    }

    if (xTaskGetCurrentTaskHandle() ==
        g_mpu6050_task_handle)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_sampling =
        false;

    g_mpu6050_pause_sampling =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    ret =
        mpu6050_stop_timer(
            g_mpu6050_sample_timer
        );

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Failed to stop sample timer: %s",
            esp_err_to_name(ret)
        );

        mpu6050_control_give();
        return ret;
    }

    /*
     * timer 瀹告彃浠犻崥搴礉Task 閸欘垵鍏樻禒宥夋▎婵夌偛婀?notification閵?
     * 娑撹濮╅崬銈夊晪閿涘矁顔€鐎瑰啳顫囩€?sampling=false 閸氬氦鍤滅悰宀勨偓鈧崙鎭掆偓?
     */
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    if (g_mpu6050_task_handle != NULL)
    {
        xTaskNotifyGive(
            g_mpu6050_task_handle
        );
    }

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (g_mpu6050_task_handle != NULL)
    {
        if (xSemaphoreTake(
                g_mpu6050_task_exit_sem,
                pdMS_TO_TICKS(
                    MPU6050_DEINIT_WAIT_MS
                )
            ) != pdTRUE)
        {
            ESP_LOGE(
                MPU6050_TAG,
                "Sampling task did not exit cleanly"
            );

            mpu6050_control_give();
            return ESP_ERR_TIMEOUT;
        }
    }

    if (g_mpu6050_dev != NULL)
    {
        ret =
            i2c_master_bus_rm_device(
                g_mpu6050_dev
            );

        if (ret != ESP_OK)
        {
            mpu6050_control_give();
            return ret;
        }

        g_mpu6050_dev =
            NULL;
    }

    if (g_mpu6050_owns_bus &&
        (g_mpu6050_bus != NULL))
    {
        ret =
            i2c_del_master_bus(
                g_mpu6050_bus
            );

        if (ret != ESP_OK)
        {
            mpu6050_control_give();
            return ret;
        }
    }

    g_mpu6050_bus =
        NULL;

    g_mpu6050_owns_bus =
        false;

    mpu6050_destroy_runtime_resources();

    mpu6050_clear_public_state();

    ESP_LOGI(
        MPU6050_TAG,
        "MPU6050 deinitialized cleanly"
    );

    mpu6050_control_give();

    return ESP_OK;
}


/* ============================================================
 * 35. Raw / Physical / Attitude / Snapshot
 * ============================================================ */

void mpu6050_get_raw(
    mpu6050_raw_t *raw
)
{
    if (raw == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    *raw =
        g_mpu6050_raw;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


esp_err_t mpu6050_read_raw_now(
    mpu6050_raw_t *raw
)
{
    if (raw == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    ret =
        mpu6050_read_raw_internal(
            raw
        );

    mpu6050_control_give();

    return ret;
}


void mpu6050_get_data(
    mpu6050_data_t *data
)
{
    if (data == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    *data =
        g_mpu6050_data;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


void mpu6050_get_attitude(
    mpu6050_attitude_t *attitude
)
{
    if (attitude == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    *attitude =
        g_mpu6050_attitude;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


void mpu6050_get_snapshot(
    mpu6050_snapshot_t *snapshot
)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    snapshot->raw =
        g_mpu6050_raw;

    snapshot->data =
        g_mpu6050_data;

    snapshot->attitude =
        g_mpu6050_attitude;

    snapshot->stationary =
        g_mpu6050_stationary;

    snapshot->sample_count =
        g_mpu6050_sample_count;

    snapshot->timestamp_us =
        g_mpu6050_timestamp_us;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


void mpu6050_get_status(
    mpu6050_status_t *status
)
{
    if (status == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    status->initialized =
        g_mpu6050_initialized;

    status->calibrated =
        g_mpu6050_calibrated;

    status->stationary =
        g_mpu6050_stationary;

    status->sampling =
        g_mpu6050_initialized &&
        g_mpu6050_sampling &&
        !g_mpu6050_pause_sampling;

    status->who_am_i =
        g_mpu6050_who_am_i;

    status->sample_count =
        g_mpu6050_sample_count;

    status->read_error_count =
        g_mpu6050_read_error_count;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


void mpu6050_get_bias(
    mpu6050_bias_t *bias
)
{
    if (bias == NULL)
    {
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    *bias =
        g_mpu6050_bias;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


/* ============================================================
 * 36. 韫囶偅宓?Getter
 * ============================================================ */

float mpu6050_get_roll(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_attitude.roll;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


float mpu6050_get_pitch(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_attitude.pitch;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


float mpu6050_get_yaw(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_attitude.yaw;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


float mpu6050_get_yaw_wrapped(void)
{
    return mpu6050_wrap_180(
        mpu6050_get_yaw()
    );
}


float mpu6050_get_gyro_x(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_data.gx;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


float mpu6050_get_gyro_y(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_data.gy;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


float mpu6050_get_gyro_z(void)
{
    float value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_data.gz;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


/* ============================================================
 * 37. Reset
 * ============================================================ */

void mpu6050_reset_yaw(void)
{
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_attitude.yaw =
        0.0f;

    /*
     * Start the next trapezoidal interval from the current filtered
     * Z-rate, avoiding any contribution from a pre-reset history value.
     */
    g_mpu6050_prev_yaw_rate_dps =
        g_mpu6050_data.gz;

    g_mpu6050_yaw_rate_initialized =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


void mpu6050_reset_attitude(void)
{
    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_attitude =
        (mpu6050_attitude_t){0};

    g_mpu6050_attitude_initialized =
        false;

    g_mpu6050_filter_initialized =
        false;

    g_mpu6050_yaw_rate_initialized =
        false;

    g_mpu6050_prev_yaw_rate_dps =
        0.0f;

    g_mpu6050_timebase_reset_requested =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


/* ============================================================
 * 38. Calibration API
 * ============================================================ */

esp_err_t mpu6050_calibrate_gyro(
    uint16_t samples
)
{
    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskGetCurrentTaskHandle() ==
        g_mpu6050_task_handle)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    ret =
        mpu6050_calibrate_internal(
            samples,
            false
        );

    mpu6050_control_give();

    return ret;
}


esp_err_t mpu6050_calibrate_level(
    uint16_t samples
)
{
    esp_err_t ret =
        mpu6050_control_take();

    if (ret != ESP_OK)
    {
        return ret;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    if (xTaskGetCurrentTaskHandle() ==
        g_mpu6050_task_handle)
    {
        mpu6050_control_give();
        return ESP_ERR_INVALID_STATE;
    }

    ret =
        mpu6050_calibrate_internal(
            samples,
            true
        );

    mpu6050_control_give();

    return ret;
}


/* ============================================================
 * 39. Stationary / Pause / Resume
 * ============================================================ */

bool mpu6050_is_stationary(void)
{
    bool value;

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    value =
        g_mpu6050_stationary;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    return value;
}


void mpu6050_pause(void)
{
    if (mpu6050_control_take() !=
        ESP_OK)
    {
        return;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return;
    }

    bool timer_was_active = false;

    esp_err_t ret =
        mpu6050_pause_sampler_sync(
            &timer_was_active
        );

    (void)timer_was_active;

    if (ret != ESP_OK)
    {
        ESP_LOGE(
            MPU6050_TAG,
            "Pause failed: %s",
            esp_err_to_name(ret)
        );
    }

    mpu6050_control_give();
}


void mpu6050_resume(void)
{
    if (mpu6050_control_take() !=
        ESP_OK)
    {
        return;
    }

    if (!g_mpu6050_initialized)
    {
        mpu6050_control_give();
        return;
    }

    if (g_mpu6050_sample_timer == NULL)
    {
        mpu6050_control_give();
        return;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_pause_sampling =
        false;

    g_mpu6050_timebase_reset_requested =
        true;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );

    if (!esp_timer_is_active(
            g_mpu6050_sample_timer
        ))
    {
        esp_err_t ret =
            esp_timer_start_periodic(
                g_mpu6050_sample_timer,
                MPU6050_SAMPLE_PERIOD_US
            );

        if (ret != ESP_OK)
        {
            taskENTER_CRITICAL(
                &g_mpu6050_lock
            );

            g_mpu6050_pause_sampling =
                true;

            taskEXIT_CRITICAL(
                &g_mpu6050_lock
            );

            ESP_LOGE(
                MPU6050_TAG,
                "Resume: timer start failed: %s",
                esp_err_to_name(ret)
            );
        }
    }

    mpu6050_control_give();
}


/* ============================================================
 * 40. Debug Config
 * ============================================================ */

void mpu6050_set_debug(
    bool enable,
    uint32_t period_ms
)
{
    if (period_ms < 50U)
    {
        period_ms =
            50U;
    }

    taskENTER_CRITICAL(
        &g_mpu6050_lock
    );

    g_mpu6050_debug_enable =
        enable;

    g_mpu6050_debug_period_ms =
        period_ms;

    g_mpu6050_debug_last_print_us =
        0;

    taskEXIT_CRITICAL(
        &g_mpu6050_lock
    );
}


#endif /* MPU6050_IMPLEMENTATION */

#endif /* MPU6050_H */