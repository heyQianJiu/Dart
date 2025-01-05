#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "rm_task.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#define HWTIMER_DEV_NAME   "timer4"     /* 定时器名称 */
#include <rtdbg.h>

/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
static struct chassis_cmd_msg chassis_cmd;
static struct chassis_fdb_msg chassis_fdb;
// static struct ins_msg ins_data;

static publisher_t *pub_chassis;
static subscriber_t *sub_cmd;
// static subscriber_t *sub_ins;
static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */
// static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
// static pid_config_t chassis_follow_config = INIT_PID_CONFIG(CHASSIS_KP_V_FOLLOW, CHASSIS_KI_V_FOLLOW, CHASSIS_KD_V_FOLLOW, CHASSIS_INTEGRAL_V_FOLLOW, CHASSIS_MAX_V_FOLLOW,
//                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid;//大致调角度的话不用角度环，细调要用
}chassis_controller[2];

static dji_motor_object_t *chassis_motor[2];  // 底盘电机实例
static int16_t motor_ref[2]; // yaw pitch 电机控制期望值

static void chassis_motor_init();
/*定时器初始化*/
static int TIM_Init(void);
/*角度数据*/
static float pitch_degree,yaw_degree;


// static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
// static struct chassis_real_speed_t
// {
//     float chassis_vx_ch;
//     float chassis_vy_ch;
//     // float chassis_vw_ch;
// }chassis_real_speed;

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float cmd_dt;
void chassis_thread_entry(void *argument)
{
    static float cmd_start;

    chassis_pub_init();
    chassis_sub_init();
    chassis_motor_init();
    TIM_Init();

    LOG_I("Chassis Task Start");
    for (;;)
    {
        cmd_start = dwt_get_time_ms();

        /* 更新该线程所有的订阅者 */
        chassis_sub_pull();

        for (uint8_t i = 0; i < 2; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }

        switch (chassis_cmd.ctrl_mode)
        {
            case CHASSIS_RELAX:
                for (uint8_t i = 0; i < 2; i++)
                {
                    dji_motor_relax(chassis_motor[i]);
                }
            break;
            case CHASSIS_OPEN_LOOP://速度环控制的
                motor_ref[YAW_MOTOR] = chassis_cmd.vw_yaw;
                motor_ref[PITCH_MOTOR] = chassis_cmd.vw_pitch;
                break;
            case CHASSIS_INIT://角度环控制的
                motor_ref[YAW_MOTOR] = CENTER_ECD_YAW;
                motor_ref[PITCH_MOTOR] =PITCH_INIT_DEGREE;
                if(chassis_motor[YAW_MOTOR]->measure.ecd - CENTER_ECD_YAW <= 20) {
                    chassis_fdb.back_mode = BACK_IS_OK;
                }else {
                    chassis_fdb.back_mode = BACK_STEP;
                }

            default:
                for (uint8_t i = 0; i < 2; i++)
                {
                    dji_motor_relax(chassis_motor[i]);
                }
            break;
        }

        /* 更新发布该线程的msg */
        chassis_pub_push();

        /* 用于调试监测线程调度使用 */
        cmd_dt = dwt_get_time_ms() - cmd_start;
        if (cmd_dt > 1)
            LOG_E("Chassis Task is being DELAY! dt = [%f]", &cmd_dt);

        rt_thread_delay(1);
    }
}
/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    pub_push_msg(pub_chassis,&chassis_fdb);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    sub_get_msg(sub_cmd, &chassis_cmd);
}
/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static rt_int16_t motor_control_yaw(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(rt_int16_t) pid_calculate(chassis_controller[YAW_MOTOR].speed_pid, measure.speed_rpm, motor_ref[YAW_MOTOR]);
    return set;
}

static rt_int16_t motor_control_pitch(dji_motor_measure_t measure)
{
    static rt_int16_t set = 0;
    set =(rt_int16_t) pid_calculate(chassis_controller[PITCH_MOTOR].speed_pid, measure.speed_rpm, motor_ref[PITCH_MOTOR]);
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *motor_control[2] =
{
    motor_control_yaw,
    motor_control_pitch,
};
motor_config_t chassis_motor_config[2] =
    {
        {
            .motor_type = M3508,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x201,
            .controller = &chassis_controller[YAW_MOTOR],
        },
    {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x202,
        .controller = &chassis_controller[PITCH_MOTOR],
        }
    };
/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{   //速度环
    pid_config_t yaw_speed_config = INIT_PID_CONFIG(YAW_KP_V, YAW_KI_V, YAW_KD_V, YAW_INTEGRAL_V, YAW_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[YAW_MOTOR].speed_pid = pid_register(&yaw_speed_config);
    pid_config_t pitch_speed_config = INIT_PID_CONFIG(PITCH_KP_V, PITCH_KI_V, PITCH_KD_V, PITCH_INTEGRAL_V, PITCH_MAX_V,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[PITCH_MOTOR].speed_pid = pid_register(&pitch_speed_config);
    chassis_motor[YAW_MOTOR] = dji_motor_register(&chassis_motor_config[YAW_MOTOR], motor_control[YAW_MOTOR]);
    chassis_motor[PITCH_MOTOR] = dji_motor_register(&chassis_motor_config[YAW_MOTOR], motor_control[PITCH_MOTOR]);
    //角度环
    pid_config_t yaw_angle_config = INIT_PID_CONFIG(YAW_KP_A,YAW_KI_A,YAW_KD_A,YAW_INTEGRAL_A,YAW_MAX_A,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[YAW_MOTOR].angle_pid = pid_register(&yaw_angle_config);
    pid_config_t pitch_angle_config = INIT_PID_CONFIG(PITCH_KP_A,PITCH_KI_A,PITCH_KD_A,PITCH_INTEGRAL_A,PITCH_MAX_A,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    chassis_controller[PITCH_MOTOR].angle_pid = pid_register(&pitch_angle_config);
}
static rt_err_t timeout_cb(rt_device_t dev, rt_size_t size)
{
    //这里应该能直接用measure的编码器值解算
    // yaw_degree = (chassis_motor[YAW_MOTOR]->measure.total_angle - CENTER_ECD_YAW ) * TRIGGER_MOTOR_45_TO_ANGLE;
    // chassis_fdb.yaw_degree = yaw_degree;
    // chassis_fdb.pitch_degree = pitch_degree;

    // chassis_motor[PITCH_MOTOR]->measure.total_angle;
    return 0;
}
int TIM_Init(void)
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeout_s;      /* 定时器超时值 */
    rt_device_t hw_dev = RT_NULL;   /* 定时器设备句柄 */
    rt_hwtimer_mode_t mode;         /* 定时器模式 */
    rt_uint32_t freq = 10000;               /* 计数频率 */

    /* 查找定时器设备 */
    hw_dev = rt_device_find("timer4" );
    if (hw_dev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", HWTIMER_DEV_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(hw_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", HWTIMER_DEV_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(hw_dev, timeout_cb);

    rt_device_control(hw_dev, HWTIMER_CTRL_FREQ_SET, &freq);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(hw_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    timeout_s.sec = 0;      /* 秒 */
    timeout_s.usec = 1000;     /* 微秒 */
    if (rt_device_write(hw_dev, 0, &timeout_s, sizeof(timeout_s)) != sizeof(timeout_s))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }
}
