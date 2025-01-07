/*
* Change Logs:
* Date            Author          Notes
* 2023-09-05      ChuShicheng     first version
*/

#include "example_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"

#define DBG_TAG   "rm.task"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

// static struct chassis_controller_t{
//     pid_obj_t *speed_pid;
// }chassis_controller[4];

static struct gimbal_controller_t{
    pid_obj_t *speed_pid;
    pid_obj_t *angle_pid;
}gimbal_controlelr[4];
static dji_motor_object_t *shoot_motor1;
static dji_motor_object_t *shoot_motor2;
static dji_motor_object_t *shoot_motor3;
static dji_motor_object_t *shoot_motor4;
static float shoot_motor_ref[4];
static float reflection;

static rt_int16_t shoot_control1(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[0].speed_pid, measure.speed_rpm,shoot_motor_ref[0] );
    return set;
}
static rt_int16_t shoot_control2(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[1].speed_pid, measure.speed_rpm, shoot_motor_ref[1]);
    return set;

}
static rt_int16_t shoot_control3(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[2].speed_pid, measure.speed_rpm, shoot_motor_ref[2]);
    return set;
}

static rt_int16_t shoot_control4(dji_motor_measure_t measure){
    static rt_int16_t set = 0;
    set = pid_calculate(gimbal_controlelr[3].speed_pid, measure.speed_rpm, shoot_motor_ref[3]);
    return set;
}

// static rt_int16_t gimbal_control(dji_motor_measure_t measure){
//     static rt_int16_t set = 0;
//     set = pid_calculate(gimbal_controlelr.speed_pid, measure.speed_rpm, 0);
//     return set;
// }

static void example_init()
{
    pid_config_t shoot1_speed_config = {
            .Kp = 10, // 4.5
            .Ki = 0,  // 0
            .Kd = 0,  // 0
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 12000,
    };
    pid_config_t shoot2_speed_config = {
        .Kp = 10, // 4.5
        .Ki = 0,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
};
    pid_config_t shoot3_speed_config = {
        .Kp = 10, // 4.5
        .Ki = 0,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
};
    pid_config_t shoot4_speed_config = {
        .Kp = 10, // 4.5
        .Ki = 0,  // 0
        .Kd = 0,  // 0
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .MaxOut = 12000,
};
    // pid_config_t gimbal_speed_config = {
    //         .Kp = 50,  // 50
    //         .Ki = 200, // 200
    //         .Kd = 0,
    //         .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //         .IntegralLimit = 3000,
    //         .MaxOut = 20000,
    // };
    gimbal_controlelr[0].speed_pid = pid_register(&shoot1_speed_config);
    gimbal_controlelr[1].speed_pid = pid_register(&shoot2_speed_config);
    gimbal_controlelr[2].speed_pid = pid_register(&shoot3_speed_config);
    gimbal_controlelr[3].speed_pid = pid_register(&shoot4_speed_config);
    // gimbal_controlelr.speed_pid = pid_register(&gimbal_speed_config);

    motor_config_t shoot1_motor_config = {
            .motor_type = M3508,
            .can_name = CAN_CHASSIS,
            .rx_id = 0x201,
            .controller = &gimbal_controlelr[0],
    };
    motor_config_t shoot2_motor_config = {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x202,
        .controller = &gimbal_controlelr[1],
};
    motor_config_t shoot3_motor_config = {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x203,
        .controller = &gimbal_controlelr[2],
};
    motor_config_t shoot4_motor_config = {
        .motor_type = M3508,
        .can_name = CAN_CHASSIS,
        .rx_id = 0x204,
        .controller = &gimbal_controlelr[3],
};
    // motor_config_t gimbal_motor_config = {
    //         .motor_type = GM6020,
    //         .can_name = CAN_GIMBAL,
    //         .rx_id = 0x206,
    //         .controller = &gimbal_controlelr,
    // };
    shoot_motor1 = dji_motor_register(&shoot1_motor_config, shoot_control1);
    shoot_motor2 = dji_motor_register(&shoot2_motor_config, shoot_control2);
    shoot_motor3 = dji_motor_register(&shoot3_motor_config, shoot_control3);
    shoot_motor4 = dji_motor_register(&shoot4_motor_config, shoot_control4);
    // gimbal_motor = dji_motor_register(&gimbal_motor_config, gimbal_control);
}

void example_thread_entry(void *argument)
{
    static float example_dt;
    static float example_start;


    example_init();
    LOG_I("Example Task Start");
    for (;;)
    {
        example_start = dwt_get_time_ms();
        // for(int i=0;i<4;i++) {
        //     shoot_motor_ref[i]=reflection;
        // }
        reflection= 9000;
        shoot_motor_ref[0]=-9000;
        shoot_motor_ref[1]=9000;
        shoot_motor_ref[2]=-8000;
        shoot_motor_ref[3]=8000;



        /* 鐢ㄤ簬璋冭瘯鐩戞祴绾跨▼璋冨害浣跨敤 */
        example_dt = dwt_get_time_ms() - example_start;
        if (example_dt > 1)
            LOG_E("Example Task is being DELAY! dt = [%f]", &example_dt);


        rt_thread_delay(1);
    }
}