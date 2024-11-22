#pragma once

#ifndef __DJI_MOTOR_HPP
#define __DJI_MOTOR_HPP

#include "Motor_General_def.hpp"
#include "bsp_can.hpp"

#define DJI_MOTOR_CNT 12

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f     // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f    // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f// (360/8192),将编码器值转化为角度制

namespace Motor
{
    /* DJI电机CAN反馈信息*/
    struct DJIMotorMeasure_t
    {
        uint16_t feedback_ecd;        // 反馈数据, 电机末端单圈编码, 0-8191,刻度总共有8192格
        float feedback_speed;         // 反馈数据, 电机末端实时转速, 单位rpm
        int16_t feedback_real_current;// 反馈数据, 实际电流
        uint8_t feedback_temperature; // 反馈数据, 温度 Celsius

        bool is_first;// 判断是否为首次接收到数据

        uint16_t first_ecd;      // 首次接收到的编码值
        uint16_t last_ecd;       // 上一次读取的编码器值
        uint16_t last_record_ecd;// 上一次总累计编码值
        float angle_single_round;// 单圈角度
        float total_angle;       // 总角度,注意方向
        float total_round;       // 总圈数,注意方向
        int32_t record_ecd;      // 总累计编码值, 注意方向
        float record_length;     // 输出轴总累计长度, 注意方向, 单位为: 毫米

        float speed_aps;    // 根据反馈值计算的角速度,单位为: 度/秒
        float angular_speed;// 输出轴角速度,单位为: 度/秒
        float linear_speed; // 输出轴线速度,单位为: 米/秒
    };

    // dji电机堵转初始化结构体
    struct DJIMotorBlockInit_t
    {
        uint16_t block_times;
        bool block_init_if_finish;
        float last_position;
        int16_t init_current;// 朝某个方向初始化的电流值
    };

    /**
     * @brief DJI电机返回信息处理类
    */
    class DJIMotorMeasure_c
    {
    public:
        DJIMotorMeasure_t measure;
        float ecd2length;   // 编码值转长度, 单位为: 编码/毫米
        float radius;       // 电机输出轴连接轮子半径, 单位: 毫米;
        int16_t gear_Ratio; // 减速比
        int16_t lap_encoder;// 编码器单圈码盘值（8192=12bit）

        void init(float ecd2length, float radius, Motor_Type_e motor_type);

        void reset();
    };

    void DJIMotorTick();

    void DebugDecodeDJIMotor(void *data, uint8_t length, void *private_data);

    /**
     * @brief DJI电机发送信息类
    */
    class DJI_Motor_Instance
    {
    private:
        float set_length;// 设定的长度值
        float set_speed; // 设定的速度值

        static BSP::CANInstance_c sender_assignment[6];

        // 发送的数据
        uint8_t *send_buffer;

        // 计算反馈时间间隔需要的变量，使用DebugDecodeDJIMotor作为接收函数的时候有效
        uint64_t last_feedback_timestamp;// 上一次获得反馈的时间戳
        float dt;                        // 反馈时间间隔, 单位秒

        float *other_speed_feedback_ptr_ = nullptr;
        float *other_angle_feedback_ptr_ = nullptr;// 其他反馈来源的反馈数据指针
        float *speed_feedforward_ptr_ = nullptr;   // 速度前馈数据指针,可以通过此指针设置速度前馈值,或LQR等时作为速度状态变量的输入
        float *current_feedforward_ptr_ = nullptr; // 电流前馈指针

        DJIMotorBlockInit_t block_val;
        DJIMotorMeasure_c MotorMeasure;
        Motor_Controller_c motor_controller;
        BSP::CANInstance_c motor_can_instance;// 电机CAN实例
        Motor_Control_Setting_s motor_settings;
        Motor_State_e state;    // 启停标志
        Motor_Type_e motor_type;// 电机类型

    private:
        // 静态变量
        static Util::InstanceManger<DJI_Motor_Instance, DJI_MOTOR_CNT> instances;
        static bool sender_enable_flag[6];

    public:
        explicit DJI_Motor_Instance(Motor_Init_Config_s config);

    private:
        void InitMotorCan(BSP::CAN_Init_Config_s &can_init_config);
        void DJIMotorBlockInitAchieve();
        bool DJIMotorBlockInit(int16_t init_current);
        void tick();
        void set_current(int16_t give_current) const;

    public:
        void setMotorState(Motor_State_e state);
        void DJIMotorSetRef(float ref);

        friend void DJIMotorTick();
        friend void DebugDecodeDJIMotor(void *data, uint8_t length, void *private_data);
    };
}

#endif // !__DJI_MOTOR_HPP
