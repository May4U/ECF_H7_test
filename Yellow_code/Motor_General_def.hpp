/************************** Dongguan-University of Technology -ACE**************************
* @file  Motor_General_def.hpp
* @brief 电机的通用配置定义文件
* @author wuage2335俞永祺 
* @version 1.0
* @date 2022-11-24
*
* ==============================================================================
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/

#pragma once

#ifndef __MOTOR_GENERAL_DEF_HPP
#define __MOTOR_GENERAL_DEF_HPP

#include "Alg_PID.hpp"
#include "bsp_can.hpp"

namespace Motor
{
    /* 电机正反转标志 */
    enum Motor_Reverse_Flag_e
    {
        MOTOR_DIRECTION_NORMAL = 0,
        MOTOR_DIRECTION_REVERSE = 1
    };


    /* 反馈量正反标志 */
    enum Feedback_Reverse_Flag_e
    {
        FEEDBACK_DIRECTION_NORMAL = 0,
        FEEDBACK_DIRECTION_REVERSE = 1
    };

    enum Motor_State_e
    {
        MOTOR_STOP = 0,
        MOTOR_ENABLED = 1,
        MOTOR_LOCK = 2,
        MOTOR_INIT_BLOCK_MODE = 3// 表示此时正在初始化状态
    };

    /* 反馈来源设定,若设为OTHER_FEED则需要指定数据来源指针,详见Motor_Controller_s*/
    enum Feedback_Source_e
    {
        MOTOR_FEED = 0,
        OTHER_FEED,
    };
    // 闭环部分

    enum CloseLoop_Type_e
    {
        OPEN_LOOP,   // 开环
        CURRENT_LOOP,// 电流闭环
        SPEED_LOOP,  // 速度闭环
        ANGLE_LOOP,  // 角度闭环
    };

    struct CloseLoop_Config_t
    {
        bool is_current_loop = true;// 电流闭环
        bool is_speed_loop = true;  // 速度闭环
        bool is_angle_loop = true;  // 角度闭环
    };

    enum Feedfoward_Type_e
    {
        FEEDFORWARD_NONE = 0b00,
        CURRENT_FEEDFORWARD = 0b01,
        SPEED_FEEDFORWARD = 0b10,
        CURRENT_AND_SPEED_FEEDFORWARD = CURRENT_FEEDFORWARD | SPEED_FEEDFORWARD,
    };

    /* 电机控制设置,包括闭环类型,反转标志和反馈来源 */
    struct Motor_Control_Setting_s
    {
        CloseLoop_Type_e outer_loop_type;             // 最外层的闭环,未设置时默认为最高级的闭环
        CloseLoop_Config_t close_loop_config;         // 使用的闭环
        Motor_Reverse_Flag_e motor_reverse_flag;      // 是否反转
        Feedback_Reverse_Flag_e feedback_reverse_flag;// 反馈是否反向
        Feedback_Source_e angle_feedback_source;      // 角度反馈类型
        Feedback_Source_e speed_feedback_source;      // 速度反馈类型
        Feedfoward_Type_e feedforward_flag;           // 前馈标志
    };

    /* 电机类型枚举 */
    typedef enum
    {
        MOTOR_TYPE_NONE = 0,
        MOTOR_TYPE_GM6020,
        MOTOR_TYPE_M3508,
        MOTOR_TYPE_M2006,
        MOTOR_TYPE_LK9025,
        MOTOR_TYPE_HT04,
    } Motor_Type_e;

    /**
     * @brief 电机控制器初始化结构体,包括三环PID的配置以及两个反馈数据来源指针
     *        如果不需要某个控制环,可以不设置对应的pid config
     *        需要其他数据来源进行反馈闭环,不仅要设置这里的指针还需要在Motor_Control_Setting_s启用其他数据来源标志
     */
    struct Motor_Controller_Init_s
    {
        Algorithm::PID_Init_Config_t current_PID;
        Algorithm::PID_Init_Config_t speed_PID;
        Algorithm::PID_Init_Config_t angle_PID;
    };

    /* 用于初始化CAN电机的结构体,各类电机通用 */
    struct Motor_Init_Config_s
    {
        Motor_Controller_Init_s controller_param_init_config;
        Motor_Control_Setting_s controller_setting_init_config;
        Motor_Type_e motor_type;
        BSP::CAN_Init_Config_s can_init_config;
        float radius;       // 输出轴半径
        float ecd2length;
    } ;

    /* 电机控制器,包括其他来源的反馈数据指针,3环控制器和电机的参考输入*/
    // 后续增加前馈数据指针
    class Motor_Controller_c
    {
        private:
            float pid_ref = 0; // 将会作为每个环的输入和输出顺次通过串级闭环
        public:
            Algorithm::pid_alg_c current_PID;
            Algorithm::pid_alg_c speed_PID;
            Algorithm::pid_alg_c angle_PID;
            void RefValChange(float ref_val);
            float GetRefVal()const;
    };

}



#endif // !__MOTOR_GENERAL_DEF_HPP
