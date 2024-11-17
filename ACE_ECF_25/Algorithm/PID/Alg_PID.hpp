#pragma once

#ifndef __PID_ALG_HPP
#define __PID_ALG_HPP

#include "filter.hpp"
#include <cstdint>

namespace Algorithm
{
    // PID 枚举结构体
    enum PID_mode_e
    {
        NONE = 0X00,                     // 0000 0000 无
        Deadzone = 0x01,                 // 0000 0001 死区
        Integral_Limit = 0x02,           // 0000 0010 积分限幅
        Output_Limit = 0x04,             // 0000 0100 输出限幅
        Derivative_On_Measurement = 0x08,// 0000 1000 微分先行 TODO:
        Separated_Integral = 0x10,       // 0001 0000 积分分离
        ChangingIntegrationRate = 0x20,  // 0010 0000 变积分
        OutputFilter = 0x40,             // 0100 0000 输出滤波
        DerivativeFilter = 0x80,         // 1000 0000 微分滤波
        StepIn = 0x0100,                 // 0000 0001 0000 0000 步进式
        Trapezoid_integral = 0x0200,     // 0000 0010 0000 0000 梯形积分
        Feedforward = 0x0400,            // 0000 0100 0000 0000 前馈PID
    };

    //PID初始化结构体
    struct PID_Init_Config_t
    {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;

        //前馈PID系数
        float Kfa = 0;
        float Kfb = 0;

        float *ActualValueSource = nullptr;//数据来源

        uint32_t mode = 0;// pid模式
        /* 输出限幅 */
        float max_out = 0;
        /* 积分限幅 */
        float max_Ierror = 0;//最大积分输出
        /* 微分先行 */
        float gama = 0;// 微分先行滤波系数
        /* 误差死区 */
        float deadband = 0;
        /* 积分分离 */
        float threshold_max = 0;//积分分离最大值
        float threshold_min = 0;//积分分离最小值
        /* 变积分 */
        float errorabsmax = 0;//偏差绝对值最大值
        float errorabsmin = 0;//偏差绝对值最小值
        /* 微分滤波 */
        float d_filter_num = 0;
        /* 输出滤波 */
        float out_filter_num = 0;
        /* 步进数 */
        float stepIn = 0;
    };

    // pid结构体变量
    struct pid_parameter_t
    {
        float Kp = 0;
        float Ki = 0;
        float Kd = 0;

        //前馈PID系数
        float Kfa = 0;
        float Kfb = 0;

        float SetValue = 0;//设定值
        float LastSetValue = 0;
        float PerrSetValue = 0;
        float ActualValue = 0;//实际值
        float LastActualValue = 0;

        float Ierror = 0;//误差积累
        float Pout = 0;
        float Iout = 0;
        float Dout = 0;
        float Fout = 0;
        float out = 0;

        float Derror = 0;//微分项
        float LastDerror = 0;
        float LastLastDerror = 0;
        float error = 0;//误差项
        float LastError = 0;

        float max_out = 0;//最大输出

        uint32_t mode = 0;// pid模式

        /* 积分限幅 */
        float max_Ierror = 0;//最大积分输出
        /* 误差死区 */
        float deadband = 0;
        /* 积分分离 */
        float threshold_max = 0;//积分分离最大值
        float threshold_min = 0;//积分分离最小值
        /* 抗积分饱和 */
        // float maximum = 0; //最大值
        // float minimum = 0; //最小值
        /* 变积分 */
        float errorabsmax = 0;//偏差绝对值最大值
        float errorabsmin = 0;//偏差绝对值最小值
        /* 微分先行 */
        float gama = 0;// 微分先行滤波系数

        /* 步进数 */
        float stepIn = 0;
    };

    // pid算法类
    class pid_alg_c
    {

    public:
        pid_alg_c(PID_Init_Config_t pid_config);
        pid_alg_c();
        void pid_init(PID_Init_Config_t pid_config);
        void ECF_PID_CLEAR(void);
        float ECF_PID_Calculate(float SetValuew);
        void ECF_PID_ChangeActValSource(float *ActValSource);


    private:
        /* 微分滤波 */
        first_order_filter_c d_filter;//微分滤波结构体
        /* 输出滤波 */
        first_order_filter_c out_filter; //输出滤波结构体
        pid_parameter_t pid_measure_;
        float *ActualValueSource_;   // pid 运算实际值来源
        inline void f_Separated_Integral(void);
        inline void f_Integral_Limit(void);
        inline void f_Derivative_On_Measurement(void);
        inline float Changing_Integration_Rate(void);
        inline void f_Output_Limit(void);
        inline void f_StepIn(void);
        inline void f_feedforward(void);
    };

};

#endif
