/**
 * @file Alg_PID.cpp
 * @brief PID控制算法
 *
 * @version 2.1         
 * @date 2024-9-16  1.0 加入了梯形积分
 *       2024-10-3  1.1 实际使用发现还是使用初始化函数方便一点
 *       2024-10-22 2.0 并未发现野指针的问题（我觉得没有），修正了变积分和微分先行算法
 *       2024-10-28 2.1 加入前馈PID控制和无参构造函数
 * @example
 * 1.引用命名空间后，需要创建一个结构体 //PID_Init_Config_t pid_cof
 * 2.为其赋值，然后可以选择使用构造函数或者初始化函数对其初始化
 * 3.之后就可以直接调用计算方法ECF_PID_Calculate
 * 例程如下：
 *
    pid_alg_n::PID_Init_Config_t pid_cof = {
    .Kp = 1,
    .Ki = 1,
    .Kd = 0.5,
    .Kfa = 0,
    .Kfb = 0,
    .ActualValueSource = ptr,
    .mode = Deadzone | Integral_Limit | Derivative_On_Measurement | DerivativeFilter,
    .max_out = 2000,
    .max_Ierror = 1000,
    .gama = 0.1,
    .deadband = 10,
    .threshold_max = 2000,
    .threshold_min = -2000,
    .errorabsmax = 100,
    .errorabsmin = -100,
    .d_filter_num = 5,
    .out_filter_num = 10,
    .stepIn = 100
    };
    //不需要全部设置，根据所需设定需要的数值即可

    pid_alg_n::pid_alg_c pid_instance(pid_cof);
    pid_instance.ECF_PID_Calculate(SetValue); // 计算
    if (error || chance)
        pid_instance.ECF_PID_CLEAR();             //清除
 *
 */


#include "Alg_PID.hpp"
#include "user_maths.hpp"

#include <cstring>

namespace Algorithm
{
    /**
     * @brief          PID构造函数
     * @param[in]      pid_config pid 初始化参数结构体
     * @note 如想使用多个模式可以使用 ' | ',
     * @note 比如使用 积分分离 和 输出滤波 则可以在 mode 入参 (OutputFilter | Separated_Integral)
     */
    pid_alg_c::pid_alg_c(PID_Init_Config_t pid_config)
        : d_filter(pid_config.d_filter_num),
          out_filter(pid_config.out_filter_num)
    {
        if (pid_config.ActualValueSource != nullptr)
        {
            this->ActualValueSource_ = pid_config.ActualValueSource;
        }
        else
        {
            this->ActualValueSource_ = nullptr;
        }
        this->pid_measure_.Kp = pid_config.Kp;
        this->pid_measure_.Ki = pid_config.Ki;
        this->pid_measure_.Kd = pid_config.Kd;
        this->pid_measure_.Kfa = pid_config.Kfa;
        this->pid_measure_.Kfb = pid_config.Kfb;
        this->pid_measure_.mode = pid_config.mode;

        this->pid_measure_.max_Ierror = pid_config.max_Ierror;

        this->pid_measure_.gama = pid_config.gama;

        this->pid_measure_.threshold_max = pid_config.threshold_max;
        this->pid_measure_.threshold_min = pid_config.errorabsmin;

        this->pid_measure_.max_out = pid_config.max_out;

        this->pid_measure_.errorabsmax = pid_config.errorabsmax;
        this->pid_measure_.errorabsmin = pid_config.errorabsmin;


        this->pid_measure_.deadband = pid_config.deadband;

        this->pid_measure_.stepIn = pid_config.stepIn;
        this->ECF_PID_CLEAR();
    }

    //无参构造函数
    pid_alg_c::pid_alg_c()
        : d_filter(0),
          out_filter(0) {}

    /**
     * @brief          PID初始化函数,如果使用了构造就不需要这个函数了，这里主要用于初始化内部类
     * @param[in]      pid_config pid 初始化参数结构体
     * @note 如想使用多个模式可以使用 ' | ',
     * @note 比如使用 积分分离 和 输出滤波 则可以在 mode 入参 (OutputFilter | Separated_Integral)
     */
    void pid_alg_c::pid_init(PID_Init_Config_t pid_config)
    {
        // memset(&this->pid_measure_, 0, sizeof(pid_parameter_t));
        this->ActualValueSource_ = pid_config.ActualValueSource;
        this->pid_measure_.Kp = pid_config.Kp;
        this->pid_measure_.Ki = pid_config.Ki;
        this->pid_measure_.Kd = pid_config.Kd;
        this->pid_measure_.Kfa = pid_config.Kfa;
        this->pid_measure_.Kfb = pid_config.Kfb;
        this->pid_measure_.mode = pid_config.mode;

        this->pid_measure_.max_Ierror = pid_config.max_Ierror;

        this->pid_measure_.gama = pid_config.gama;

        this->pid_measure_.threshold_max = pid_config.threshold_max;
        this->pid_measure_.threshold_min = pid_config.errorabsmin;

        this->pid_measure_.max_out = pid_config.max_out;

        this->out_filter.first_order_filter_init(pid_config.out_filter_num);

        this->pid_measure_.errorabsmax = pid_config.errorabsmax;
        this->pid_measure_.errorabsmin = pid_config.errorabsmin;

        this->d_filter.first_order_filter_init(pid_config.d_filter_num);

        this->pid_measure_.deadband = pid_config.deadband;

        this->pid_measure_.stepIn = pid_config.stepIn;
        this->ECF_PID_CLEAR();
    }

    /**
     * @brief 更改闭环的实际值来源
     * @param ActValSource
    */
    void pid_alg_c::ECF_PID_ChangeActValSource(float *ActValSource)
    {
        this->ActualValueSource_ = ActValSource;
    }


    /**
     * @brief          PID清除
     * @retval         none
     * @attention      只是清除所有计算的数据，不会清除pid或者模式的数据
     */
    void pid_alg_c::ECF_PID_CLEAR(void)
    {
        this->pid_measure_.error = this->pid_measure_.LastError = 0.0f;
        this->pid_measure_.Derror = this->pid_measure_.LastDerror = this->pid_measure_.LastLastDerror = 0.0f;
        this->pid_measure_.out = this->pid_measure_.Pout = this->pid_measure_.Iout = this->pid_measure_.Dout = this->pid_measure_.Ierror = this->pid_measure_.Fout = 0.0f;
        this->pid_measure_.ActualValue = this->pid_measure_.SetValue = this->pid_measure_.LastActualValue = this->pid_measure_.LastSetValue = this->pid_measure_.PerrSetValue = 0.0f;
        this->d_filter.first_order_filter_clear();
        this->out_filter.first_order_filter_clear();
    }

    /**
     * @brief          PID通用计算
     * @param[in]      设定参考值
     * @retval         none
     */
    float pid_alg_c::ECF_PID_Calculate(float SetValuew)
    {
        float thiserr = 0;//积分增值

        this->pid_measure_.SetValue = SetValuew;

        // 步进式pid
        if (this->pid_measure_.mode & StepIn)
            this->f_StepIn();

        this->pid_measure_.ActualValue = *this->ActualValueSource_;

        this->pid_measure_.error = this->pid_measure_.SetValue - this->pid_measure_.ActualValue;

        this->pid_measure_.Derror = this->pid_measure_.error - this->pid_measure_.LastError;
        if (UserMath::abs(this->pid_measure_.error) >= this->pid_measure_.deadband)//死区
        {
            this->pid_measure_.Pout = this->pid_measure_.error * this->pid_measure_.Kp;
            //梯形积分
            if (this->pid_measure_.mode & Trapezoid_integral)
                thiserr = (this->pid_measure_.error + this->pid_measure_.LastError) / 2;
            else
                thiserr = this->pid_measure_.error;

            // 微分先行
            if (this->pid_measure_.mode & Derivative_On_Measurement)
                this->f_Derivative_On_Measurement();
            else
                this->pid_measure_.Dout = this->pid_measure_.Kd * this->pid_measure_.Derror;

            // 变积分
            if (this->pid_measure_.mode & ChangingIntegrationRate)
                this->pid_measure_.Ierror += thiserr * this->Changing_Integration_Rate();
            else
                this->pid_measure_.Ierror += thiserr;


            // 积分限幅
            if (this->pid_measure_.mode & Integral_Limit)
                this->f_Integral_Limit();
            this->pid_measure_.Iout = this->pid_measure_.Ki * this->pid_measure_.Ierror;

            // 积分分离 注意需要放在iout计算后
            if (this->pid_measure_.mode & Separated_Integral)
                this->f_Separated_Integral();

            // 微分滤波
            if (this->pid_measure_.mode & DerivativeFilter)
                this->pid_measure_.Dout = this->d_filter.first_order_filter(this->pid_measure_.Dout);

            //前馈控制器
            if (this->pid_measure_.mode & Feedforward)
                this->f_feedforward();

            this->pid_measure_.out = this->pid_measure_.Pout + this->pid_measure_.Iout + this->pid_measure_.Dout + this->pid_measure_.Fout;

            // 输出滤波
            if (this->pid_measure_.mode & OutputFilter)
                this->pid_measure_.out = this->out_filter.first_order_filter(this->pid_measure_.out);

            // 输出限幅
            if (this->pid_measure_.mode & Output_Limit)
                this->f_Output_Limit();
        }
        else
        {
            this->ECF_PID_CLEAR();
        }

        //数据更新
        this->pid_measure_.LastActualValue = this->pid_measure_.ActualValue;
        this->pid_measure_.PerrSetValue = this->pid_measure_.LastSetValue;
        this->pid_measure_.LastSetValue = this->pid_measure_.SetValue;
        this->pid_measure_.LastDerror = this->pid_measure_.Derror;
        this->pid_measure_.LastError = this->pid_measure_.error;

        return this->pid_measure_.out;
    }


    /**
     * @brief          积分分离
     * @retval         none
     * @note           error值超过阈值的时候把iout清零就行
     * @note           当设定值和参考值差别过大时, 只使用 PD 控制
     */
    void pid_alg_c::f_Separated_Integral(void)
    {
        if (this->pid_measure_.threshold_min > this->pid_measure_.error && this->pid_measure_.error < this->pid_measure_.threshold_max)
            this->pid_measure_.Iout = 0;
    }


    /**
     * @brief          积分限幅
     * @retval         none
     * @note      防止积分误差值(Ierror)超过限幅
     */
    void pid_alg_c::f_Integral_Limit(void)
    {
        if (this->pid_measure_.Ierror > this->pid_measure_.max_Ierror)
        {
            this->pid_measure_.Ierror = this->pid_measure_.max_Ierror;
        }
        if (this->pid_measure_.Ierror < -(this->pid_measure_.max_Ierror))
        {
            this->pid_measure_.Ierror = -(this->pid_measure_.max_Ierror);
        }
    }

    /**
     * @brief          微分先行
     * @retval         none
     * @attention      //似乎存在问题 详见 https://blog.csdn.net/foxclever/article/details/80633275
     *                 问题似乎是原文c3符号的问题，评论区有争议，查阅其他资料是取负的，可参考：
     *https://wenku.baidu.com/view/17078d1bb91aa8114431b90d6c85ec3a87c28bbb?aggId=820bd2e1581b6bd97e19eacf&fr=catalogMain_text_ernie_recall_v1%3Awk_recommend_main1&_wkts_=1729588237892&bdQuery=%E5%BE%AE%E5%88%86%E5%85%88%E8%A1%8Cpid
     */
    void pid_alg_c::f_Derivative_On_Measurement(void)
    {
        float c1, c2, c3, temp;

        temp = this->pid_measure_.gama * this->pid_measure_.Kd + this->pid_measure_.Kp;
        c3 = this->pid_measure_.Kd / temp;
        c2 = (this->pid_measure_.Kd + this->pid_measure_.Kp) / temp;
        c1 = this->pid_measure_.gama * c3;
        this->pid_measure_.Dout = c1 * this->pid_measure_.Dout + c2 * this->pid_measure_.ActualValue - c3 * this->pid_measure_.LastActualValue;
    }


    /**
     * @brief           变积分系数处理函数，实现一个输出0和1之间的分段线性函数
     * @param[in]       pid结构体
     * @retval          积分增值系数
     * @note            当偏差的绝对值小于最小值时，输出为1；当偏差的绝对值大于最大值时，输出为0
     * @note            当偏差的绝对值介于最大值和最小值之间时，输出在0和1之间线性变化
     * @note            系统偏差大时，对积分作用减弱甚至是全无，而在偏差小时，加强积分的作用
     */
    float pid_alg_c::Changing_Integration_Rate(void)
    {
        if (UserMath::abs(this->pid_measure_.error) <= this->pid_measure_.errorabsmin)//最小值
        {
            return 1.0f;
        }
        else if (UserMath::abs(this->pid_measure_.error) > this->pid_measure_.errorabsmax)//最大值
        {
            return 0.0f;
        }
        else
        {
            return (this->pid_measure_.errorabsmax - UserMath::abs(this->pid_measure_.error)) / (this->pid_measure_.errorabsmax - this->pid_measure_.errorabsmin);
        }
    }


    /**
     * @brief          输出限幅
     * @param[in]      pid结构体
     * @retval         none
     * @attention      none
     */
    void pid_alg_c::f_Output_Limit(void)
    {
        if (this->pid_measure_.out > this->pid_measure_.max_out)
        {
            this->pid_measure_.out = this->pid_measure_.max_out;
        }
        if (this->pid_measure_.out < -(this->pid_measure_.max_out))
        {
            this->pid_measure_.out = -(this->pid_measure_.max_out);
        }
    }


    /**
     * @brief           步进式pid
     * @retval          none
     * @attention       详见 https://blog.csdn.net/foxclever/article/details/81151898
     * @note            固定每周期的设定值变化值
     */
    void pid_alg_c::f_StepIn(void)
    {
        if (UserMath::abs(this->pid_measure_.LastSetValue - this->pid_measure_.SetValue) <= this->pid_measure_.stepIn)
        {
            return;
        }
        float kFactor = 0.0f;
        if ((this->pid_measure_.LastSetValue - this->pid_measure_.SetValue) > 0.0f)
        {
            kFactor = -1.0f;
        }
        else if ((this->pid_measure_.LastSetValue - this->pid_measure_.SetValue) < 0.0f)
        {
            kFactor = 1.0f;
        }
        else
        {
            kFactor = 0.0f;
        }
        this->pid_measure_.SetValue = this->pid_measure_.LastSetValue + kFactor * this->pid_measure_.stepIn;

    }


    /**
     * @brief           前馈控制器
     * @param           SetValuew 当前设定值
     * @note            原理：求导数对未来状态进行预测
     */
    void pid_alg_c::f_feedforward(void)
    {
        this->pid_measure_.Fout = this->pid_measure_.Kfa * (this->pid_measure_.SetValue - this->pid_measure_.LastSetValue) +
                                  this->pid_measure_.Kfb * (this->pid_measure_.SetValue - 2 * this->pid_measure_.LastSetValue + this->pid_measure_.PerrSetValue);
    }
}