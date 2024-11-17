/*************************** Dongguan-University of Technology -ACE**************************
 * @file    dji_motor.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/10/18
 * @brief   电机模块文件
 ******************************************************************************
 * @verbatim
 * 支持PID控制 + 温度保护
 */
// demo:
//   // 初始化开始，关中断
//   __disable_irq();
//   Motor_Init_Config_s config = {
//       .controller_param_init_config = {
//         .speed_PID = {
//           .Kp = 2.0f,
//           .Ki = 0.0f,
//           .Kd = 0.0f,
//           .ActualValueSource = NULL,
//           .mode = Output_Limit,
//           .max_out = 9000,
//           .deadband = 0.3,
//         },
//       },
//       .controller_setting_init_config = {
//         .outer_loop_type       =  SPEED_LOOP,
//         .close_loop_type       =  SPEED_LOOP,
//         .motor_reverse_flag    =  MOTOR_DIRECTION_NORMAL,
//         .feedback_reverse_flag =  FEEDBACK_DIRECTION_NORMAL,
//         .speed_feedback_source =  MOTOR_FEED,
//         .feedforward_flag      =  FEEDFORWARD_NONE,
//       },
//       .motor_type = M2006,
//       .can_init_config = {
//         .can_handle = &hcan1,
//         .tx_id = 8,// 看电调闪几下就填几
//       }
//   };
//   DJI_Motor_Instance dji_motor1(config);
//   dji_motor1.DJIMotorEnable();
//   // 初始化完成,开启中断
//   __enable_irq();
//   dji_motor1.DJIMotorSetRef(400);
//   while(1)
//   {
//     DJIMotorControl();
//   }
/*
 * @attention
 *      无
 * @version           time
 * v1.0   基础版本（can）     2024-10-18    已测试
 ************************** Dongguan-University of Technology -ACE***************************/

#include "dji_motor.hpp"
#include "bsp_dwt.hpp"
#include "user_maths.hpp"
#include <cstring>

#ifdef USE_H7
#include <fdcan.h>
#else
#include <can.h>
#endif

#ifndef RPM_2_ANGLE_PER_SEC
#define RPM_2_ANGLE_PER_SEC 6.0f// ×360°/60sec
#endif

namespace Motor
{
    Util::InstanceManger<DJI_Motor_Instance, DJI_MOTOR_CNT> DJI_Motor_Instance::instances;

    /**
    * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2can*3group)can_instance专门负责发送
    *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
    *
    * @note  因为只用于发送,所以不需要在bsp_can中注册
    *
    * C610(m2006)/C620(m3508):0x1ff,0x200;
    * GM6020:0x1ff,0x2ff
    * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
    * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
    * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
    */

    /**
    * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
    *        flag的初始化在 MotorSenderGrouping()中进行
    */
    DJI_Motor_Instance::DJI_Motor_Instance(Motor_Init_Config_s config)
    {
        if (instances.size() == 0)
        {
#ifdef USE_H7
            CANHandle can1 = &hfdcan1;
            CANHandle can2 = &hfdcan2;
            uint32_t SAND_IDE = FDCAN_STANDARD_ID;
#else
            CANHandle can1 = &hcan1;
            CANHandle can2 = &hcan2;
            uint32_t SAND_IDE = CAN_ID_STD;
#endif
            sender_assignment[0].Init({.can_handle = can1, .tx_id = 0x1ff, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
            sender_assignment[1].Init({.can_handle = can1, .tx_id = 0x200, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
            sender_assignment[2].Init({.can_handle = can1, .tx_id = 0x2ff, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
            sender_assignment[3].Init({.can_handle = can2, .tx_id = 0x1ff, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
            sender_assignment[4].Init({.can_handle = can2, .tx_id = 0x200, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
            sender_assignment[5].Init({.can_handle = can2, .tx_id = 0x2ff, .rx_id = 0x000, .SAND_IDE = SAND_IDE});
        }

        // motor basic setting 电机基本设置
        this->InitMotorCan(config.can_init_config);
        this->motor_type = config.motor_type;                        // 6020 or 2006 or 3508
        this->motor_settings = config.controller_setting_init_config;// 正反转,闭环类型等
        this->motor_controller.angle_PID.pid_init(config.controller_param_init_config.angle_PID);
        this->motor_controller.speed_PID.pid_init(config.controller_param_init_config.speed_PID);
        this->motor_controller.current_PID.pid_init(config.controller_param_init_config.current_PID);
        this->MotorMeasure.radius = config.radius;
        this->MotorMeasure.ecd2length = config.ecd2length;
        switch (this->motor_type)
        {
            case MOTOR_TYPE_M3508:
            {
                this->MotorMeasure.lap_encoder = 8192;
                this->MotorMeasure.gear_Ratio = 19;
                break;
            }
            case MOTOR_TYPE_M2006:
            {
                this->MotorMeasure.lap_encoder = 8192;
                this->MotorMeasure.gear_Ratio = 36;
                break;
            }
            case MOTOR_TYPE_GM6020:
            {
                this->MotorMeasure.lap_encoder = 16384;
                this->MotorMeasure.gear_Ratio = 1;
                break;
            }
            default:
                break;
        }

        // 添加实例
        instances.add(this);

        this->DJIMotorStop();
        this->MotorMeasure.MeasureClear();
    }

    /***
     * @brief 设定闭环的参考值
     * @param ref 用作闭环计算大的参考值
     * @note ref 的值具体设置为什么看你在初始化设置的闭环环路, 以及最外环的数据来源 ActualValueSource
     */
    void DJI_Motor_Instance::DJIMotorSetRef(float ref)
    {
        if (this->state == MOTOR_Lock || this->state == MOTOR_STOP)
        {
            return;
        }
        if ((this->motor_settings.close_loop_type & ANGLE_LOOP) && this->motor_settings.outer_loop_type == ANGLE_LOOP)
        {
            this->set_length = ref;
        }


        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((this->motor_settings.close_loop_type & SPEED_LOOP) && (this->motor_settings.outer_loop_type & SPEED_LOOP))
        {
            this->set_speed = ref;
        }

        this->motor_controller.RefValChange(ref);
    }

    void DJI_Motor_Instance::InitMotorCan(BSP::CAN_Init_Config_s &can_init_config)
    {
        uint8_t motor_id = can_init_config.tx_id - 1;// 下标从零开始,先减一方便赋值
        uint8_t motor_send_num;
        uint8_t motor_grouping;
#ifdef USE_H7
        CANHandle can1 = &hfdcan1;
#else
        CANHandle can1 = &hcan1;
#endif
        switch (this->motor_type)
        {
            case MOTOR_TYPE_M3508:
            case MOTOR_TYPE_M2006:
            {
                if (motor_id < 4)// 根据ID分组
                {
                    motor_send_num = motor_id;
                    motor_grouping = can_init_config.can_handle == can1 ? 1 : 4;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = can_init_config.can_handle == can1 ? 0 : 3;
                }
                // 计算接收id并设置分组发送id
                can_init_config.rx_id = 0x200 + motor_id + 1;   // 把ID+1,进行分组设置
                this->sender_enable_flag[motor_grouping] = true;// 设置发送标志位,防止发送空帧
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                break;
            }
            case MOTOR_TYPE_GM6020:
            {
                if (motor_id < 4)
                {
                    motor_send_num = motor_id;
                    motor_grouping = can_init_config.can_handle == can1 ? 0 : 3;
                }
                else
                {
                    motor_send_num = motor_id - 4;
                    motor_grouping = can_init_config.can_handle == can1 ? 2 : 5;
                }
                can_init_config.rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
                this->sender_enable_flag[motor_grouping] = true;// 只要有电机注册到这个分组,置为1;在发送函数中会通过此标志判断是否有电机注册
                this->message_num = motor_send_num;
                this->sender_group = motor_grouping;
                break;
            }
            default:
                Util::Assert(false);
                break;
        }

        // 初始化电机CAN
        this->motor_can_instance.Init(can_init_config, this);

        // 检查是否发生id冲突
        for (uint8_t i = 0; i < instances.size(); ++i)
        {
            if (instances.get(i)->motor_can_instance.Get_can_handle() == this->motor_can_instance.Get_can_handle() &&
                instances.get(i)->motor_can_instance.Get_rx_id() == this->motor_can_instance.Get_rx_id())
            {
                // [6020的id 1-4] 和 [2006/3508的id 5-8] 会发生冲突
                Util::Assert(false);
            }
        }
    }

    /**
    * @brief 电机堵转初始化
    * @param init_current 堵转初始化设置电流, 注意方向
    * @return 如果初始化完成, 则返回 false;
    * @note 为了方便可能的重复初始化, 因此在初始化判断完成, 返回 true 之后, 再次调用会重新开始初始化进程
    * @note 因此, 最好在外部额外用一个变量, 判断是否完成初始化, 初始化完成, 后不再继续调用该函数
    */
    bool DJI_Motor_Instance::DJIMotorBlockInit(int16_t init_current)
    {
        // 限制电流大小
        switch (this->motor_type)
        {
            case MOTOR_TYPE_GM6020:
            case MOTOR_TYPE_M3508:
                init_current = UserMath::clamp(init_current, -16384, 16384);
                break;
            case MOTOR_TYPE_M2006:
                init_current = UserMath::clamp(init_current, -10000, 10000);
                break;
            default:
                break;
        }

        this->block_val.init_current = init_current;
        if (this->block_val.block_init_if_finish)
        {
            this->block_val.block_init_if_finish = 0;
            this->block_val.last_position = 0;
            this->block_val.last_position = 0;
            return true;
        }
        else
        {
            this->state = MOTOR_INIT;
            return false;
        }
    }

    /**
     * @brief DJI 电机使能
     * @note 默认为使能
     */
    void DJI_Motor_Instance::DJIMotorEnable()
    {
        this->state = MOTOR_ENABLED;
    }

    /**
     * @brief DJI 电机停止
     * @note 具体实现为在发送时对应数据发0
     */
    void DJI_Motor_Instance::DJIMotorStop()
    {
        this->state = MOTOR_STOP;
    }

    /**
     * @brief Dji 电机锁死
     * @note 具体实现为不允许更改闭环运算参考值
     */
    void DJI_Motor_Instance::DJIMotorLock()
    {
        this->state = MOTOR_Lock;
    }

    /**
     * @brief 设置 dt 值
     * @param dt 新的dt值
     * @return none
     */
    void DJI_Motor_Instance::Set_dt(float dt)
    {
        this->dt = dt;
    }

    /**
     * @brief 获取当前 dt 值
     * @return 当前 dt 值
     */
    float DJI_Motor_Instance::Get_dt() const
    {
        return this->dt;
    }

    void DJI_Motor_Instance::DJIMotorBlockInitAchieve()
    {
        if (this->state == MOTOR_INIT)
        {
            if (this->block_val.block_init_if_finish)
            {
                this->MotorMeasure.MeasureClear();
                this->DJIMotorEnable();
                return;
            }
            if (this->MotorMeasure.measure.is_first)
            {
                if (UserMath::abs<int32_t>(this->block_val.last_position - this->MotorMeasure.measure.record_ecd) < 20)
                {
                    this->block_val.block_times++;
                }
                else
                {
                    this->block_val.block_times = 0;
                }

                if (this->block_val.block_times > 400)
                {
                    this->block_val.block_init_if_finish = 1;
                }
            }
            this->block_val.last_position = this->MotorMeasure.measure.record_ecd;
        }
    }

    /***
     * @brief 清空measure 结构体中的内容
     */
    void DJIMotorMeasure_c::MeasureClear()
    {
        memset(&this->measure, 0, sizeof(DJIMotorMeasure_t));
        this->measure.is_first = false;
    }

    // 临角处理16位（对应角度正值）
    int16_t angle_limiting_int16(int16_t Angl_Err, int16_t lap_encoder)
    {
        //|当前值 - 上一次值| > 编码器最大值/2 时说明向上溢出
        if (Angl_Err < -(lap_encoder / 2))
        {
            Angl_Err += (lap_encoder - 1);
        }
        if (Angl_Err > (lap_encoder / 2))
        {
            Angl_Err -= (lap_encoder - 1);
        }
        return Angl_Err;
    }

    // 为所有电机实例计算三环PID,发送控制报文
    void DJIMotorControl()
    {
        // 直接保存一次指针引用从而减小访存的开销,同样可以提高可读性
        uint8_t group, num;// 电机组号和组内编号
        int16_t set;       // 电机控制CAN发送设定值
        DJI_Motor_Instance *motor;
        Motor_Control_Setting_s *motor_setting;// 电机控制参数
        Motor_Controller_c *motor_controller;  // 电机控制器
        float pid_ref;                         // 电机PID测量值和设定值

        // 遍历所有电机实例,进行串级PID的计算并设置发送报文的值
        for (uint8_t i = 0; i < DJI_Motor_Instance::instances.size(); ++i)
        {
            // 减小访存开销,先保存指针引用
            motor = DJI_Motor_Instance::instances.get(i);
            motor_setting = &motor->motor_settings;
            motor_controller = &motor->motor_controller;
            pid_ref = motor_controller->GetRefVal();// 保存设定值,防止motor_controller->pid_ref在计算过程中被修改

            // 分组填入发送数据
            group = motor->sender_group;
            num = motor->message_num;
            // 若该电机处于停止状态,直接将buff置零
            if (motor->state == MOTOR_STOP)
            {
                memset(DJI_Motor_Instance::sender_assignment[group].tx_buff + 2 * num, 0, sizeof(uint16_t));
                continue;
            }

            if (motor->state == MOTOR_INIT)
            {
                motor->DJIMotorBlockInitAchieve();
                set = motor->block_val.init_current;
                DJI_Motor_Instance::sender_assignment[group].tx_buff[2 * num] = static_cast<uint8_t>(set >> 8);        // 低八位
                DJI_Motor_Instance::sender_assignment[group].tx_buff[2 * num + 1] = static_cast<uint8_t>(set & 0x00ff);// 高八位
                motor->give_current = set;
                continue;
            }

            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
                pid_ref = -pid_ref;// 设置反转

            // pid_ref会顺次通过被启用的闭环充当数据的载体
            // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
            if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
            {
                if (motor_setting->angle_feedback_source == OTHER_FEED)
                    motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(motor->other_angle_feedback_ptr_);
                else
                    motor->motor_controller.angle_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.angle_single_round);
                // 更新pid_ref进入下一个环
                pid_ref = motor_controller->angle_PID.ECF_PID_Calculate(pid_ref);
            }

            // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
            if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
            {
                // 有前馈的话才用到下面这段代码
                // if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                //     pid_ref += *motor_controller->speed_feedforward_ptr;
                if (motor_setting->speed_feedback_source == OTHER_FEED)
                    motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(motor->other_speed_feedback_ptr_);
                else
                    motor->motor_controller.speed_PID.ECF_PID_ChangeActValSource(&motor->MotorMeasure.measure.feedback_speed);
                // 更新pid_ref进入下一个环
                pid_ref = motor_controller->speed_PID.ECF_PID_Calculate(pid_ref);
            }
            // 计算电流环,目前只要启用了电流环就计算,不管外层闭环是什么,并且电流只有电机自身传感器的反馈
            if (motor_setting->close_loop_type & CURRENT_LOOP)
            {
                if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
                    pid_ref += *motor->current_feedforward_ptr_;
                pid_ref = motor_controller->speed_PID.ECF_PID_Calculate(pid_ref);
            }

            // 获取最终输出

            set = static_cast<int16_t>(pid_ref);

            switch (motor->motor_type)
            {
                case MOTOR_TYPE_GM6020:
                case MOTOR_TYPE_M3508:
                    set = UserMath::clamp(set, -16384, 16384);
                    break;
                case MOTOR_TYPE_M2006:
                    set = UserMath::clamp(set, -10000, 10000);
                    break;
                default:
                    break;
            }
            motor->give_current = set;
            DJI_Motor_Instance::sender_assignment[group].tx_buff[2 * num] = static_cast<uint8_t>(set >> 8);        // 低八位
            DJI_Motor_Instance::sender_assignment[group].tx_buff[2 * num + 1] = static_cast<uint8_t>(set & 0x00ff);// 高八位
        }

        // 遍历flag,检查是否要发送这一帧报文
        for (uint8_t i = 0; i < 6; ++i)
        {
            if (DJI_Motor_Instance::sender_enable_flag[i])
            {
                DJI_Motor_Instance::sender_assignment[i].ECF_Transmit(9);
            }
        }
    }


    /**
     * @todo  是否可以简化多圈角度的计算？
     * @brief 根据返回的can_instance对反馈报文进行解析
     *
     * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
     */
    void DebugDecodeDJIMotor(void *data, uint8_t length, void *private_data)
    {
        DJI_Motor_Instance *register_instance = reinterpret_cast<DJI_Motor_Instance *>(private_data);
        if (register_instance->motor_can_instance.Get_rx_id() > 0x200 && register_instance->motor_can_instance.Get_rx_id() < 0x20C) {}
        else// 不符合电机返回ID的数据不进行处理
        {
            return;
        }
        // Debug查看数据看dji_motor_instance_p这个数组里面的值
        for (uint8_t i = 0; i < DJI_Motor_Instance::instances.size(); i++)
        {
            DJI_Motor_Instance *motor = DJI_Motor_Instance::instances.get(i);
            if (motor->motor_can_instance.Get_rx_id() == register_instance->motor_can_instance.Get_rx_id())
            {
                uint8_t *rxbuff = static_cast<uint8_t *>(data);
                DJIMotorMeasure_c *measure = &motor->MotorMeasure;// measure要多次使用,保存指针减小访存开销
                motor->Set_dt(BSP::DWT_c::Get_Instance()->GetFloatDeltaT(&motor->feed_cnt));
                // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
                measure->measure.last_ecd = measure->measure.feedback_ecd;
                measure->measure.feedback_ecd = ((uint16_t) rxbuff[0]) << 8 | rxbuff[1];
                if (measure->measure.is_first == false)
                {
                    measure->measure.is_first = true;
                    measure->measure.last_ecd = measure->measure.first_ecd = measure->measure.feedback_speed;
                    measure->measure.record_ecd = 0;
                    measure->measure.last_record_ecd = measure->measure.record_ecd;
                    motor->DJIMotorEnable();
                }
                measure->measure.angle_single_round = (float) measure->measure.feedback_ecd / measure->lap_encoder * 360.0;
                measure->measure.feedback_speed = (float)(int16_t)(rxbuff[2] << 8 | rxbuff[3]);

                measure->measure.speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->measure.speed_aps +
                                    RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF *  measure->measure.feedback_speed;
                measure->measure.feedback_real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->measure.feedback_real_current +
                                    CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
                measure->measure.feedback_temperature = rxbuff[6];
                // 单个电机温度保护
                if(measure->measure.feedback_temperature > 60)
                {
                    motor->DJIMotorStop();
                }
                // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
                volatile int erro = measure->measure.feedback_ecd - measure->measure.last_ecd;
                if (erro > (measure->lap_encoder / 2.0f))
                {
                    measure->measure.total_round--;
                }
                else if (erro< -(measure->lap_encoder / 2.0f) )
                {
                    measure->measure.total_round++;
                }
                measure->measure.last_record_ecd = measure->measure.record_ecd;
                erro = angle_limiting_int16(erro, measure->lap_encoder);

                measure->measure.record_ecd += erro;

                measure->measure.total_round  = measure->measure.record_ecd / measure->lap_encoder + measure->measure.angle_single_round / 360.0f;
                measure->measure.total_angle = measure->measure.total_round * 360.0f;
                measure->measure.record_length = measure->measure.record_ecd / measure->ecd2length;
                measure->measure.angular_speed = measure->measure.speed_aps / measure->gear_Ratio;
                measure->measure.linear_speed = measure->measure.angular_speed * measure->radius;
                break;
            }
        }
    }

}// namespace Motor
