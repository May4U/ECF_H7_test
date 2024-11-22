#pragma once

#ifndef __BSP_CAN_HPP
#define __BSP_CAN_HPP

#include "util_instance_manager.hpp"
#include <cstdint>

// 检测是否是H7，如果是H7，就添加USE_H7这个宏
#if defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || defined(STM32H745xx) || defined(STM32H745xG) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H747xG) || defined(STM32H757xx) || defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H7A3xx) || defined(STM32H7B3xx) || defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) || defined(STM32H735xx) || defined(STM32H733xx) || defined(STM32H730xx) || defined(STM32H730xxQ) || defined(STM32H725xx) || defined(STM32H723xx)
#define USE_H7
#endif

// 根据是否使用H7，导入不用的库
#ifdef USE_H7
#include <stm32h7xx_hal.h>
#else
#include <stm32f4xx_hal.h>
#endif

// 根据是否使用H7，声明不同的类型
#ifdef USE_H7
using CANHandle = FDCAN_HandleTypeDef *;
using CAN_TxHeader = FDCAN_TxHeaderTypeDef;
using CAN_RxHeader = FDCAN_RxHeaderTypeDef;
using CAN_Filter = FDCAN_FilterTypeDef;
#else
using CANHandle = CAN_HandleTypeDef *;
using CAN_TxHeader = CAN_TxHeaderTypeDef;
using CAN_RxHeader = CAN_RxHeaderTypeDef;
using CAN_Filter = CAN_FilterTypeDef;
#endif

namespace BSP
{

#define MX_REGISTER_CNT 28u   //18u     // 这个数量取决于CAN总线的负载(不能等于这个数)
#define MX_FILTER_CNT (2 * 14)// 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE 2              // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1

    enum Define_State_e
    {
        UNDEFINED,
        DEFINE,
    };

    enum CAN_State_e
    {
        CAN_OK,
        CAN_ERROR
    };

    /* CAN实例初始化结构体,将此结构体指针传入注册函数 */
    struct CAN_Init_Config_s
    {
        CANHandle can_handle = nullptr;                                              // can句柄
        uint32_t tx_id = 0;                                                          // 发送id
        uint32_t rx_id = 0;                                                          // 接收id
        uint32_t SAND_IDE = 0;                                                       // 标准帧还是拓展帧
        uint8_t DLC = 8;                                                             // CAN发送报文的数据帧长度;注意最大长度为8
        void (*on_receive)(void *data, uint8_t length, void *private_data) = nullptr;// 回调函数，private_data取决于你调用Init()函数传入的private_data
    };

    class CANInstance_c
    {
    private:
        CAN_Init_Config_s config;
        static Util::InstanceManger<CANInstance_c, MX_REGISTER_CNT> instances;// CAN实例指针数组
        CAN_TxHeader txconf;                                                  // CAN报文发送配置
        void *private_data = nullptr;                                         // 私有数据，会在回调函数传入，用于访问特定的对象

    public:
        uint8_t tx_buff[8];// 发送缓存,发送消息长度可以通过ECF_CAN_SetDLC()设定,最大为8

    public:
        CANInstance_c() = default;
        void init(CAN_Init_Config_s can_config, void *private_data = nullptr);
        CAN_State_e send(float timeout);
        [[nodiscard]] CANHandle get_can_handle() const;
        [[nodiscard]] uint32_t get_tx_id() const;
        [[nodiscard]] uint32_t get_rx_id() const;

    private:
        // 初始化过滤器，在Init函数自动调用
        void Filter_Config() const;

        // 数据先经过这个函数，再分发到各个can实例，这个函数不应该被其它地方调用
        static void CommonCallback(CANHandle _hcan, uint32_t fifox);

        // friend既友元，用于让下面函数可以访问当前类的私有成员
#ifdef USE_H7
        friend void ::HAL_FDCAN_RxFifo0Callback(CANHandle hcan, uint32_t RxFifo0ITs);
        friend void ::HAL_FDCAN_RxFifo1Callback(CANHandle hcan, uint32_t RxFifo0ITs);
#else
        friend void ::HAL_CAN_RxFifo0MsgPendingCallback(CANHandle hcan);
        friend void ::HAL_CAN_RxFifo1MsgPendingCallback(CANHandle hcan);
#endif
    };
};

#endif /* __BSP_CAN_HPP */
