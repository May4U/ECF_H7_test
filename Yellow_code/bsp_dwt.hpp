/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_dwt.hpp
 * @author  黄应杰
 * @version V1.3
 * @date    2024/11/15
 * @brief   DWT的BSP层文件，用于计时
 ******************************************************************************
 * @verbatim
 *  DWT计时函数,支持防止dwt被多次初始化
 *  使用方法：
 *      调用ECF_Get_DwtInstance()函数来获取唯一实例的地址，然后就可以用指针愉快地调用成员函数啦
 *  demo：
 *      // 延时函数
 *      BSP::DWT_c::Get_Instance()->Delay_s(1); // 延时1s
 *
 *      // 获取当前时间,单位为秒/s,即初始化后的时间
 *      // @return time 当前时间,单位为: 秒/s
 *      float time = BSP::DWT_c::Get_Instance()->GetTimeline_s(); // 获取当前时间,单位为秒/s
 *
 *      // 获取两次调用之间的时间间隔,单位为秒/s
 *      // @param cnt_last 定义一个uint32_t的计时变量，将其地址作为参数传入
 *      // @return dt 时间间隔,单位为: 秒/s
 *      uint32_t cnt_last;
 *      float dt = BSP::DWT_c::Get_Instance()->GetFloatDeltaT(&cnt_last);
 * @attention
 *      无
 * @version
 * v1.0 2024-9-11  基础版本
 * v1.1 2024-9-14  C++优化版本                                  已测试
 * v1.2 2024-9-29  采用命名空间，而且定时更准确，支持ms、us的延时      已测试
 * v1.3 2024-11-15 黄应杰使用模板元编程重构了一份
 ************************** Dongguan-University of Technology -ACE***************************/

#pragma once

#ifndef __BSP_DWT_HPP
#define __BSP_DWT_HPP

#include "bsp_dwt.hpp"
#include <cstdint>
#include <limits>

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

namespace BSP
{
    /**
     * @enum TimeUnit
     * @brief 时间单位
     */
    enum TimeUnit
    {
        TIME_UNIT_S, // 秒
        TIME_UNIT_MS,// 毫秒
        TIME_UNIT_US,// 微秒
    };

    namespace details
    {
        template<uint32_t value_>
        struct ConstUInt32
        {
            static constexpr uint32_t value = value_;
        };

        template<TimeUnit timeUnit>
        struct TimeUnit2Us
        {};

        template<>
        struct TimeUnit2Us<TIME_UNIT_S> : ConstUInt32<1000000>
        {};

        template<>
        struct TimeUnit2Us<TIME_UNIT_MS> : ConstUInt32<1000>
        {};

        template<>
        struct TimeUnit2Us<TIME_UNIT_US> : ConstUInt32<1>
        {};

        /**
         * @brief 用于获取1个时间单位等于多少微秒
         * @tparam timeUnit 时间单位
         */
        template<TimeUnit timeUnit>
        constexpr int32_t TimeUnit2Us_v = TimeUnit2Us<timeUnit>::value;

    }// namespace details

    /**
     * @class DWT_c
     * @tparam CPU_FREQ_mHz CPU频率,单位MHzz,c板为168MHz,A板为180MHz
     */
    template<uint32_t CPU_FREQ_mHz>
    class Basic_DWT_c
    {
    private:
        uint32_t CYCCNT_RountCount;// 用于记录CYCCNT计数器溢出的次数
        uint32_t CYCCNT_LAST;      // 用于记录上一次读取CYCCNT计数器的值

    private:
        /**
         * @brief 初始化DWT
         */
        explicit Basic_DWT_c()
        {
            // 使能DWT外设
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
            // DWT CYCCNT寄存器计数清0
            DWT->CYCCNT = 0u;
            // 使能Cortex-M DWT CYCCNT寄存器
            DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
            this->CYCCNT_RountCount = 0;
            this->CYCCNT_LAST = 0;
            DWT_CntUpdate();
        }

        ~Basic_DWT_c() = default;

    public:
        explicit Basic_DWT_c(Basic_DWT_c &val) = delete;
        explicit Basic_DWT_c(Basic_DWT_c &&val) = delete;
        Basic_DWT_c &operator=(const Basic_DWT_c &val) = delete;
        Basic_DWT_c &operator=(const Basic_DWT_c &&val) = delete;

        /**
         * @brief 用于获取DWT单例
         * @return BSP_DWT_c<CPU_FREQ_mHz>* DWT实例
         */
        static Basic_DWT_c<CPU_FREQ_mHz> *Get_Instance()
        {
            static Basic_DWT_c<CPU_FREQ_mHz> dwt_time;
            return &dwt_time;
        }

        uint64_t Get_CYCNT64()
        {
            volatile uint32_t cnt_now = DWT->CYCCNT;
            DWT_CntUpdate();
            return static_cast<uint64_t>(this->CYCCNT_RountCount) * std::numeric_limits<uint32_t>::max() + cnt_now;
        }

        /**
         * @brief DWT更新时间轴函数
         * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
         * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
         *            它会检测DWT_CYCCNT寄存器的值是否小于上一次读取时的值，如果是，则认为计数器已经溢出，将CYCCNT_RountCount加1，
         *            然后更新CYCCNT_LAST的值为当前的计数器值，以备下次比较使用。
         */
        void DWT_CntUpdate()
        {
            static volatile bool bit_locker = true;
            if (bit_locker)
            {
                bit_locker = false;
                volatile uint32_t cnt_now = DWT->CYCCNT;
                if (cnt_now < this->CYCCNT_LAST)
                {
                    ++this->CYCCNT_RountCount;
                }
                this->CYCCNT_LAST = DWT->CYCCNT;
                bit_locker = true;
            }
        }

        /**
         * @brief 获取两次调用之间的时间间隔,单位为秒/s
         *
         * @param cnt_last 上一次调用的时间戳
         * @return 时间间隔,单位为秒/s
         */
        template<class T>
        T GetDeltaT(uint64_t *cnt_last)
        {
            uint64_t cnt_now = Get_CYCNT64();
            T dt = static_cast<T>(cnt_now - *cnt_last) / static_cast<T>(CPU_FREQ_mHz * details::TimeUnit2Us_v<TIME_UNIT_S>);
            *cnt_last = cnt_now;
            return dt;
        }

        float GetFloatDeltaT(uint64_t *cnt_last)
        {
            return GetDeltaT<float>(cnt_last);
        }

        double GetDoubleDeltaT(uint64_t *cnt_last)
        {
            return GetDeltaT<double>(cnt_last);
        }

        /**
         * @brief 获取当前时间,单位为秒/s,即初始化后的时间
         *
         * @return 时间轴
         */
        template<TimeUnit timeUnit, class T>
        T GetTimeline()
        {
            uint64_t CYCCNT64 = Get_CYCNT64();
            constexpr uint32_t hz = CPU_FREQ_mHz * details::TimeUnit2Us_v<timeUnit>;
            // 其实是 CYCCNT64 / hz，但为了防止溢出，写复杂一点
            if constexpr (std::numeric_limits<T>::is_integer)
            {
                return static_cast<T>(CYCCNT64 / hz);
            }
            else
            {
                return static_cast<T>(CYCCNT64 / hz) + static_cast<T>(CYCCNT64 % hz) / static_cast<T>(hz);
            }
        }

        float GetTimeline_s()
        {
            return GetTimeline<TIME_UNIT_S, float>();
        }

        float GetTimeline_ms()
        {
            return GetTimeline<TIME_UNIT_MS, float>();
        }

        uint64_t GetTimeline_us()
        {
            return GetTimeline<TIME_UNIT_US, uint64_t>();
        }

        /**
         * @brief DWT延时函数
         * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
         * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
         *
         * @param time 延时时间
         */
        template <TimeUnit timeUnit, class T>
        void Delay(T time)
        {
            uint64_t tick_end = Get_CYCNT64() + time * static_cast<uint64_t>(CPU_FREQ_mHz) * details::TimeUnit2Us_v<timeUnit>;
            while (Get_CYCNT64() < tick_end)
            {
                 ;
            }
        }

        void Delay_s(float time) {
            Delay<TIME_UNIT_S, float>(time);
        }

        void Delay_ms(float time) {
            Delay<TIME_UNIT_MS, float>(time);
        }

        void Delay_us(uint64_t time) {
            Delay<TIME_UNIT_US, uint64_t>(time);
        }
    };

    // STM32控制芯片主频，单位：MHz
    using DWT_c = Basic_DWT_c<168>;

}

#endif