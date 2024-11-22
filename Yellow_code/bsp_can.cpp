/*************************** Dongguan-University of Technology -ACE**************************
 * @file    bsp_can.cpp
 * @author  study-sheep & KazuHa12441
 * @version V1.1
 * @date    2024/9/14
 * @brief   CAN的BSP层文件
 ******************************************************************************
 * @verbatim
 *  支持CAN1和CAN2，支持标准帧和扩展帧
 *  使用方法：
 *      在目标文件里面定义一个CANInstance_c对象，并传入can句柄，发送id，接收id，标准帧/拓展帧，回调函数。
 *      然后就可以用该对象愉快地调用他的成员函数了。
 *  demo：
 *      // 定义一个CANInstance_c对象
 *      @param hcan1              hcan句柄
 *      @param 0x123              发送id
 *      @param 0x456              接收id
 *      @param CAN_ID_EXT         CAN报文类型 拓展帧
 *      @param dji_motor_callback 回调函数
 *      void dji_motor_callback(CANInstance_c* register_instance)
 *      {
 * 
 *      }
 *      // can传输函数，发送数据帧到总线，用于驱动电机/CAN双板/多板通信
 *      @param  5 是邮箱堵塞的等待的最大时间,单位ms
 *      int16_t set = 1000;
 *      BSP_CAN_Part_n::CANInstance_c dji_motor2(&hcan1, 0x200, 0x201, CAN_ID_STD, dji_motor_callback);
 *      dji_motor2.tx_buff[0] = set >> 8;
 *      dji_motor2.tx_buff[1] = set;
 *      dji_motor2.tx_buff[2] = set >> 8;
 *      dji_motor2.tx_buff[3] = set;
 *      dji_motor2.tx_buff[4] = set >> 8;
 *      dji_motor2.tx_buff[5] = set;
 *      dji_motor2.tx_buff[6] = set >> 8;
 *      dji_motor2.tx_buff[7] = set;
 *      while (1)
 *      {
 *          dji_motor2.send(1); // 发送数据   已测试，别喷我！！！
 *      }
 *      // 修改CAN发送报文的数据帧长度
 *      @param  7        数据长度
 *      dji_motor1.ECF_CAN_SetDLC(7); // 设置数据长度
 *
 * @attention
 *      在ECF_CANFIFOxCallback函数里面的条件判断(rxconf.IDE = CAN_ID_EXT) 目前拓展帧就用到DM电机，他的Extid是包含消息，是不断变换的。
 *      如果其他地方用到拓展帧，需要Extid完全匹配的（ID不会发生变化），可以考虑使用ID List Mode
 * 
 *      目前把CAN过滤器设置为ID Mask Mode,过滤器不过滤，由后面CAN接收中断函数来判断接收的数据是不是要使用
 *      如果想要开启ID List Mode，在CANFilter_Config函数里面把部分代码注释取消，加上两行注释的代码即可，
 *      或者考虑是不是要CANInstance_c加上一个成员变量，来判断使用实例使用 ID Mask Mode  或者  ID List Mode
 * @version                                                  time
 * v1.0   基础版本                                           2024-9-11
 * v1.1   C++优化版本                                        2024-9-16
 * v1.2   修复了CAN外设初始化忘记把can1_is_置为OK的bug🤡      2024-9-16
 * v1.3   对宏定义进行修改，调用抽象类进行继承重写，优化文件结构 2024-9-20
 ************************** Dongguan-University of Technology -ACE***************************/

#include "bsp_can.hpp"
#include "bsp_dwt.hpp"

#include <functional>
#ifdef USE_H7
#include <fdcan.h>
#else
#include <can.h>
#endif

/**
 * @brief 在第一个CAN实例初始化的时候会自动调用此函数,启动CAN服务
 *
 * @note 此函数会启动CAN1和CAN2,开启CAN1和CAN2的FIFO0 & FIFO1溢出通知
 *
 */
static void CANServiceInit()
{
#ifdef USE_H7
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);// 启动中断
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);// 启动中断
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);// 启动中断
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);// 启动中断
#else
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);// 如果测试时，只需要一个CAN的话，在这里把另一个can的初始化注释掉
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);// F4只有两个CAN  H7 老老实实 用FDCAN文件，兼容传统的CAN和FDCAN的设置
#endif
}

namespace BSP
{
    Util::InstanceManger<CANInstance_c, MX_REGISTER_CNT> CANInstance_c::instances;

    // 上层模块调用的CAN构造函数
    void CANInstance_c::init(CAN_Init_Config_s can_config, void *private_data)
    {
        this->config = can_config;
        this->private_data = private_data;

        if (instances.size() == 0)
        {
            CANServiceInit();
        }

        // 进行发送报文的配置
#ifdef USE_H7
        txconf.IdType = config.SAND_IDE;
        txconf.Identifier = config.tx_id;
        txconf.TxFrameType = FDCAN_DATA_FRAME;         // 数据帧
        txconf.DataLength = config.DLC;                // 发送长度
        txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 指定错误状态指示器（发送节点错误活跃
        txconf.BitRateSwitch = FDCAN_BRS_OFF;          // 指定发送的T恤帧是带位率转换还是不带
        txconf.FDFormat = FDCAN_CLASSIC_CAN;           // 指定发送帧是classic 还是fd
        txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;// 指定帧开始时捕获的时间戳计数器值传播（不存储tx事件
        txconf.MessageMarker = 0;                      // 指定复制到Tx EventFIFO元素中的消息标记用于识别Tx信息状态
#else
        txconf.IDE = can_config.SAND_IDE;
        if (can_config.SAND_IDE == CAN_ID_STD)
        {
            txconf.StdId = can_config.tx_id;// 使用标准id
        }
        else
        {
            txconf.ExtId = can_config.tx_id;// 使用扩展id
        }
        txconf.RTR = CAN_RTR_DATA;// 发送数据帧(目前没有远程帧的需求)
        txconf.DLC = config.DLC;  // 发送长度
#endif

        // 将当前实例加入指针数组中
        if (config.on_receive != nullptr)
        {
            instances.add(this);
        }

        // 添加CAN过滤器规则
        Filter_Config();
    }

    /**
     * @brief CAN传输函数，发送数据帧到总线，用于驱动电机
     * @param delay_ms 设定延时时间，单位ms
     */
    CAN_State_e CANInstance_c::send(float delay_ms)
    {
        // 根据不同的CPU，获得不同的函数
#ifdef USE_H7
        auto CAN_GetTxFifoFreeLevel = HAL_FDCAN_GetTxFifoFreeLevel;
        auto CAN_AddMessageToTxFifoQ = std::bind(HAL_FDCAN_AddMessageToTxFifoQ, config.can_handle, &txconf, tx_buff);
#else
        uint32_t tx_mailbox;
        auto CAN_GetTxFifoFreeLevel = HAL_CAN_GetTxMailboxesFreeLevel;
        auto CAN_AddMessageToTxFifoQ = std::bind(HAL_CAN_AddTxMessage, config.can_handle, &txconf, tx_buff, &tx_mailbox);
#endif

        // 延时
        DWT_c::Get_Instance()->Delay_ms(delay_ms);

        // 等待邮箱空闲
        while (CAN_GetTxFifoFreeLevel(config.can_handle) == 0)
        {
            ;
        }

        // 发送消息
        if (CAN_AddMessageToTxFifoQ())
        {
            return CAN_ERROR;
        }

        // 发送成功
        return CAN_OK;
    }

    CANHandle CANInstance_c::get_can_handle() const
    {
        return config.can_handle;
    }

    uint32_t CANInstance_c::get_tx_id() const
    {
        return config.tx_id;
    }

    uint32_t CANInstance_c::get_rx_id() const
    {
        return config.rx_id;
    }

    /**
     * @brief 添加过滤器以实现对特定id的报文的接收,会被CANInstance_c的构造函数调用
     *        给CAN添加过滤器后,BxCAN会根据接收到的报文的id进行消息过滤,符合规则的id会被填入FIFO触发中断
     *
     * @note f407的bxCAN有28个过滤器,这里将其配置为前14个过滤器给CAN1使用,后14个被CAN2使用
     *       初始化时,奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
     *       注册到CAN1的模块使用过滤器0-13,CAN2使用过滤器14-27
     *
     * @attention 你不需要完全理解这个函数的作用,因为它主要是用于初始化,在开发过程中不需要关心底层的实现
     *            享受开发的乐趣吧!如果你真的想知道这个函数在干什么,请联系作者或自己查阅资料(请直接查阅官方的reference manual)
     *
     * @param _instance can instance owned by specific module
     */
    void CANInstance_c::Filter_Config() const
    {
        // 检测关键传参
        Util::Assert(config.can_handle != nullptr);

        // 设置过滤参数
        CAN_Filter filter;
#ifdef USE_H7
        filter.IdType = txconf.IdType;         // 32位工作
        filter.FilterType = FDCAN_FILTER_RANGE;// 范围过滤
        filter.FilterIndex = 0;                // bank
        filter.FilterID1 = 0;                  // 传统模式
        filter.FilterID2 = 0;
        filter.RxBufferIndex = 0x0000;
        filter.IsCalibrationMsg = 0;
        filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
#else
        static uint8_t can1_filter_idx = 0, can2_filter_idx = 14;// 0-13给can1用,14-27给can2用
        // 数据帧
        // 掩码后ID的高16bit
        filter.FilterIdHigh = 0x00;
        // 掩码后ID的低16bit
        filter.FilterIdLow = 0x00;
        // ID掩码值高16bit
        filter.FilterMaskIdHigh = 0x00;
        // ID掩码值低16bit
        filter.FilterMaskIdLow = 0x00;

        /*
          // 遥控帧
          // 掩码后ID的高16bit
          can_filter_init_structure.FilterIdHigh = txconf.StdId;
          // 掩码后ID的低16bit
          can_filter_init_structure.FilterIdLow = (txconf.IDE & 0x03) << 1;
          // ID掩码值高16bit
          can_filter_init_structure.FilterMaskIdHigh = 0;
          // ID掩码值低16bit
          can_filter_init_structure.FilterMaskIdLow = 0;
        */

        filter.FilterMode = CAN_FILTERMODE_IDMASK;
        filter.FilterScale = CAN_FILTERSCALE_32BIT;

        // 滤波器序号, 0-27, 共28个滤波器, can1是0~13, can2是14~27
        filter.FilterBank = config.can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);// 根据can_handle判断是CAN1还是CAN2,然后自增;
        // 滤波器绑定FIFOx, 只能绑定一个
        filter.FilterFIFOAssignment = (config.tx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;// 奇数id的模块会被分配到FIFO0,偶数id的模块会被分配到FIFO1
        // 使能滤波器
        filter.FilterActivation = CAN_FILTER_ENABLE;
        // 从机模式选择开始单元
        filter.SlaveStartFilterBank = 14;// 从第14个过滤器开始配置从机过滤器(在STM32的BxCAN控制器中CAN2是CAN1的从机
#endif

        // 根据不同的CPU选择不同的函数
#ifdef USE_H7
        auto CAN_ConfigFilter = HAL_FDCAN_ConfigFilter;
#else
        auto CAN_ConfigFilter = HAL_CAN_ConfigFilter;
#endif

        // 添加过滤器
        if (CAN_ConfigFilter(config.can_handle, &filter) != HAL_OK)
        {
            Error_Handler();
        }
    }

    /* -----------------------belows are callback definitions--------------------------*/

    /**
     * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
     *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
     *
     * @param _hcan
     * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
     */
    void CANInstance_c::CommonCallback(CANHandle _hcan, uint32_t fifox)
    {
#ifdef USE_H7
        auto CAN_GetRxMessage = HAL_FDCAN_GetRxMessage;
        auto GetRxFifoFillLevel = HAL_FDCAN_GetRxFifoFillLevel;
#else
        auto CAN_GetRxMessage = HAL_CAN_GetRxMessage;
        auto GetRxFifoFillLevel = HAL_CAN_GetRxFifoFillLevel;
#endif
        while (GetRxFifoFillLevel(_hcan, fifox))// FIFO不为空,有可能在其他中断时有多帧数据进入
        {
            CAN_RxHeader rxconf;                                 // 数据身份信息
            uint8_t can_rx_data[8];                              // 接收数据的缓存区
            CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_data);// 从FIFO中获取数据
            for (size_t i = 0; i < instances.size(); i++)
            {
                // 两者相等说明这是要找的实例
                CANInstance_c *instance_c = instances.get(i);
                if (_hcan != instance_c->config.can_handle)
                {
                    continue;
                }

                // 标准帧需要检测ID是否匹配
#ifdef USE_H7
                if (rxconf.IdType == FDCAN_STANDARD_ID && rxconf.Identifier != instance_c->config.rx_id)
#else
                if (rxconf.IDE == CAN_ID_STD && rxconf.StdId != instance_c->config.rx_id)
#endif
                {
                    continue;
                }

                // 调用回调函数
                if (instance_c->config.on_receive != nullptr)
                {
#ifdef USE_H7
                    instance_c->config.on_receive(&can_rx_data, rxconf.DataLength, instance_c->private_data);// 触发回调进行数据解析和处理
#else
                    instance_c->config.on_receive(&can_rx_data, rxconf.DLC, instance_c->private_data);// 触发回调进行数据解析和处理
#endif
                }
                return;
            }
        }
    }
}

/**
 * @brief 注意,STM32的两个CAN设备共享两个FIFO
 * 下面两个函数是HAL库中的回调函数,他们被HAL声明为__weak,这里对他们进行重载(重写)
 * 当FIFO0或FIFO1溢出时会调用这两个函数
 * @note 下面的函数会调用CANFIFOxCallback()来进一步处理来自特定CAN设备的消息
 */

#ifdef USE_H7

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_FDCAN_RxFifo0Callback(CANHandle hfdcan, uint32_t RxFifo0ITs)
{
    BSP::CANInstance_c::CommonCallback(hfdcan, FDCAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}

/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_FDCAN_RxFifo1Callback(CANHandle hfdcan, uint32_t RxFifo0ITs)
{
    BSP::CANInstance_c::CommonCallback(hfdcan, FDCAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

#else

/**
 * @brief rx fifo callback. Once FIFO_0 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_0 comes from
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CANHandle hcan)
{
    BSP::CANInstance_c::CommonCallback(hcan, CAN_RX_FIFO0); // 调用我们自己写的函数来处理消息
}
/**
 * @brief rx fifo callback. Once FIFO_1 is full,this func would be called
 *
 * @param hcan CAN handle indicate which device the oddest mesg in FIFO_1 comes from
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CANHandle hcan)
{
    BSP::CANInstance_c::CommonCallback(hcan, CAN_RX_FIFO1); // 调用我们自己写的函数来处理消息
}

#endif
