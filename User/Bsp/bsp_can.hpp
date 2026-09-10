/**
 * @file bsp_can.hpp
 * @author Rh
 * @brief CAN2.0 标准帧 8长度 收发驱动
 * @version 0.4
 * @date 2026-05-02
 *
 * @todo 1. 滤波器未处理（可能也不需要处理）
 *       2. 只实现了一个模式
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用示例：
 *
 * @note 先在bsp_cfg实例化并外部声明，再在freertos_init()中初始化它
 *
 *      bsp_can1({&hfdcan1, "CAN1"});
 *      bsp_can1.init();
 *
 * @note 如何使用：（未管理任务级的互斥情况）
 *
 *      uint8_t data[8] = {0x01,0x02...}; // 定义数据内容
 *      bsp_can1.send(0x101,data);        // 存入缓冲区中 自动处理发送
 *
 *      CanRxMsg data1 = {0};             // 定义数据内容
 *      bsp_can1.receive(&data1);         // 从缓冲区取值（自动接收到缓冲区）
 *
 */

#ifndef __BSP_CAN_HPP__
#define __BSP_CAN_HPP__

#include "cmsis_os2.h"
#include "fdcan.h"    // IWYU pragma: keep
#include "FreeRTOS.h" // IWYU pragma: keep
#include "message_buffer.h"
#include "task.h"

#include "status.hpp" // 统一状态码


/**
 * @brief CAN工作模式
 */
enum class CanMode
{
  NORMAL   = 0, ///< 正常模式
  SILENT   = 1, ///< 静默模式
  LOOPBACK = 2  ///< 环回模式
};


/**
 * @brief CAN接收消息结构体（固定8字节数据）
 */
typedef struct
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t               data[8];
} CanRxMsg;


/**
 * @brief CAN发送消息结构体（固定8字节数据）
 */
typedef struct
{
  uint32_t std_id;
  uint8_t  data[8];
} CanTxMsg;


/**
 * @brief CAN驱动类
 *
 * @note CAN2.0标准帧，固定8字节数据
 * @note 收发使用FreeRTOS Message Buffer
 * @note 发送不等待硬件FIFO，直接放入缓冲区，由中断完成发送
 */
class BspCan
{
public:
  /**
   * @brief CAN 配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(FDCAN_HandleTypeDef *hfdcan = nullptr, const char *name = "CAN", CanMode mode = CanMode::NORMAL) : hfdcan(hfdcan),
                                                                                                              name(name),
                                                                                                              mode(mode)
    {
    }

    FDCAN_HandleTypeDef *hfdcan; ///< CAN 句柄
    const char          *name;   ///< 实例名称（用于调试）
    CanMode              mode;   ///< 工作模式
  };

  /**
   * @brief 构造函数
   * @param cfg CAN 配置（句柄/名称/工作模式，可匿名按序传入）
   */
  BspCan(const Config &cfg);
  ~BspCan();

  /**
   * @brief 初始化：创建收发消息缓冲区、配置过滤器、启动硬件与接收中断
   *
   * @return Status OK=初始化成功，IO_ERROR=资源创建/硬件启动失败
   */
  Status init();

  /**
   * @brief 发送一帧 CAN 标准帧（8 字节，非阻塞，入缓冲后由中断发送）
   *
   * @param std_id 标准帧 ID（11 位）
   * @param data   8 字节数据
   * @return Status OK=已入发送缓冲，FULL=发送缓冲满，
   *                BAD_ARG=参数非法，NOT_INIT=未初始化
   */
  Status send(uint32_t std_id, uint8_t *data);

  /**
   * @brief 从接收缓冲取出一帧消息
   *
   * @param msg     接收消息结构体（CanRxMsg，固定 8 字节）
   * @param timeout 等待超时（ticks）
   * @return Status OK=成功取出一帧，TIMEOUT=超时无数据，
   *                BAD_ARG=参数非法，NOT_INIT=未初始化
   */
  Status receive(CanRxMsg *msg, uint32_t timeout);
  void   trigger_tx();                                               // 触发一次发送（任务上下文调用）
  void   trigger_tx_from_isr(BaseType_t *pxHigherPriorityTaskWoken); // 触发一次发送（ISR上下文调用）
  void   process_fifo0_isr();                                        // FIFO0 中断处理（供中断回调调用）

  MessageBufferHandle_t _rx_message_buffer;
  MessageBufferHandle_t _tx_message_buffer;

private:
  FDCAN_HandleTypeDef *_hfdcan;
  const char          *_name;
  CanMode              _work_mode;

  Status config_filter();
  Status start_hardware();
  Status start_reception();
};

#endif
