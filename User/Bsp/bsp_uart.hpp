/**
 * @file bsp_uart.hpp
 * @author Rh
 * @brief 实现了一个简易的串口驱动（FreeRTOS + IDLE中断 + DMA）
 * @version 0.3
 * @date 2026-09-10
 *
 * @todo 1. 接收到的数据,需要在应用层的app_message写分发处理
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用示例：（必须要在freertos的任务中运行收发，中断中不行，中断不能阻塞）
 *           使用IDLE中断接收，TX Complete中断链式发送，全程DMA。
 *
 * @note 模板参数为缓冲区大小（uint8_t）
 *
 *   // 全局实例化模板在bsp_uart.cpp中
 *   template class BspUart<128>;
 *
 *   // 全局实例化类 在bsp_cfg.cpp中
 *   __attribute__((section(".dma_buffer")))
 *   BspUart<128> bsp_uart1({&huart1, true});    // huart句柄 + 是否启用发送
 *
 *   bsp_uart1.init();                       // 放到bsp_init中初始化串口（需FreeRTOS调度器已启动）
 *
 *   bsp_uart1.send(buffer, 8);              // 存入发送流缓冲区，DMA自动发送
 *   bsp_uart1.printf("val=%d\r\n", 42);     // 格式化输出（非阻塞，DMA发送）
 *   bsp_uart1.receive(buffer, 8);           // 从接收流缓冲区读数据，读取后对应数据会被清空
 *
 * @note 串口设备只有流缓冲区内置的锁，没有主动写的mutex，多任务并发写需自行协调。
 */

#ifndef __BSP_UART_HPP__
#define __BSP_UART_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "stream_buffer.h"
#include "task.h"  // IWYU pragma: keep
#include "usart.h" // IWYU pragma: keep

#include "status.hpp" // 统一状态码


/**
 * @brief 简易串口驱动（IDLE中断 + DMA收发 + FreeRTOS流缓冲区）
 *
 * @tparam BUFFER_SIZE DMA收发缓冲区大小（单位uint8_t），也是流缓冲区容量
 *
 * @note RX 路径：IDLE中断 → 停DMA → 投递流缓冲区 → 立即重启DMA；
 *       TX 路径：写入流缓冲区 → 任务上下文启动或TX Complete中断续传。
 */
template <size_t BUFFER_SIZE = 256>
class BspUart
{

private:

  UART_HandleTypeDef *_huart; ///< UART句柄指针，指向底层硬件接口

  StreamBufferHandle_t _rx_stream_buffer = nullptr; ///< 接收流缓冲区

  StreamBufferHandle_t _tx_stream_buffer = nullptr; ///< FreeRTOS发送流缓冲区句柄

  bool     _rx_active = false;          ///< 接收状态标志，指示是否正在接收数据
  bool     _transmit_enable;            ///< 是否启用发送
  uint8_t  _rx_dma_buffer[BUFFER_SIZE]; ///< DMA接收缓冲区，用于多字节接收
  uint8_t  _tx_dma_buffer[BUFFER_SIZE]; ///< DMA发送缓冲区，用于多字节发送
  char     _printf_buffer[BUFFER_SIZE]; ///< printf 格式化缓冲区（vsnprintf 输出到此处）


public:
  /**
   * @brief 串口配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     * @param huart           UART 句柄
     * @param transmit_enable 是否启用发送
     */
    Config(UART_HandleTypeDef *huart = nullptr, bool transmit_enable = true)
      : huart(huart),
        transmit_enable(transmit_enable)
    {
    }

    UART_HandleTypeDef *huart;           ///< UART 句柄
    bool                transmit_enable; ///< 是否启用发送
  };

  /**
   * @brief 构造函数（只做赋值，FreeRTOS资源创建推迟到 init()）
   *
   * @param cfg 串口配置（huart/发送使能，可匿名按序传入）
   */
  BspUart(const Config &cfg);

  /**
   * @brief 初始化函数 创建FreeRTOS对象并启动IDLE接收
   *
   * @return Status OK=初始化成功，IO_ERROR=FreeRTOS资源创建失败
   */
  Status init();

  // 析构函数 释放所有分配的资源
  ~BspUart();

  /**
   * @brief 发送数据 将数据放入发送缓冲区，并启动DMA传输。
   *
   * @param data 要发送的数据指针
   * @param size 数据大小（不能超过缓冲区容量，否则返回 BAD_ARG）
   * @param written 实际写入的字节数（可为 nullptr）
   * @param timeout 超时时间（ticks）
   *
   * @return Status OK=全部写入，TIMEOUT=超时部分写入，
   *                BAD_ARG=参数非法或超长，IO_ERROR=未初始化
   */
  Status send(const uint8_t *data, size_t size, size_t *written = nullptr, uint32_t timeout = portMAX_DELAY);

  /**
   * @brief 格式化输出到本串口（printf 风格）
   *
   * @note 内部 vsnprintf 格式化后调用 send()，非阻塞（DMA 发送）；
   *       格式化结果超过缓冲区（BUFFER_SIZE）时自动截断。
   *       必须在任务上下文调用。
   *
   * @param fmt 格式化字符串
   * @param ... 可变参数
   * @return Status OK=发送成功，其余同 send()
   */
  Status printf(const char *fmt, ...);

  /**
   * @brief 从接收流缓冲区读取数据
   *
   * @param buffer 接收数据的缓冲区
   * @param size 请求读取的数据大小
   * @param received 实际读取的字节数（可为 nullptr）
   * @param timeout 超时时间（ticks）
   * @return Status OK=读到数据，TIMEOUT=超时或无数据，
   *                BAD_ARG=参数非法，IO_ERROR=缓冲区未创建
   */
  Status receive(uint8_t *buffer, size_t size, size_t *received = nullptr, uint32_t timeout = portMAX_DELAY);

  /**
   * @brief 获取发送缓冲区剩余空间
   *
   * @return size_t 剩余空间大小
   */
  size_t get_tx_free_space();

  /**
   * @brief 获取接收缓冲区可用数据量
   *
   * @return size_t 可用数据量
   */
  size_t get_rx_available_data();

  /**
   * @brief IDLE接收完成处理（ISR上下文）
   *
   * @note 三步内联：停DMA → 投递流缓冲区 → 立即重启DMA。
   *
   * @param size 本帧接收到的字节数（HAL回调提供）
   * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
   */
  void on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief 发送完成/续传处理（ISR上下文，由 TX Complete 中断调用）
   *
   * @note 流缓冲区中有数据则继续发送，没有数据则直接返回。
   *
   * @param pxHigherPriorityTaskWoken 需初始化为pdFALSE，若唤醒高优先级任务则置为pdTRUE
   */
  void start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken);

  /**
   * @brief DMA错误恢复：停DMA并重启接收（供 HAL_UART_ErrorCallback 接线调用）
   */
  void handle_dma_error();

private:
  // 开始接收数据 启动DMA接收
  void start_reception();

  // 停止接收数据 停止DMA接收
  void stop_reception();

  // 清理资源 清理所有分配的资源
  void cleanup_resources();

  // 开始传输数据 启动DMA发送（任务上下文）
  void start_transmission();

  /**
   * @brief 检查是否正在传输
   *
   * @return true 正在传输
   * @return false 未在传输
   */
  bool is_transmitting();
};


#endif // __BSP_UART_HPP__
