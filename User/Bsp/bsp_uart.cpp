#include "bsp_uart.hpp"
#include "bsp_cfg.hpp" // 中断回调中直接引用 bsp_usartX / bsp_uartX 全局实例
#include "FreeRTOS.h"  // IWYU pragma: keep
#include <stdarg.h>
#include <stdio.h>

/* ==================== 模板实例化 ==================== */

/**
 * @brief 模板实例化实现
 * @param 缓冲区大小（uint8_t）
 *
 */
template class BspUart<128>;


extern "C"
{
  /**
   * @brief IDLE串口回调函数
   * @note 直接 if-else 判断 UART 句柄并调用对应全局实例
   */
  void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart == &huart1)
    {
      bsp_uart1.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
      bsp_uart3.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart4)
    {
      bsp_uart4.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart5)
    {
      bsp_uart5.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart7)
    {
      bsp_uart7.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart8)
    {
      bsp_uart8.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart9)
    {
      bsp_uart9.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }
    else if (huart == &huart10)
    {
      bsp_uart10.on_idle_isr(Size, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief UART TX Complete 回调函数
   * @note 发送完成时触发，链式续传发送缓冲区中的剩余数据
   */
  void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // 注：UART5 无发送功能（未配 TX DMA），不会触发本回调，故无 huart5 分支
    if (huart == &huart1)
    {
      bsp_uart1.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart3)
    {
      bsp_uart3.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart4)
    {
      bsp_uart4.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart7)
    {
      bsp_uart7.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart8)
    {
      bsp_uart8.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart9)
    {
      bsp_uart9.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }
    else if (huart == &huart10)
    {
      bsp_uart10.start_transmission_from_isr(&xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }

  /**
   * @brief UART 错误回调函数
   * @note ORE/FE/NE 等错误后复位 RX 并重新武装，避免接收无声停摆
   */
  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    // UART5 仅接收，同样需要错误恢复
    if (huart == &huart1)
    {
      bsp_uart1.handle_dma_error();
    }
    else if (huart == &huart3)
    {
      bsp_uart3.handle_dma_error();
    }
    else if (huart == &huart4)
    {
      bsp_uart4.handle_dma_error();
    }
    else if (huart == &huart5)
    {
      bsp_uart5.handle_dma_error();
    }
    else if (huart == &huart7)
    {
      bsp_uart7.handle_dma_error();
    }
    else if (huart == &huart8)
    {
      bsp_uart8.handle_dma_error();
    }
    else if (huart == &huart9)
    {
      bsp_uart9.handle_dma_error();
    }
    else if (huart == &huart10)
    {
      bsp_uart10.handle_dma_error();
    }
  }
}

/**
 * @brief BspUart<BUFFER_SIZE> 类函数定义
 *
 * 串口驱动组件实现：只使用IDLE中断接收，DMA普通模式收发。
 * 线程安全：收发各用一条FreeRTOS流缓冲区（内置锁），
 *           无主动写的mutex，多任务并发写同一串口需上层自行协调。
 *
 * @note 经过测试，无任何测试问题。
 *
 * @param BUFFER_SIZE 缓冲区大小（DMA收发缓冲区与流缓冲区容量，单位uint8_t）
 * @param cfg 串口配置（huart/发送使能，可匿名按序传入）
 */
template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::BspUart(const Config &cfg)

  : _huart(cfg.huart),
    _transmit_enable(cfg.transmit_enable)
{
  // 构造函数只做赋值；运行时逻辑（FreeRTOS资源创建）推迟到 init()
}

template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::init()
{
  // 创建接收流缓冲区
  _rx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
  if (_rx_stream_buffer == nullptr)
  {
    cleanup_resources();     // 清理已创建的资源
    return Status::IO_ERROR; // 流缓冲区创建失败
  }

  if (_transmit_enable)
  {
    // 创建发送流缓冲区
    _tx_stream_buffer = xStreamBufferCreate(BUFFER_SIZE, 1);
    if (_tx_stream_buffer == nullptr)
    {
      cleanup_resources();     // 清理已创建的资源
      return Status::IO_ERROR; // 发送流缓冲区创建失败
    }
  }
  else
  {
    _tx_stream_buffer = nullptr;
  }

  // 启动接收（HAL 内部会清 IDLE 标志并使能 IDLE 中断；失败则释放资源并上报）
  if (!arm_reception())
  {
    abort_reception();       // 复位 RX 状态并关掉 IDLE 中断源
    cleanup_resources();     // 释放已创建的资源
    return Status::IO_ERROR; // 接收启动失败，避免静默失能
  }

  return Status::OK; // 初始化成功
}

// 析构函数实现
template <size_t BUFFER_SIZE>
BspUart<BUFFER_SIZE>::~BspUart()
{
  stop_reception();

  cleanup_resources();
}

template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::cleanup_resources()
{
  // 释放接收流缓冲区
  if (_rx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_rx_stream_buffer);
    _rx_stream_buffer = nullptr;
  }

  // 释放发送流缓冲区
  if (_tx_stream_buffer != nullptr)
  {
    vStreamBufferDelete(_tx_stream_buffer);
    _tx_stream_buffer = nullptr;
  }
}

// 发送数据实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::send(const uint8_t *data, size_t size, size_t *written, uint32_t timeout)
{
  if (data == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  // 单次发送不能超过流缓冲区容量，否则 xStreamBufferSend 会永久阻塞（死锁）
  if (size > BUFFER_SIZE)
  {
    return Status::BAD_ARG; // 数据超长，调用方应分包发送
  }

  if (!_transmit_enable || _tx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 未启用发送或发送缓冲区未初始化
  }

  // 将数据写入发送流缓冲区
  size_t bytes_written = xStreamBufferSend(_tx_stream_buffer, data, size, timeout);

  // 如果发送缓冲区中有数据，启动发送
  if (bytes_written > 0)
  {
    start_transmission();
  }

  if (written != nullptr)
  {
    *written = bytes_written;
  }

  return (bytes_written == size) ? Status::OK : Status::TIMEOUT;
}

// printf 格式化发送实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::printf(const char *fmt, ...)
{
  if (fmt == nullptr)
  {
    return Status::BAD_ARG; // 参数非法
  }

  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(_printf_buffer, sizeof(_printf_buffer), fmt, args);
  va_end(args);

  if (len <= 0)
  {
    return Status::BAD_ARG; // 格式化失败或空输出
  }

  // vsnprintf 返回的是期望写入的完整长度，实际写入可能被截断
  if (static_cast<size_t>(len) >= sizeof(_printf_buffer))
  {
    len = static_cast<int>(sizeof(_printf_buffer)) - 1;
  }

  return send(reinterpret_cast<const uint8_t *>(_printf_buffer), static_cast<size_t>(len));
}

// 接收数据实现
template <size_t BUFFER_SIZE>
Status BspUart<BUFFER_SIZE>::receive(uint8_t *buffer, size_t size, size_t *received, uint32_t timeout)
{
  if (buffer == nullptr || size == 0)
  {
    return Status::BAD_ARG; // 参数非法
  }

  if (_rx_stream_buffer == nullptr)
  {
    return Status::IO_ERROR; // 流缓冲区未创建
  }

  size_t bytes_read = xStreamBufferReceive(_rx_stream_buffer, buffer, size, pdMS_TO_TICKS(timeout));
  if (received != nullptr)
  {
    *received = bytes_read;
  }
  return (bytes_read > 0) ? Status::OK : Status::TIMEOUT;
}

// 获取发送缓冲区剩余空间实现
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_tx_free_space()
{
  if (_tx_stream_buffer != nullptr)
  {
    return xStreamBufferSpacesAvailable(_tx_stream_buffer);
  }
  return 0;
}

// 获取接收缓冲区可用数据量实现
template <size_t BUFFER_SIZE>
size_t BspUart<BUFFER_SIZE>::get_rx_available_data()
{
  if (_rx_stream_buffer != nullptr)
  {
    return xStreamBufferBytesAvailable(_rx_stream_buffer);
  }
  return 0;
}

// 武装 DMA 接收实现（仅 RX；调用前 RxState 必须为 READY）
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::arm_reception()
{
  // 启动多字节 DMA 接收（IDLE 模式）；HAL 会自动清 IDLE 标志并使能 IDLE 中断
  if (HAL_UARTEx_ReceiveToIdle_DMA(_huart, _rx_dma_buffer, BUFFER_SIZE) != HAL_OK)
  {
    return false; // 武装失败：由调用方决定重试或上报
  }
  _rx_active = true;
  return true;
}

// 中止 RX 通道实现（仅 RX：清错误标志 + 复位接收状态，不触碰 TX DMA）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::abort_reception()
{
  HAL_UART_AbortReceive(_huart);
  ATOMIC_CLEAR_BIT(_huart->Instance->CR1, USART_CR1_IDLEIE); // 防止残留 IDLE 中断源
}

// 停止接收数据实现（终止场景：RX/TX DMA 全停）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::stop_reception()
{
  _rx_active = false;
  HAL_UART_DMAStop(_huart);
}

// 开始传输数据实现（任务上下文）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::start_transmission()
{
  if (_tx_stream_buffer == nullptr)
  {
    return; // 发送缓冲区未初始化
  }

  if (!is_transmitting())
  {
    // 从发送缓冲区获取数据准备发送（任务级API）
    size_t bytes_to_send = xStreamBufferReceive(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, 0);
    if (bytes_to_send > 0)
    {
      HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer, bytes_to_send);
    }
  }
}

// 发送完成/续传处理实现（ISR上下文，由 TX Complete 中断调用）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::start_transmission_from_isr(BaseType_t *pxHigherPriorityTaskWoken)
{
  if (_tx_stream_buffer == nullptr || !_transmit_enable)
  {
    return; // 未启用发送或发送缓冲区未初始化
  }

  // 从发送缓冲区获取数据准备发送（ISR级API）；无数据则直接返回
  size_t bytes_to_send = xStreamBufferReceiveFromISR(_tx_stream_buffer, _tx_dma_buffer, BUFFER_SIZE, pxHigherPriorityTaskWoken);
  if (bytes_to_send > 0)
  {
    HAL_UART_Transmit_DMA(_huart, _tx_dma_buffer, bytes_to_send);
  }
}

// 检查是否正在传输实现
template <size_t BUFFER_SIZE>
bool BspUart<BUFFER_SIZE>::is_transmitting()
{
  return (_huart->gState == HAL_UART_STATE_BUSY_TX);
}

// IDLE/TC 接收完成处理实现（ISR上下文）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::on_idle_isr(uint16_t size, BaseType_t *pxHigherPriorityTaskWoken)
{
  // HAL 在半传输（HT）事件时也会回调本函数：HT 不是帧边界，直接忽略
  if (_huart->RxEventType == HAL_UART_RXEVENT_HT)
  {
    return;
  }

  // 到此处 RxState 已为 READY —— HAL 在 IDLE/TC 事件中已自行停掉 RX DMA，
  // 这里只需投递数据并重新武装，全程不触碰 TX DMA（不打断正在进行的发送）
  if (_rx_stream_buffer != nullptr && size > 0)
  {
    xStreamBufferSendFromISR(_rx_stream_buffer, _rx_dma_buffer, size, pxHigherPriorityTaskWoken);
  }

  // 重新武装 RX；失败则走一次完整恢复，避免接收无声停摆
  if (_rx_active && !arm_reception())
  {
    handle_dma_error();
  }
}

// UART 错误恢复实现（供 HAL_UART_ErrorCallback 调用）
template <size_t BUFFER_SIZE>
void BspUart<BUFFER_SIZE>::handle_dma_error()
{
  // 未启用接收，或非阻塞错误（RX 仍在进行，不打断以免丢弃在途数据）
  if (!_rx_active || _huart->RxState == HAL_UART_STATE_BUSY_RX)
  {
    return;
  }

  // 阻塞性错误后 HAL 已中止 RX：复位接收状态并重新武装（仅 RX，不触碰 TX DMA）
  abort_reception();
  if (!arm_reception())
  {
    _rx_active = false; // 仍失败：停止重试（需重新 init 才能恢复），避免回调空转
  }
}
