#include "usart_printf.h"
#include <string.h>

#define UART_TX_BUF_SIZE 512

static UART_HandleTypeDef *g_huart = NULL;
static uint8_t uart_tx_buf[UART_TX_BUF_SIZE];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t tx_dma_busy = 0;
static volatile uint16_t tx_dma_len = 0;

// 加入__FILE结构体定义
struct __FILE
{
    int handle;
};
FILE __stdout;

void usart_printf_init(UART_HandleTypeDef *huart)
{
    g_huart = huart;
    tx_head = 0;
    tx_tail = 0;
    tx_dma_busy = 0;
}

// 获取缓冲区中的数据量
static uint16_t uart_tx_get_count(void)
{
    if (tx_head >= tx_tail) {
        return tx_head - tx_tail;
    } else {
        return UART_TX_BUF_SIZE - tx_tail + tx_head;
    }
}

// 环形缓冲区入队
static int uart_tx_enqueue(uint8_t ch)
{
    uint16_t next = (tx_head + 1) % UART_TX_BUF_SIZE;
    if (next != tx_tail) // 缓冲区未满
    {
        uart_tx_buf[tx_head] = ch;
        tx_head = next;
        return 1;
    }
    return 0; // 缓冲区满
}

// 启动DMA发送
static void uart_tx_start_dma(void)
{
    if (tx_dma_busy || tx_head == tx_tail || g_huart == NULL) return;

    uint16_t len;
    if (tx_head >= tx_tail) {
        len = tx_head - tx_tail;
    } else {
        // 环形缓冲区回绕时，只发送到缓冲区末尾
        len = UART_TX_BUF_SIZE - tx_tail;
    }
    
    tx_dma_len = len;
    tx_dma_busy = 1;
    HAL_UART_Transmit_DMA(g_huart, &uart_tx_buf[tx_tail], len);
}

// 强制刷新缓冲区
void uart_flush(void)
{
    // 等待缓冲区清空
    uint32_t timeout = 0xFFFF;
    while ((tx_head != tx_tail || tx_dma_busy) && timeout--)
    {
        uart_tx_start_dma();
    }
}

// Keil的fputc重定向
int fputc(int ch, FILE *f)
{
    if (g_huart == NULL) return -1;
    
    // 等待缓冲区有空间
    while (1)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        
        if (uart_tx_enqueue((uint8_t)ch))
        {
            // 如果是换行符或缓冲区快满了，立即发送
            if (ch == '\n' || uart_tx_get_count() > UART_TX_BUF_SIZE * 3 / 4)
            {
                uart_tx_start_dma();
            }
            
            __set_PRIMASK(primask);
            break;
        }
        
        // 缓冲区满，尝试启动DMA
        uart_tx_start_dma();
        __set_PRIMASK(primask);
        
        // 短暂延时，让DMA有机会工作
        for (volatile int j = 0; j < 100; j++);
    }
    
    return ch;
}

// 避免使用半主机模式
void _sys_exit(int x)
{
    x = x;
}

// 避免使用半主机模式
void _ttywrch(int ch)
{
    ch = ch;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == g_huart)
    {
        // 更新tail指针
        tx_tail = (tx_tail + tx_dma_len) % UART_TX_BUF_SIZE;
        tx_dma_busy = 0;
        
        // 立即检查是否还有数据需要发送
        uart_tx_start_dma();
    }
}

