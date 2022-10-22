/******************************************************************************
 *  Copyright © 2019 Shenzhen ECIOT Technology Co., Ltd All Rights Reserved
 *-----------------------------------------------------------------------------
 * @file        main.c
 * @brief       app demo
 * @author      mo@eciot.cn (qq:2201920828,v:eciot_mo)
 * @date        2022-02-13
 * @version     1.0
 ******************************************************************************/

#include "main.h"
#ifdef EC_APP_MAIN

#include "ec_core.h"
#include "ec_app_flash.h"
#include "ec_app_ble_peripheral.h"
#include "ec_app_ble.h"

#define EC_APP_UART0_TX_BUF_SIZE 1024                 //串口0发送缓冲区大小，可以根据需要调整
#define EC_APP_UART0_RX_BUF_SIZE 1024                 //串口0接收缓冲区大小，可以根据需要调整
#define EC_APP_UART1_TX_BUF_SIZE 1024                 //串口1发送缓冲区大小，可以根据需要调整
#define EC_APP_UART1_RX_BUF_SIZE 1024                 //串口1接收缓冲区大小，可以根据需要调整
uint8_t uart0_tx_buf[EC_APP_UART0_TX_BUF_SIZE] = {0}; //串口0发送缓冲区
uint8_t uart0_rx_buf[EC_APP_UART0_RX_BUF_SIZE] = {0}; //串口0接收缓冲区
uint8_t uart1_tx_buf[EC_APP_UART1_TX_BUF_SIZE] = {0}; //串口1发送缓冲区
uint8_t uart1_rx_buf[EC_APP_UART1_RX_BUF_SIZE] = {0}; //串口1接收缓冲区

void uart0_rx(uint8_t *buf, uint16_t len)
{
    ec_core_ble_send(buf, len); //串口数据转发到蓝牙

    if (strcmp((const char *)buf, "DISC") == 0)
        ec_core_ble_disconnect(); //主动断开蓝牙连接
}

void uart1_rx(uint8_t *buf, uint16_t len) //串口1接收数据中断
{
    if ((buf[0] == 'A') && (buf[1] == 'T'))
    {
        // The AT command is not supported.
        ec_core_uart1_printf("The AT command is not supported.\r\n");
    }

    ec_core_ble_send(buf, len); //串口数据转发到蓝牙
}

void uart1_init(void) //串口1初始化 波特率精度受时钟频率影响
{
    ec_core_uart_init(EC_CORE_UART1, 115200, EC_CORE_UART_PARITY_NONE,
                      EC_CORE_GPIO_P2, EC_CORE_GPIO_P1,
                      uart1_tx_buf, EC_APP_UART1_TX_BUF_SIZE, uart1_rx_buf, EC_APP_UART1_RX_BUF_SIZE,
                      uart1_rx);
}

typedef int bool;
#define true 1
#define false 0;


#define NUM 10000000

typedef struct
{
    float x;
    float y;
} Point;

float fRand(float fMin, float fMax)
{
    float f = (float) rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

Point pRand()
{
    float x, y;
    Point p;

    do
    {
        x = fRand(-1, 1);
        y = fRand(-1, 1);
    }
    while (x * x + y * y < 1);

    p.x = x;
    p.y = y;
    return p;
}

float cross_3(Point a, Point b)
{
    return a.x * b.y - b.x * a.y;
}

bool isSameSign(float *n_array, int len)
{
    for (int k = 0; k < len; k++)
    {
        for (int j = k; j < len; j++)
        {
            if (n_array[k] * n_array[j] < 0)
            {
                return false;
            }
        }
    }

    return true;
}



bool isInSemiCircle(Point *p_array, int len)
{
    float *c3 = (float *)(malloc((len - 1) * sizeof(float)));

    for(int j = 0; j < len; j++)
    {
        int index = 0;

        for (int k = 0; k < len; k++)
        {
            if(k != j)
            {
                c3[index] = cross_3(p_array[j], p_array[k]);
                index++;
            }
        }

        bool result = isSameSign(c3, len - 1);

        if(result)
        {
            free(c3);
            return true;
        }
    }

    free(c3);
    return false;
}


char sendBuf[100];
int v1 = 0;
int s1 = 0;
int s2 = 0;
Point *p_array;
int p_num = 4;
static ec_core_sw_timer_e A_timer = EC_CORE_SW_TIMER6;
static void A_task(void)
{
    for (int j = 0; j < p_num; j++)
    {
        p_array[j] = pRand();
    }

    if (isInSemiCircle(p_array, p_num))
    {
        s1++;
    }

    s2++;

    if(s2 % 100 == 0)
    {

        sprintf(sendBuf, "%d   %d", s1, s2);
        v1++;
        ec_core_ble_send(sendBuf, strlen(sendBuf));
    }

}




int main(void)
{
    p_array = (Point *)(malloc(p_num * sizeof(Point)));
    ec_core_sys_clk_set(EC_CORE_SYS_CLK_48M); //配置系统时钟

    ec_app_flash_sys_param_read(); // 从flash读取系统参数
    ec_app_ble_param_init();       // 初始化蓝牙相关的参数

    ec_core_init(); //蓝牙内核初始化

    //串口0初始化 波特率精度受时钟频率影响
    ec_core_uart_init(EC_CORE_UART0, 115200, EC_CORE_UART_PARITY_NONE,
                      EC_CORE_GPIO_P4, EC_CORE_GPIO_P5,
                      uart0_tx_buf, 1024, uart0_rx_buf, 1024,
                      uart0_rx);
    uart1_init();

    uint8_t ver[3] = {0};
    ec_core_ver(ver);                                                       //读取软件版本
    ec_core_uart0_printf("ECB02 SDK %d.%d.%d\r\n", ver[0], ver[1], ver[2]); //串口0 printf打印

    ec_core_sw_watchdog_init(EC_CORE_SW_TIMER6, 2, 3); //初始化软件看门狗，广播超时时间2分钟，蓝牙连接超时时间3分钟

    ec_core_sleep_disable(); //禁止睡眠，串口可以接收数据





    ec_core_sw_timer_start(A_timer, 10, A_task);

    ec_core_main_loop_run(); //系统主循环
}

#endif
