/**
 * @file protocol_test.c
 * @brief 匿名协议测试文件
 * @author Claude
 * @date 2025-08-03
 */

#include "anony_protocol.h"
#include <stdio.h>

/**
 * @brief 测试协议发送功能
 */
void ANO_Protocol_Test(void)
{
    printf("=== 匿名协议V8测试开始 ===\r\n");
    
    // 测试姿态角发送
    printf("测试姿态角发送(功能码0x03)...\r\n");
    ANO_Send_Attitude(10.5f, -5.2f, 180.0f);
    
    // 测试惯性传感器数据发送
    printf("测试惯性传感器数据发送(功能码0x01)...\r\n");
    ANO_Send_InertialSensor(100, -50, 981, 1000, -500, 200, 0);
    
    // 测试磁力计温度数据发送
    printf("测试磁力计温度数据发送(功能码0x02)...\r\n");
    ANO_Send_MagTempSensor(150, -200, 300, 250, 1);
    
    // 测试用户自定义数据发送
    printf("测试用户自定义数据发送(功能码0xF1)...\r\n");
    ANO_Send_UserData(1.23f, -4.56f, 7.89f, 0.12f, -3.45f);
    
    printf("=== 匿名协议V8测试完成 ===\r\n");
}

/**
 * @brief 校验和测试
 */
void ANO_Checksum_Test(void)
{
    uint8_t test_data[] = {0xAB, 0xDD, 0xFE, 0x03, 0x07, 0x00, 0x94, 0x01, 0xEB, 0xFF, 0x1C, 0x4B, 0x01};
    uint16_t len = sizeof(test_data);
    
    uint8_t sum_check = ANO_Calculate_SumCheck(test_data, len);
    uint8_t add_check = ANO_Calculate_AddCheck(test_data, len);
    
    printf("校验和测试:\r\n");
    printf("数据长度: %d\r\n", len);
    printf("和校验: 0x%02X\r\n", sum_check);
    printf("附加校验: 0x%02X\r\n", add_check);
}