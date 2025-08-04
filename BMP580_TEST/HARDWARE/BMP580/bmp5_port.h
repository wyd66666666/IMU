#ifndef BMP5_PORT_H_
#define BMP5_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bmp5.h"

//BMP580测试程序
void BMP580_Test_Run();


// 声明您将要实现的平台适配函数
BMP5_INTF_RET_TYPE bmp5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);
BMP5_INTF_RET_TYPE bmp5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);
void bmp5_delay_us(uint32_t period_us, void *intf_ptr);

#ifdef __cplusplus
}
#endif

#endif // BMP5_PORT_H_


