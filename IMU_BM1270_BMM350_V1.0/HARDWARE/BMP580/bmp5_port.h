#ifndef BMP5_PORT_H_
#define BMP5_PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "bmp5.h"

int8_t BMP580_Init(void);
int8_t BMP580_ReadData(float *temperature, float *pressure);

#ifdef __cplusplus
}
#endif

#endif // BMP5_PORT_H_


