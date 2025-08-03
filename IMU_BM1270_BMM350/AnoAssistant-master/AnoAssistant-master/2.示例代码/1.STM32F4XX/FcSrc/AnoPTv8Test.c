#include "AnoPTv8Test.h"
#include <math.h>

void AnoPTv8TxFrameF1(uint8_t _u8, uint16_t _u16, int16_t _s16, float _f)
{
	uint8_t databuf[9];
	databuf[0] = _u8;
	databuf[1] = BYTE0(_u16);
	databuf[2] = BYTE1(_u16);
	databuf[3] = BYTE0(_s16);
	databuf[4] = BYTE1(_s16);
	databuf[5] = BYTE0(_f);
	databuf[6] = BYTE1(_f);
	databuf[7] = BYTE2(_f);
	databuf[8] = BYTE3(_f);
	
	AnoPTv8SendBuf(ANOPTV8DEVID_SWJ, 0xF1, databuf, 9);
}

void AnoPTv8TxFrameF2(void)
{
	static uint16_t scnt = 0;
	scnt++;
	if(scnt >= 360)
		scnt = 0;
	
	uint16_t v1 = scnt;
	float v2 = sin((double)scnt / 180 * 3.14159) * 180 + 180;
	float v3 = sin((double)(scnt+120) / 180 * 3.14159) * 180 + 180;
	float v4 = sin((double)(scnt+240) / 180 * 3.14159) * 180 + 180;
	
	
	uint8_t databuf[14];
	databuf[0] = BYTE0(v1);
	databuf[1] = BYTE1(v1);
	databuf[2] = BYTE0(v2);
	databuf[3] = BYTE1(v2);
	databuf[4] = BYTE2(v2);
	databuf[5] = BYTE3(v2);
	databuf[6] = BYTE0(v3);
	databuf[7] = BYTE1(v3);
	databuf[8] = BYTE2(v3);
	databuf[9] = BYTE3(v3);
	databuf[10] = BYTE0(v4);
	databuf[11] = BYTE1(v4);
	databuf[12] = BYTE2(v4);
	databuf[13] = BYTE3(v4);
	
	AnoPTv8SendBuf(ANOPTV8DEVID_SWJ, 0xF2, databuf, 14);
}





