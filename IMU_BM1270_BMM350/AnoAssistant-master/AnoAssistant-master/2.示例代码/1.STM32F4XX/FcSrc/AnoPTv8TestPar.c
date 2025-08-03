#include "AnoPTv8TestPar.h"
//
float 	TestParFloat[5] = {1.1, 2.2, 3.3, 4.4, 5.5};
int32_t TestParInt32[5] = {11, 22, 33, 44, 55};

const _st_par_info _parInfoList[] = {
    //参数指针：最小值：最大值：缩放倍数：参数名称：参数信息
    //缩放倍数：0：int32型参数：非0：Float型参数
    {&TestParFloat[0],		-100,	100,	10,	"TestParF0", 		"测试参数0，float型"},
	{&TestParFloat[1],		-100,	100,	10,	"TestParF1", 		"测试参数1，float型"},
	{&TestParFloat[2],		-100,	100,	10,	"TestParF2", 		"测试参数2，float型"},
	{&TestParFloat[3],		-100,	100,	10,	"TestParF3", 		"测试参数3，float型"},
	{&TestParFloat[4],		-100,	100,	10,	"TestParF4", 		"测试参数4，float型"},
	
	{&TestParInt32[0],		-100,	100,	0,	"TestParI0", 		"测试参数0，int32型"},
	{&TestParInt32[1],		-100,	100,	0,	"TestParI1", 		"测试参数1，int32型"},
	{&TestParInt32[2],		-100,	100,	0,	"TestParI2", 		"测试参数2，int32型"},
	{&TestParInt32[3],		-100,	100,	0,	"TestParI3", 		"测试参数3，int32型"},
	{&TestParInt32[4],		-100,	100,	0,	"TestParI4", 		"测试参数4，int32型"},
};

void AnoParameterInit(void)
{
    static u8 _alreadyInit = 0;
    if(_alreadyInit)
        return;
    _alreadyInit = 1;

    int _c = sizeof(_parInfoList) / sizeof(_st_par_info);
    for(int i=0; i<_c; i++)
        AnoPTv8ParRegister(&_parInfoList[i]);
}
