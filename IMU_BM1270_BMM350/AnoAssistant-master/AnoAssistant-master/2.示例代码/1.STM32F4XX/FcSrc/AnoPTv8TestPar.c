#include "AnoPTv8TestPar.h"
//
float 	TestParFloat[5] = {1.1, 2.2, 3.3, 4.4, 5.5};
int32_t TestParInt32[5] = {11, 22, 33, 44, 55};

const _st_par_info _parInfoList[] = {
    //����ָ�룺��Сֵ�����ֵ�����ű������������ƣ�������Ϣ
    //���ű�����0��int32�Ͳ�������0��Float�Ͳ���
    {&TestParFloat[0],		-100,	100,	10,	"TestParF0", 		"���Բ���0��float��"},
	{&TestParFloat[1],		-100,	100,	10,	"TestParF1", 		"���Բ���1��float��"},
	{&TestParFloat[2],		-100,	100,	10,	"TestParF2", 		"���Բ���2��float��"},
	{&TestParFloat[3],		-100,	100,	10,	"TestParF3", 		"���Բ���3��float��"},
	{&TestParFloat[4],		-100,	100,	10,	"TestParF4", 		"���Բ���4��float��"},
	
	{&TestParInt32[0],		-100,	100,	0,	"TestParI0", 		"���Բ���0��int32��"},
	{&TestParInt32[1],		-100,	100,	0,	"TestParI1", 		"���Բ���1��int32��"},
	{&TestParInt32[2],		-100,	100,	0,	"TestParI2", 		"���Բ���2��int32��"},
	{&TestParInt32[3],		-100,	100,	0,	"TestParI3", 		"���Բ���3��int32��"},
	{&TestParInt32[4],		-100,	100,	0,	"TestParI4", 		"���Բ���4��int32��"},
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
