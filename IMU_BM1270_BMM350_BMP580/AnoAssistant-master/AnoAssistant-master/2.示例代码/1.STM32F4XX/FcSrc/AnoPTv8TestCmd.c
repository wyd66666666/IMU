#include "AnoPTv8TestCmd.h"
#include "AnoPTv8Run.h"

uint8_t 	TestCmdVal0 = 0;
int8_t 		TestCmdVal1 = 0;
uint16_t 	TestCmdVal2 = 0;
int16_t 	TestCmdVal3 = 0;

static void AnoPTv8CmdFun_Test1(void)
{
	//需要参数的命令，这个例子有4个参数
    AnoPTv8SendStr(ANOPTV8DEVID_SWJ, LOG_COLOR_GREEN, "Cmd Test1 Call");
	TestCmdVal0 = AnoPTv8CurrentAnlFrame.data[3];
	TestCmdVal1 = AnoPTv8CurrentAnlFrame.data[4];
	TestCmdVal2 = *(uint16_t *)&AnoPTv8CurrentAnlFrame.data[5];
	TestCmdVal3 = *(int16_t *)&AnoPTv8CurrentAnlFrame.data[7];
}

static void AnoPTv8CmdFun_Test2(void)
{
	//不需要参数的命令，只需要触发即可
    AnoPTv8SendStr(ANOPTV8DEVID_SWJ, LOG_COLOR_GREEN, "Cmd Test2 Call");
}

//定义命令信息结构体
const _st_cmd_info _pCmdInfoList[] = {
	{{0x01,0x01,0x01,0x01,0x02,0x03,0x04,0x00,0x00,0x00,0x00},"CmdTest1","CmdTest1 need cmd vals",AnoPTv8CmdFun_Test1},
	{{0x01,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},"CmdTest2","CmdTest2 no need cmd vals",AnoPTv8CmdFun_Test2},
};

void AnoCommandInit(void)
{
	static u8 _alreadyInit = 0;
    if(_alreadyInit)
        return;
    _alreadyInit = 1;

    int _c = sizeof(_pCmdInfoList) / sizeof(_st_cmd_info);
    for(int i=0; i<_c; i++)
        AnoPTv8CmdRegister(&_pCmdInfoList[i]);
}
