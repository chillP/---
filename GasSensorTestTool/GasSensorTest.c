#include "GasSensorTest.h"
#include "usart.h"



/**
 * @brief   霍尼DTU模拟网关 - 测试模式运行
 * @details 以测试指令响应节点的握手帧，配合DTU完成测试后输出其测试结果
 * @param   无
 * @return  无
 */
void FactoryTestMode_Run(void)
{
	while(1)
	{
		DEBUG_Printf("***Test Mode***\r\n");
		HAL_Delay(5000);
	}
}

/**
 * @brief   霍尼DTU模拟网关 - 透传调试运行
 * @details 以调试指令响应节点的握手帧，进入透传模式后，模拟网关与DTU间建立透传通道
 * @param   无
 * @return  无
 */
void FactoryDebugMode_Run(void)
{
	
}
	