#include "GasSensorTest.h"
#include "usart.h"



/**
 * @brief   ����DTUģ������ - ����ģʽ����
 * @details �Բ���ָ����Ӧ�ڵ������֡�����DTU��ɲ��Ժ��������Խ��
 * @param   ��
 * @return  ��
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
 * @brief   ����DTUģ������ - ͸����������
 * @details �Ե���ָ����Ӧ�ڵ������֡������͸��ģʽ��ģ��������DTU�佨��͸��ͨ��
 * @param   ��
 * @return  ��
 */
void FactoryDebugMode_Run(void)
{
	
}
	