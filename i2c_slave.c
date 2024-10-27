#include "i2c_slave.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

extern I2C_HandleTypeDef hi2c1;
extern int flag;
extern uint8_t I2C_recvBuf[3];



void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{

	
	
	flag = 1;

	HAL_I2C_EnableListen_IT(&hi2c1);
	

}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{


	
	if (TransferDirection == I2C_DIRECTION_TRANSMIT)  // 主机发送数据
	{
		HAL_I2C_Slave_Seq_Receive_IT(hi2c, I2C_recvBuf, sizeof(I2C_recvBuf), I2C_FIRST_AND_LAST_FRAME);
		flag = 9999;
		
	}
	else  // 如果主机尝试接收数据，处理错误
	{
		Error_Handler();
	}

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{

	
	
	flag = 5;

	HAL_I2C_EnableListen_IT(hi2c);  // 重新启用监听，准备接收下一条数据
	
	

}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{

	
	 
	flag = 7;
	// 打印或记录错误信息以供调试
	printf("I2C Error Code: %d\n", hi2c->ErrorCode);
//	HAL_Init();
//	SystemClock_Config();
//	MX_I2C1_Init();
	HAL_I2C_EnableListen_IT(hi2c);  // 重新启用监听，以防错误后无法继续工作
	

}






