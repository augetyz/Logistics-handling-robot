#include "usart.h"
#include "JY61.h"

static void NVIC_Configuration_USART1(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}
static void NVIC_Configuration_USART2(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}
static void NVIC_Configuration_USART3(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* 配置USART为中断源 */
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  /* 抢断优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  /* 子优先级为1 */
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  /* 使能中断 */
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  /* 初始化配置NVIC */
  NVIC_Init(&NVIC_InitStructure);
}

 /**
  * @brief  DEBUG_USART GPIO 配置,工作模式配置。115200 8-N-1 ，中断接收模式
  * @param  无
  * @retval 无
  */
void USART1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
        
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

    /* 使能 USART 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* GPIO初始化 */
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    /* 配置Tx引脚为复用功能  */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9  ;  

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置Rx引脚为复用功能 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 连接 PXx 到 USARTx_Tx*/
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);

    /*  连接 PXx 到 USARTx__Rx*/
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource9 ,GPIO_AF_USART1);

    /* 配置串DEBUG_USART 模式 */
    /* 波特率设置：DEBUG_USART_BAUDRATE */
    USART_InitStructure.USART_BaudRate   = 115200;
    /* 字长(数据位+校验位)：8 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    /* 停止位：1个停止位 */
    USART_InitStructure.USART_StopBits   = USART_StopBits_1;
    /* 校验位选择：不使用校验 */
    USART_InitStructure.USART_Parity     = USART_Parity_No;
    /* 硬件流控制：不使用硬件流 */
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* USART模式控制：同时使能接收和发送 */
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    /* 完成USART初始化配置 */
    USART_Init(USART1, &USART_InitStructure); 

    /* 嵌套向量中断控制器NVIC配置 */
    NVIC_Configuration_USART1();

    /* 使能串口接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    /* 使能串口 */
    USART_Cmd(USART1, ENABLE);
}
void USART2_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE);

  /* 使能 UART 时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  
  /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);

  /*  连接 PXx 到 USARTx__Rx*/
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_USART2);

  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
			
  /* 配置串口RS232_USART 模式 */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure); 
	
	NVIC_Configuration_USART2();
	/*配置串口接收中断*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(USART2, ENABLE);
}
void USART3_Config(void)
{
       GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
		
  RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE);

  /* 使能 UART 时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  
  /* 连接 PXx 到 USARTx_Tx*/
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10, GPIO_AF_USART3);

  /*  连接 PXx 到 USARTx__Rx*/
GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_USART3);

  /* 配置Tx引脚为复用功能  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10  ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* 配置Rx引脚为复用功能 */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
			
  /* 配置串口RS232_USART 模式 */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure); 
	
	NVIC_Configuration_USART3();
	/*配置串口接收中断*/
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	
  USART_Cmd(USART3, ENABLE);
}
/*****************  发送一个字符 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
    do 
    {
        Usart_SendByte( pUSARTx, *(str + k) );
        k++;
    } while(*(str + k)!='\0');
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}
void Uart2Send(unsigned char *p_data, unsigned int uiSize)
{	
	unsigned int i;
	for(i = 0; i < uiSize; i++)
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, *p_data++);		
	}
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART1, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART1);
}
 void USART1_IRQHandler(void)
{
	uint8_t ucTemp;
	if (USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET) 
	{
		ucTemp = USART_ReceiveData( USART1 );
		USART_SendData(USART1,ucTemp);
        Usart_SendString(USART1,"USART1 ok\n");
	}
} 
 void USART2_IRQHandler(void)
{
	uint8_t ucTemp;
	if (USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET) 
	{
		ucTemp = USART_ReceiveData( USART2 );
		WitSerialDataIn(ucTemp);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}
 void USART3_IRQHandler(void)
{
	uint8_t ucTemp;
	if (USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET) 
	{
		ucTemp = USART_ReceiveData( USART3 );
		USART_SendData(USART3,ucTemp);
        Usart_SendString(USART1,"USART3 ok\n");
	}
}
/*********************************************END OF FILE**********************/
