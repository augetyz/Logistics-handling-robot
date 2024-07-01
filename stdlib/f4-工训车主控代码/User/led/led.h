#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

//���Ŷ���
/*******************************************************/
//R ��ɫ��
#define LED1_PIN                  GPIO_Pin_6                 
#define LED1_GPIO_PORT            GPIOF                      
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOF

//G ��ɫ��
#define LED2_PIN                  GPIO_Pin_7                 
#define LED2_GPIO_PORT            GPIOF                      
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOF

//B ��ɫ��
#define LED3_PIN                  GPIO_Pin_8                 
#define LED3_GPIO_PORT            GPIOF                       
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOF
/************************************************************/


/** ����LED������ĺ꣬
	* LED�͵�ƽ��������ON=0��OFF=1
	* ��LED�ߵ�ƽ�����Ѻ����ó�ON=1 ��OFF=0 ����
	*/
#define ON  0
#define OFF 1

/* ���κ꣬��������������һ��ʹ�� */
#define LED1(a)	if (a)	\
					GPIO_SetBits(LED1_GPIO_PORT,LED1_PIN);\
					else		\
					GPIO_ResetBits(LED1_GPIO_PORT,LED1_PIN)

#define LED2(a)	if (a)	\
					GPIO_SetBits(LED2_GPIO_PORT,LED2_PIN);\
					else		\
					GPIO_ResetBits(LED2_GPIO_PORT,LED2_PIN)

#define LED3(a)	if (a)	\
					GPIO_SetBits(LED3_GPIO_PORT,LED3_PIN);\
					else		\
					GPIO_ResetBits(LED3_GPIO_PORT,LED3_PIN)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)			 {p->BSRRL=i;}		//����Ϊ�ߵ�ƽ
#define digitalLo(p,i)			 {p->BSRRH=i;}		//����͵�ƽ
#define digitalToggle(p,i)	 {p->ODR ^=i;}		//�����ת״̬

/* �������IO�ĺ� */
#define LED1_TOGGLE		digitalToggle(LED1_GPIO_PORT,LED1_PIN)
#define LED1_OFF			digitalHi(LED1_GPIO_PORT,LED1_PIN)
#define LED1_ON				digitalLo(LED1_GPIO_PORT,LED1_PIN)

#define LED2_TOGGLE		digitalToggle(LED2_GPIO_PORT,LED2_PIN)
#define LED2_OFF			digitalHi(LED2_GPIO_PORT,LED2_PIN)
#define LED2_ON				digitalLo(LED2_GPIO_PORT,LED2_PIN)

#define LED3_TOGGLE		digitalToggle(LED3_GPIO_PORT,LED3_PIN)
#define LED3_OFF			digitalHi(LED3_GPIO_PORT,LED3_PIN)
#define LED3_ON				digitalLo(LED3_GPIO_PORT,LED3_PIN)

/* ������ɫ������߼��÷�ʹ��PWM�ɻ��ȫ����ɫ,��Ч������ */

//��
#define LED_RED  \
					LED1_ON;\
					LED2_OFF;\
					LED3_OFF

//��
#define LED_GREEN		\
					LED1_OFF;\
					LED2_ON;\
					LED3_OFF

//��
#define LED_BLUE	\
					LED1_OFF;\
					LED2_OFF;\
					LED3_ON

					
//��(��+��)					
#define LED_YELLOW	\
					LED1_ON;\
					LED2_ON;\
					LED3_OFF
//��(��+��)
#define LED_PURPLE	\
					LED1_ON;\
					LED2_OFF;\
					LED3_ON

//��(��+��)
#define LED_CYAN \
					LED1_OFF;\
					LED2_ON;\
					LED3_ON
					
//��(��+��+��)
#define LED_WHITE	\
					LED1_ON;\
					LED2_ON;\
					LED3_ON
					
//��(ȫ���ر�)
#define LED_RGBOFF	\
					LED1_OFF;\
					LED2_OFF;\
					LED3_OFF		

#define COLOR_TIM_GPIO_CLK             (RCC_AHB1Periph_GPIOF)

/************���***************/
#define COLOR_RED_TIM           						TIM10
#define COLOR_RED_TIM_CLK       						RCC_APB2Periph_TIM10
#define COLOR_RED_TIM_APBxClock_FUN        RCC_APB2PeriphClockCmd
/*����˵����c�ļ�*/
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2��ע��Ҫ������ͨ���Ķ�ʱ��Ƶ������Ϊһ��*/
#define COLOR_RED_TIM_PRESCALER						(((SystemCoreClock)/1000000)*30-1)

/************�̵�***************/
#define COLOR_GREEN_TIM           						TIM11
#define COLOR_GREEN_TIM_CLK       						RCC_APB2Periph_TIM11
#define COLOR_GREEN_TIM_APBxClock_FUN        RCC_APB2PeriphClockCmd
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2��ע��Ҫ������ͨ���Ķ�ʱ��Ƶ������Ϊһ��*/
#define COLOR_GREEN_TIM_PRESCALER						(((SystemCoreClock)/1000000)*30-1)

/************����***************/
#define COLOR_BLUE_TIM           							TIM13
#define COLOR_BLUE_TIM_CLK       						RCC_APB1Periph_TIM13
#define COLOR_BLUE_TIM_APBxClock_FUN        RCC_APB1PeriphClockCmd
/*����ͨ�ö�ʱ����ʱ��ΪHCLK/4������ΪHCLK/2��ע��Ҫ������ͨ���Ķ�ʱ��Ƶ������Ϊһ��*/
#define COLOR_BLUE_TIM_PRESCALER						(((SystemCoreClock/2)/1000000)*30-1)



/************���***************/

#define COLOR_RED_PIN                  GPIO_Pin_6                 
#define COLOR_RED_GPIO_PORT            GPIOF                      
#define COLOR_RED_PINSOURCE				GPIO_PinSource6
#define COLOR_RED_AF					GPIO_AF_TIM10

#define  COLOR_RED_TIM_OCxInit                TIM_OC1Init            //ͨ����ʼ������
#define  COLOR_RED_TIM_OCxPreloadConfig       TIM_OC1PreloadConfig //ͨ���������ú���

//ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
//�Ժ��װ��ʹ��������ʽ��COLOR_TIMx->COLOR_RED_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define  COLOR_RED_CCRx                       	CCR1		

/************�̵�***************/
#define COLOR_GREEN_PIN                  GPIO_Pin_7                 
#define COLOR_GREEN_GPIO_PORT            GPIOF                      
#define COLOR_GREEN_PINSOURCE						GPIO_PinSource7
#define COLOR_GREEN_AF										GPIO_AF_TIM11

#define  COLOR_GREEN_TIM_OCxInit                TIM_OC1Init            //ͨ����ʼ������
#define  COLOR_GREEN_TIM_OCxPreloadConfig       TIM_OC1PreloadConfig //ͨ���������ú���

//ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
//�Ժ��װ��ʹ��������ʽ��COLOR_TIMx->COLOR_GREEN_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define  COLOR_GREEN_CCRx                       CCR1

/************����***************/
#define COLOR_BLUE_PIN                  GPIO_Pin_8                 
#define COLOR_BLUE_GPIO_PORT            GPIOF                       
#define COLOR_BLUE_PINSOURCE						GPIO_PinSource8
#define COLOR_BLUE_AF										GPIO_AF_TIM13

#define   COLOR_BLUE_TIM_OCxInit              TIM_OC1Init            //ͨ����ʼ������
#define   COLOR_BLUE_TIM_OCxPreloadConfig    TIM_OC1PreloadConfig  //ͨ���������ú���

//ͨ���ȽϼĴ�������TIMx->CCRx��ʽ�ɷ��ʸüĴ����������µıȽ�ֵ������ռ�ձ�
//�Ժ��װ��ʹ��������ʽ��COLOR_TIMx->COLOR_BLUE_CCRx ���ɷ��ʸ�ͨ���ıȽϼĴ���
#define   COLOR_BLUE_CCRx                      CCR1


/************************************************************/

/* RGB��ɫֵ 24λ*/
#define COLOR_WHITE          0xFFFFFF
#define COLOR_BLACK          0x000000
#define COLOR_GREY           0xC0C0C0
#define COLOR_BLUE           0x0000FF
#define COLOR_RED            0xFF0000
#define COLOR_MAGENTA        0xFF00FF
#define COLOR_GREEN          0x00FF00
#define COLOR_CYAN           0x00FFFF
#define COLOR_YELLOW         0xFFFF00


void LED_GPIO_Config(void);
void ColorLED_Config(void);
void SetRGBColor(uint32_t rgb);
void SetColorValue(uint8_t r,uint8_t g,uint8_t b);
#endif /* __LED_H */
