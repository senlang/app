#include "stm32f10x.h"
#include "key.h"
#include "sys.h" 
#include "delay.h"


//STM32F103���İ�����
//�⺯���汾����
/************** Ƕ��ʽ������  **************/
/********** mcudev.taobao.com ��Ʒ  ********/


//////////////////////////////////////////////////////////////////////////////////	 

//STM32������
//������������	   
						  
//////////////////////////////////////////////////////////////////////////////////  
								    
//������ʼ������
void KEY_Init(void) //IO��ʼ��
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;
	//��ʼ��KEY1-->PA0��������
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��PAʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA


}
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������


static u8 key_up=1;//�������ɿ���־

u8 KEY_Scan(u8 mode)
{	 
	
	if(mode)key_up=1;  //֧������		  
	if(key_up&&(KEY1==0))
	{
		delay_ms(10);//ȥ���� 
		key_up=0;
		if(KEY1==0) return 1;

	}else if(KEY1==1)key_up=1; 	    
 	return 0;// �ް�������
}
