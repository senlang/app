
//****************************************************************//
//DHT12���������ݶ�ȡ����
//****************************************************************//

#include "dht21.h"
#include "usart.h"

//������ʪ�ȱ��� ���˱���Ϊȫ�ֱ���
u8 Sensor_AnswerFlag=0;//���崫������Ӧ��־
u8 Sensor_ErrorFlag;  //�����ȡ�����������־


//��λDHT11
void DHT11_Rst(void)	   
{                 
	SDA_OUT(); 	//SET OUTPUT
    SEND_SDA=0; 	//����DQ
    delay_ms(20);    	//��������18ms
    SEND_SDA=1; 	//DQ=1 
	delay_us(30);     	//��������20~40us
}



/********************************************\
|* ���ܣ� ��ʼ��SDA(PB14),CLK(PB13)	        *|
\********************************************/
void DHT12_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );//ʹ��GPIOC	
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;//PC2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;   //��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_2); 	//PC2 �����(SCL�ϵ���������)
}


/********************************************\
|* ���ܣ� �����������͵ĵ����ֽ�	        *|
\********************************************/
u8 DHT12_Rdata(void)
{
	u8 i;
	u16 j;
	u8 data=0,bit=0;
	
	for(i=0;i<8;i++)
	{
		while(!READ_SDA)//����ϴε͵�ƽ�Ƿ����
		{
			if(++j>=50000) //��ֹ������ѭ��
			{
				break;
			}
		}
		//��ʱMin=26us Max70us ��������"0" �ĸߵ�ƽ		 
		delay_us(30);

		//�жϴ�������������λ
		bit=0;
		if(READ_SDA)
		{
			bit=1;
		}
		j=0;
		while(READ_SDA)	//�ȴ��ߵ�ƽ����
		{
			if(++j>=50000) //��ֹ������ѭ��
			{
				break;
			}		
		}
		data<<=1;
		data|=bit;
	}
	return data;
}


/********************************************\
|* ���ܣ�DHT12��ȡ��ʪ�Ⱥ���       *|
\********************************************/
//������Humi_H��ʪ�ȸ�λ��Humi_L��ʪ�ȵ�λ��Temp_H���¶ȸ�λ��Temp_L���¶ȵ�λ��Temp_CAL��У��λ
//���ݸ�ʽΪ��ʪ�ȸ�λ��ʪ��������+ʪ�ȵ�λ��ʪ��С����+�¶ȸ�λ���¶�������+�¶ȵ�λ���¶�С����+ У��λ
//У�飺У��λ=ʪ�ȸ�λ+ʪ�ȵ�λ+�¶ȸ�λ+�¶ȵ�λ
u8 DHT12_READ(int *Temprature, int *Humi)
{
	u32 j;
	u8 Humi_H,Humi_L,Temp_H,Temp_L,Temp_CAL,temp;
	int Temp_Temprature = 0, Temp_Humi = 0;
	u8 retval = 0;

	//����������ʼ�ź�
	SDA_OUT(); //��Ϊ���ģʽ
	SEND_SDA=0;	//�������������ߣ�SDA������
	delay_ms(2);//����һ��ʱ�䣨����800us���� ֪ͨ������׼������
	SEND_SDA=1;	 //�ͷ�����
	SDA_IN();	//��Ϊ����ģʽ���жϴ�������Ӧ�ź�
	delay_us(30);//��ʱ30us

	Sensor_AnswerFlag=0;	//��������Ӧ��־
	//�жϴӻ��Ƿ��е͵�ƽ��Ӧ�ź� �粻��Ӧ����������Ӧ����������	  
	if(READ_SDA==0)
	{
		Sensor_AnswerFlag=1;	//�յ���ʼ�ź�

		j=0;
		while((!READ_SDA)) //�жϴӻ����� 80us �ĵ͵�ƽ��Ӧ�ź��Ƿ����	
		{
			if(++j>=500) //��ֹ������ѭ��
			{
				Sensor_ErrorFlag=1;
				break;
			}
		}

		j=0;
		while(READ_SDA)//�жϴӻ��Ƿ񷢳� 80us �ĸߵ�ƽ���緢����������ݽ���״̬
		{
			if(++j>=800) //��ֹ������ѭ��
			{
				Sensor_ErrorFlag=1;
				break;
			}		
		}
		//��������
		Humi_H=DHT12_Rdata();
		Humi_L=DHT12_Rdata();
		Temp_H=DHT12_Rdata();	
		Temp_L=DHT12_Rdata();
		Temp_CAL=DHT12_Rdata();
		
		temp=(u8)(Humi_H+Humi_L+Temp_H+Temp_L);//ֻȡ��8λ

		UsartPrintf(USART_DEBUG, "%d \r%d \r%d \r%d \r%d \r%d \r\n",Humi_H,Humi_L,Temp_H,Temp_L,Temp_CAL,temp);

		if(Temp_CAL==temp)//���У��ɹ�����������
		{
			Temp_Humi=Humi_H*256 + Humi_L; //ʪ��
			
			if(Temp_H&0X80)	//Ϊ���¶�
			{
				Temp_Temprature =0 - ((Temp_H&0x7F)*256+Temp_L);
			}
			else   //Ϊ���¶�
			{
				Temp_Temprature = Temp_H * 256 + Temp_L;//Ϊ���¶�
			}
			
			//UsartPrintf(USART_DEBUG, "Temprature:  %f  ��\r\n",(float)Temp_Temprature/10); //��ʾ�¶�
			//UsartPrintf(USART_DEBUG, "Humi:  %f  ��\r\n",(float)Temp_Humi/10); //��ʾ�¶�
			
			//�ж������Ƿ񳬹����̣��¶ȣ�-20��~60�棬ʪ��20��RH~95��RH��
			if(Temp_Humi > 950) 
			{
			  Temp_Humi = 950;
			}
			else if(Temp_Humi < 100)
			{
				Temp_Humi = 100;
			}
			
			if(Temp_Temprature > 800)
			{
			  Temp_Temprature = 800;
			}
			else if(Temp_Temprature < -400)
			{
				Temp_Temprature = -400;
			}

			*Temprature = Temp_Temprature;
			*Humi = Temp_Humi;
			
			//UsartPrintf(USART_DEBUG, "\r\n�¶�Ϊ:  %d.%d  ��\r\n",*Temprature, *Temprature/10, *Temprature%10); //��ʾ�¶�
			//UsartPrintf(USART_DEBUG, "ʪ��Ϊ:  %d.%d  %%RH\r\n",*Humi, *Humi/10, *Humi%10);//��ʾʪ��	
			retval = 0;
		}
		else
		{
		 	UsartPrintf(USART_DEBUG, "CAL Error!!\r\n");
			UsartPrintf(USART_DEBUG, "%d \r%d \r%d \r%d \r%d \r%d \r\n",Humi_H,Humi_L,Temp_H,Temp_L,Temp_CAL,temp);
			retval = 1;
		}
	}
	else
	{
		Sensor_ErrorFlag=0;  //δ�յ���������Ӧ
		UsartPrintf(USART_DEBUG, "Sensor Error!!\r\n");
		retval = 1;
	}

	return retval;
}



