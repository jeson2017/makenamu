/****************************************************************************
* Copyright (C), 2013 ���ϻ�����
*
* �ļ���: sensers.c
* ���ݼ���:
*       ����������˲���10DOFоƬ�ĺ������������ú����ݶ�ȡ����Ҫע�����
*       ���ù��̻���printf�����ؽ�չ���
*
* �ļ���ʷ:
* �汾��  ����       ����    ˵��
* v0.1    2013-1-5   �Ÿ�  �������ļ�
****************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stdio.h"

/* data -----------------------------------------------------------*/
//��������
//BMP085��E2PROM�洢�ı궨����
short AC1=0;   			//(0xAA,0xAB)
short AC2=0;  			//(0xAC,0xAD)
short AC3=0;  			//(0xAE,0xAF)
unsigned short AC4=0;				//(0xB0,0xB1)
unsigned short AC5=0;				//(0xB2,0xB3)
unsigned short AC6=0;				//(0xB4,0xB5)
short B1=0;				//(0xB6,0xB7)
short B2=0;				//(0xB8,0xB9)
short MB=0;				//(0xBA,0xBB)
short MC=0;				//(0xBC,0xBD)
short MD=0;				//(0xBE,0xBF)

long T=0;				//�¶�
long P=0;				//ѹ��

/* Private define ------------------------------------------------------------*/
#define BMP085Write (0xee)
#define BMP085Read  (0xef)

extern void I2C_delay(void);
extern bool I2C_Start(void);
extern void I2C_Stop(void);
extern void I2C_Ack(void);
extern void I2C_NoAck(void);
extern bool I2C_WaitAck(void);
extern void I2C_SendByte(u8 SendByte);
extern u8 I2C_ReceiveByte(void);
extern void Delay(__IO uint32_t nCount); 

/*******************************************************************
�����ӵ�ַ�������Ͷ��ֽ����ݺ���
����ԭ��: bit   ISendStr(uchar sla,uchar suba,ucahr *s,uchar no);  
����:      ���������ߵ����͵�ַ���ӵ�ַ,���ݣ��������ߵ�ȫ����,������
           ��ַsla���ӵ�ַsuba������������sָ������ݣ�����no���ֽڡ�
           �������1��ʾ�����ɹ��������������
ע�⣺     ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
bool I2C_WriteData(u8 byAD, u8 byRA, u8 * pData, u8 byCount)
{
    u8 i = 0;
    //�������ߣ����ɹ�������
    if(!I2C_Start())
	{
	    return FALSE;
    }                               
	//����������ַ�����ȴ���Ӧ
    I2C_SendByte(byAD);             
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}
	//���������Ĵ�����ַ�����ȴ���Ӧ
    I2C_SendByte(byRA);             
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}
	//��Ĵ����������ݣ����ù���
    for(i = 0; i < byCount; i++)
    { 
	    //һ���ֽ�һ���ֽڵط���  
        I2C_SendByte(*pData);             
        if(!I2C_WaitAck())
	    {
		    I2C_Stop();
		    return FALSE;
	    }
        pData++;
    }
	//���߽��� 
    I2C_Stop();                  

    return TRUE;
}

/*******************************************************************
�����ӵ�ַ������ȡ���ֽ����ݺ���               
����ԭ��: bool I2C_ReadData(u8 byAD, u8 byRA,u8 * pData, u8 byCount)  
����:      ���������ߵ����͵�ַ���ӵ�ַ,�����ݣ��������ߵ�ȫ����
           �������1��ʾ�����ɹ��������������
ע�⣺     ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
bool I2C_ReadData(u8 byAD, u8 byRA,u8 * pData, u8 byCount)
{
    u8 i = 0;
	//��������
    if(!I2C_Start())
	{
	    return FALSE;
    }
	//����������ַ                            
    I2C_SendByte(byAD);              
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}
	//���������Ĵ�����ַ
    I2C_SendByte(byRA);             
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}
	//������������
    if(!I2C_Start())
	{
	    return FALSE;
    }                               
	//������������ַ
    I2C_SendByte(byAD + 1);              
    if(!I2C_WaitAck())
	{
	    I2C_Stop();
	    return FALSE;
	}
	//��������
    for(i = 0; i < (byCount-1); i++)
    {
	    //����һ���ֽڵ����ݣ�������Ӧ��λ
        *pData=I2C_ReceiveByte();                
        I2C_Ack();                        
        pData++;
    }
	//���һ���ֽڽ��պ��ͷ�Ӧ��
    *pData=I2C_ReceiveByte();
    I2C_NoAck(); 
	//���߽���                  
    I2C_Stop(); 
	                
    return TRUE;
}


/*******************************BMP***********************************************************/
void SetBmpTP()
{
	bool flag = FALSE;
	u8 cmd = 0x2e; 

	flag =  I2C_WriteData(0xee, 0xf4, &cmd, 1);
	if(0 == flag)
	{
	   	printf("\r\n I2C bmp write fail!\n");
	}
}
/*******************************************************************
��BMP085оƬ�ж�ȡE2prom�б궨ֵ             
����ԭ��: bool I2C_ReadData(u8 byAD, u8 byRA,u8 * pData, u8 byCount)  
����:      ���������ߵ����͵�ַ���ӵ�ַ,�����ݣ��������ߵ�ȫ����
           �������1��ʾ�����ɹ��������������
ע�⣺     ʹ��ǰ�����ѽ������ߡ�
********************************************************************/
bool ReadBmp085E2prom()
{
    u8 memo[22];
	u8 i = 0;
	bool flag = FALSE;
	bool successflag = TRUE;

	//��EEPROM�ж�ȡԤ�����
	for(i=0;i<22;i++)
	{
	    flag = I2C_ReadData(0xee, 0xAA+i, &memo[i], 1);
		if(0 == flag)
	  	{
	    	//printf("\r\n E2PROM I2C %u Read OK!\n",i);
			successflag = FALSE;
	  	}
	}
	if(FALSE == successflag)
	{
	    printf("\r\n E2PROM I2C Read Fail!\n");
	}

	AC1 = (((short)memo[0])<<8)+((short)memo[1]);
	AC2 = (((short)memo[2])<<8)+((short)memo[3]);
	AC3 = (((short)memo[4])<<8)+((short)memo[5]);
	AC4 = (((short)memo[6])<<8)+((short)memo[7]);
	AC5 = (((short)memo[8])<<8)+((short)memo[9]);
	AC6 = (((short)memo[10])<<8)+((short)memo[11]);
	B1 = (((short)memo[12])<<8)+((short)memo[13]);
	B2 = (((short)memo[14])<<8)+((short)memo[15]);
	MB = (((short)memo[16])<<8)+((short)memo[17]);
	MC = (((short)memo[18])<<8)+((short)memo[19]);
	MD = (((short)memo[20])<<8)+((short)memo[21]);
	
	return TRUE;
}

/*******************************************************************           
����ԭ��: void GetBmpTP(u8 oss)  
����:      ��BMP085оƬ�ж�ȡ���º���ѹ 
ע�⣺  ossΪ������ʽ��
		//0-ultra low power;
		//1-standard;
		//2-high resolution;
		//3-ultra high resolution   
********************************************************************/
void GetBmpTP(u8 oss)
{
	long UT=0;			//�¶�ԭʼֵ	
	long UP=0;			//ѹ��ԭʼֵ
	long X1,X2,X3;
	long B3,B5,B6,B7;
	unsigned long B4;
	u8 i = 0;
	u8 BMPmemo[3] = {0,0,0};
	bool flag = FALSE;
	u8 cmd = 0;

	flag =  ReadBmp085E2prom();
	if(0 == flag)
	{
	   	printf("\r\n I2C E2PROM Read fail!\n");
	}

	//��������¶�
	cmd = 0x2e;
	flag =  I2C_WriteData(0xee, 0xf4, &cmd, 1);
    if(0 == flag)
  	{
    	printf("\r\n I2C bmp write fail!\n");
  	}
	Delay(0x8fff);						    	//�ȴ�AD���ӳ�4.5ms����
	for(i=0;i<2;i++)					        //��ȡ���
	{
		flag =  I2C_ReadData(0xee, 0xF6+i, &BMPmemo[i], 1);
		if(0 == flag)
	  	{
	    	printf("\r\n E2PROM I2C %u Read fail!\n",i);
	  	}
	}
	UT = (((long)BMPmemo[0])<<8)+((long)BMPmemo[1]);

	cmd = 0x34+(oss<<6);
	flag =  I2C_WriteData(0xee, 0xf4, &cmd, 1);
    if(0 == flag)
  	{
    	printf("\r\n I2C bmp write fail!\n");
  	}
	Delay(0x2ffff);								//�ӳ�ʱ���ӹ�����ʽ������������ֲ�
	for(i=0;i<3;i++)					        //��ȡ���
	{
		flag =  I2C_ReadData(0xee, 0xF6+i, &BMPmemo[i], 1);
		if(0 == flag)
	  	{
	    	printf("\r\n E2PROM I2C %u Read fail!\n",i);
	  	}
	}
	UP = (((long)BMPmemo[0])<<16) + (((long)BMPmemo[1])<<8) + ((long)BMPmemo[2]);
	UP=UP>>(8-oss);

	//�����¶ȣ���ʽ��datasheet
	X1=(UT-AC6)*AC5/32768; 	                 
	X2=MC;
	X2=X2*2048/(X1+MD);
	B5=X1+X2;
	T=(B5+8)/16;

	//����ѹ������ʽ��datasheet
	B6=B5-4000;			
	X1=B2;
	X1=(X1*(B6*B6/4096))/2048;
	X2=AC2;
	X2=X2*B6/2048;
	X3=X1+X2;
	B3=AC1;
	B3=(((B3*4+X3)<<oss)+2)/4;	
	X1=AC3;
	X1=X1*B6/8192;
	X2=B1;
	X2=(X2*(B6*B6/4096))/65536;
	X3=((X1+X2)+2)/4;
	B4=AC4;
	B4=B4*(unsigned long)(X3+32768)/32768;
	B7=((unsigned long)UP-B3)*(50000>>oss);	
	if(B7<0x80000000)
	{
	    P=(B7*2)/B4;
	}
	else 
	{
	    P=(B7/B4)*2;
	}
	X1=(P/256)*(P/256);
	X1=(X1*3038)/65536;
	X2=(-7357*P)/65536;
	P=P+(X1+X2+3791)/16;  

}

/******************************��L3G4200D�������Ĳ���******************************************/
/*******************************************************************           
����ԭ��: void SetL3GXYZ()  
����:     ����L3G4200DоƬ���Ʋ��� 
ע�⣺    
********************************************************************/
void SetL3GXYZ(void)
{
    //����800Hz���������жϺ�׼�����źţ�Ĭ��bypassģʽ
	u8 ControlWord[6] = {0x0f,0x00,0x00,0x80,0x00,0x00};
	bool flag = FALSE;
	
	flag =  I2C_WriteData(0xd0, 0x20 | 0x80, ControlWord, 6);
    if(0 == flag)
  	{
    	printf("\r\n I2C bmp write fail!\n");
  	}
}

/*******************************************************************           
����ԭ��: void GetL3GXYZ()  
����:     ��L3G4200DоƬ�ж�ȡ����ֵ 
ע�⣺    
********************************************************************/
void GetL3GXYZ(u16 *x,u16 *y,u16 *z)
{
	u8 data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	bool flag = FALSE;
	u16 temp = 0;

	flag =  I2C_ReadData(0xd0, (0x28 | 0x80), data, 6);
	if(0 == flag)
	{
	    printf("\r\n L3G I2C Read fail!\n");
	}
	//����x����ٶ�
	temp  = 0;
    temp = data[1] << 8;
    temp |= data[0];
    *x = temp;

	//����y����ٶ�
	temp  = 0;
    temp = data[3] << 8;
    temp |= data[2];
    *y = temp;

	//����z����ٶ�
	temp  = 0;
    temp = data[5] << 8;
    temp |= data[4];
    *z = temp;
}

/******************************��ADXL�������Ĳ���******************************************/
/*******************************************************************           
����ԭ��: void SetADXLXYZ()  
����:     ����HMC5883оƬ���Ʋ��� 
ע�⣺    
********************************************************************/
void SetADXLXYZ(void)
{
	u8 ControlWord[13] = {0xb8,0xf4,0xf1,0x00,0x00,0x00,0x01,0x01,0x2b,0x00,0x09,0xff,0x80};
	bool flag = FALSE;
	u8 cmd = 0;
	
	//���ü��ٶȼƸ��Ĵ���
	cmd = 0x28;
	flag =  I2C_WriteData(0xa6, 0x31, &cmd, 1);
    if(0 == flag)
  	{
    	printf("\r\n I2C adxl write1 fail!\n");
  	}

	flag =  I2C_WriteData(0xa6, 0x1e, ControlWord, 13);
    if(0 == flag)
  	{
    	printf("\r\n I2C adxl write2 fail!\n");
  	}

	ControlWord[0] = 0x0a;
	ControlWord[1] = 0x2b;
	ControlWord[2] = 0x00;
	ControlWord[3] = 0x00;
	flag =  I2C_WriteData(0xa6, 0x2c, ControlWord, 4);
    if(0 == flag)
  	{
    	printf("\r\n I2C adxl write2 fail!\n");
  	}

	cmd = 0x9f;
	flag =  I2C_WriteData(0xa6, 0x38, &cmd, 1);
    if(0 == flag)
  	{
    	printf("\r\n I2C adxl write2 fail!\n");
  	}
}

/*******************************************************************           
����ԭ��: void GetL3GXYZ()  
����:     ��L3G4200DоƬ�ж�ȡ����ֵ 
ע�⣺    
********************************************************************/
void GetADXLXYZ(u16 *x,u16 *y,u16 *z)
{
	u8 data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	bool flag = FALSE;
	u16 temp = 0;
	
	flag =  I2C_ReadData(0xa6, 0x32, data, 6);
	if(0 == flag)
	{
	    printf("\r\n L3G I2C Read fail!\n");
	}
	//����x����ٶ�
	temp  = 0;
    temp = data[1] << 8;
    temp |= data[0];
    *x = temp;

	//����y����ٶ�
	temp  = 0;
    temp = data[3] << 8;
    temp |= data[2];
    *y = temp;

	//����z����ٶ�
	temp  = 0;
    temp = data[5] << 8;
    temp |= data[4];
    *z = temp;
}

/******************************��HMC�������Ĳ���******************************************/
/*******************************************************************           
����ԭ��: void SetHMCXYZ()  
����:     ����L3G4200DоƬ���Ʋ��� 
ע�⣺    
********************************************************************/
void SetHMCXYZ()
{
	u8 ControlWord[3] = {0x70,0x00,0x00};
	bool flag = FALSE;
	
	flag =  I2C_WriteData(0x3c, 0x00, ControlWord, 3);
    if(0 == flag)
  	{
    	printf("\r\n I2C bmp write fail!\n");
  	}
}

/*******************************************************************           
����ԭ��: void GetL3GXYZ()  
����:     ��L3G4200DоƬ�ж�ȡ����ֵ 
ע�⣺    
********************************************************************/
void GetHMCXYZ(u16 *x,u16 *y,u16 *z)
{
	u8 data[6] = {0x00,0x00,0x00,0x00,0x00,0x00};
	bool flag = FALSE;
	u16 temp = 0;
	
	flag =  I2C_ReadData(0x3c, 0x03, data, 6);
	if(0 == flag)
	{
	    printf("\r\n L3G I2C Read fail!\n");
	}

	//����x����ٶ�
	temp  = 0;
    temp = data[0] << 8;
    temp |= data[1];
    *x = temp;

	//����y����ٶ�
	temp  = 0;
    temp = data[2] << 8;
    temp |= data[3];
    *y = temp;

	//����z����ٶ�
	temp  = 0;
    temp = data[4] << 8;
    temp |= data[5];
    *z = temp;
}







