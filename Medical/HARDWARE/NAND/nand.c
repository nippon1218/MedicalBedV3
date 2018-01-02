#include "nand.h"
#include "delay.h"
#include "malloc.h"
#include "usart.h"

NAND_HandleTypeDef NAND_Handler;    //NAND FLASH���
nand_attriute nand_dev;             //nand��Ҫ�����ṹ��


/***********************************************************************
 ������      ��HAL_NAND_MspInit(NAND_HandleTypeDef *hnand) 
 ��������    ��NAND FALSH�ײ�����,�������ã�ʱ��ʹ��
               �˺����ᱻHAL_NAND_Init()����
 ����        ��hnand
 ���        ����
                           
************************************************************************/
void HAL_NAND_MspInit(NAND_HandleTypeDef *hnand)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_FMC_CLK_ENABLE();                //ʹ��FMCʱ��
    __HAL_RCC_GPIOD_CLK_ENABLE();              //ʹ��GPIODʱ��
    __HAL_RCC_GPIOE_CLK_ENABLE();              //ʹ��GPIOEʱ��
    __HAL_RCC_GPIOG_CLK_ENABLE();              //ʹ��GPIOGʱ��
    
	//��ʼ��PD6 R/B����
	GPIO_Initure.Pin=GPIO_PIN_6;
    GPIO_Initure.Mode=GPIO_MODE_INPUT;          //����
    GPIO_Initure.Pull=GPIO_PULLUP;    			//����          
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
	   
	//��ʼ��PG9 NCE3����,FMC ���ߵ�Ƭѡ�ź� 3��Ϊ NAND Ƭѡ�ź�
    GPIO_Initure.Pin=GPIO_PIN_9;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //����
    GPIO_Initure.Pull=GPIO_NOPULL;    			//��������          
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;         //����
	GPIO_Initure.Alternate=GPIO_AF12_FMC;       //����ΪFMC
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);  
	
    //��ʼ��PD0,1,4,5,11,12,14,15
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|\
                     GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Pull=GPIO_NOPULL;               //��������              
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    //��ʼ��PE7,8,9,10
    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}


/*********************************************************************
*������       ��NAND_Init(void)
*��������     ����ʼ��NAND FLASH,������ؿ��Ʋ����� FMC ʱ��
*����         ����
*���         ����

***********************************************************************/
u8 NAND_Init(void)
{
    FMC_NAND_PCC_TimingTypeDef ComSpaceTiming,AttSpaceTiming;
    
    //���� NAND FLASH �Ŀ��Ʋ���	
    NAND_Handler.Instance=FMC_NAND_DEVICE;
    NAND_Handler.Init.NandBank=FMC_NAND_BANK3;                          //NAND����BANK3��
    NAND_Handler.Init.Waitfeature=FMC_NAND_PCC_WAIT_FEATURE_DISABLE;    //�رյȴ�����
    NAND_Handler.Init.MemoryDataWidth=FMC_NAND_PCC_MEM_BUS_WIDTH_8;     //�������߿��:8λ���ݿ��
    NAND_Handler.Init.EccComputation=FMC_NAND_ECC_DISABLE;              //��ʹ��ECC
    NAND_Handler.Init.ECCPageSize=FMC_NAND_ECC_PAGE_SIZE_2048BYTE;      //ECCҳ��СΪ2k
    NAND_Handler.Init.TCLRSetupTime=0;                                  //����TCLR(tCLR=CLE��RE����ʱ)=(TCLR+TSET+2)*THCLK,THCLK=1/180M=5.5ns
    NAND_Handler.Init.TARSetupTime=1;                                   //����TAR(tAR=ALE��RE����ʱ)=(TAR+TSET+2)*THCLK,THCLK=1/180M=5.5n��   
   
	//ComSpace_Timing �������� NAND ͨ�ô洢���ռ�ʱ��
    ComSpaceTiming.SetupTime=2;         //����ʱ��
    ComSpaceTiming.WaitSetupTime=3;     //�ȴ�ʱ��
    ComSpaceTiming.HoldSetupTime=2;     //����ʱ��
    ComSpaceTiming.HiZSetupTime=1;      //����̬ʱ��
   
	//AttSpace_Timing �������� NAND ���Դ洢���ռ�ʱ��
    AttSpaceTiming.SetupTime=2;         //����ʱ��
    AttSpaceTiming.WaitSetupTime=3;     //�ȴ�ʱ��
    AttSpaceTiming.HoldSetupTime=2;     //����ʱ��
    AttSpaceTiming.HiZSetupTime=1;      //����̬ʱ��
    
    HAL_NAND_Init(&NAND_Handler,&ComSpaceTiming,&AttSpaceTiming); 
    NAND_Reset();       		        //��λNAND
    delay_ms(100);
    nand_dev.id=NAND_ReadID();	        //��ȡID
	NAND_ModeSet(4);			        //����ΪMODE4,����ģʽ 
  
    if(nand_dev.id==MT29F4G08ABADA)//NANDΪMT29F4G08ABADA
    {
        nand_dev.page_totalsize=2112;	//nandһ��page���ܴ�С������spare����
        nand_dev.page_mainsize=2048; 	//nandһ��page����Ч��������С
        nand_dev.page_sparesize=64;		//nandһ��page��spare����С
        nand_dev.block_pagenum=64;		//nandһ��block��������page��Ŀ
        nand_dev.plane_blocknum=2048;	//nandһ��plane��������block��Ŀ
        nand_dev.block_totalnum=4096; 	//nand����block��Ŀ
    }else return 1;	//���󣬷���
    return 0;
}

/*********************************************************************
*������       ��NAND_ModeSet(u8 mode)
*��������     ����ȡNAND FLASH��ID
*����         ��mode
*���         ��0,�ɹ�;
                ����,ʧ��

***********************************************************************/
u8 NAND_ModeSet(u8 mode)
{   
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_FEATURE;//����������������
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X01;		//��ַΪ0X01,����mode
 	*(vu8*)NAND_ADDRESS=mode;					//P1����,����mode
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0;
	*(vu8*)NAND_ADDRESS=0; 
    if(NAND_WaitForReady()==NSTA_READY)return 0;//�ɹ�
    else return 1;								//ʧ��
}

/*********************************************************************
*������       ��NAND_ReadID(void)
*��������     ����ȡNAND FLASH��ID
*����         ����
*���         ��0,�ɹ�;
                ����,ʧ��

***********************************************************************/
u32 NAND_ReadID(void)
{
    u8 deviceid[5]; 
    u32 id;  
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READID;   //���Ͷ�ȡID����
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=0X00;
	//IDһ����5���ֽ�
    deviceid[0]=*(vu8*)NAND_ADDRESS;      
    deviceid[1]=*(vu8*)NAND_ADDRESS;  
    deviceid[2]=*(vu8*)NAND_ADDRESS; 
    deviceid[3]=*(vu8*)NAND_ADDRESS; 
    deviceid[4]=*(vu8*)NAND_ADDRESS;  
    //þ���NAND FLASH��IDһ��5���ֽڣ�����Ϊ�˷�������ֻȡ4���ֽ����һ��32λ��IDֵ
    //����NAND FLASH�������ֲᣬֻҪ��þ���NAND FLASH����ôһ���ֽ�ID�ĵ�һ���ֽڶ���0X2C
    //�������ǾͿ����������0X2C��ֻȡ�������ֽڵ�IDֵ��
    id=((u32)deviceid[1])<<24|((u32)deviceid[2])<<16|((u32)deviceid[3])<<8|deviceid[4];
    return id;
}  

/*********************************************************************
*������       ��NAND_ReadStatus(void)
*��������     ����NAND״̬
*����         ����
*���         ��NAND״ֵ̬
				bit0:0,�ɹ�;1,����(���/����/READ)
				bit6:0,Busy;1,Ready

***********************************************************************/
u8 NAND_ReadStatus(void)
{
    vu8 data=0; 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_READSTA;   //���Ͷ�״̬����
	data++;data++;data++;data++;data++;	           //����ʱ,��ֹ-O2�Ż�,���µĴ���.
 	data=*(vu8*)NAND_ADDRESS;			           //��ȡ״ֵ̬
    return data;
}

/*********************************************************************
*������       ��NAND_WaitForReady(void)
*��������     ���ȴ�NAND׼����
*����         ����
*���         ��NSTA_TIMEOUT �ȴ���ʱ��
				NSTA_READY    �Ѿ�׼����

***********************************************************************/
u8 NAND_WaitForReady(void)
{
    u8 status=0;
    vu32 time=0; 
	while(1)						            //�ȴ�ready
	{
		status=NAND_ReadStatus();	            //��ȡ״ֵ̬
		if(status&NSTA_READY)break;
		time++;
		if(time>=0X1FFFF)return NSTA_TIMEOUT;   //��ʱ
	}  
    return NSTA_READY;                          //׼����
}  

/*********************************************************************
*������       ��NAND_Reset(void)
*��������     ����λNAND
*����         ����
*���         ��0,�ɹ�;
				����,ʧ��

***********************************************************************/
u8 NAND_Reset(void)
{ 
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_RESET;	 //��λNAND
    if(NAND_WaitForReady()==NSTA_READY)return 0; //��λ�ɹ�
    else return 1;								 //��λʧ��
} 

/*********************************************************************
*������       ��NAND_WaitRB(void)
*��������     ���ȴ�RB�ź�Ϊĳ����ƽ
*����         ��rb:0,�ȴ�RB==0��1,�ȴ�RB==1
*���         ��0,�ɹ�;
				����,ʧ��

***********************************************************************/
u8 NAND_WaitRB(vu8 rb)
{
    vu16 time=0;  
	while(time<10000)
	{
		time++;
		if(NAND_RB==rb)return 0;
	}
	return 1;
}

/*********************************************************************
*������       ��NAND_Delay(vu32 i)
*��������     ��NAND��ʱ
*����         ��i
*���         ����

***********************************************************************/
void NAND_Delay(vu32 i)
{
	while(i>0)i--;
}

/*********************************************************************
*������       ��NAND_ReadPage()
*��������     ����ȡNAND Flash��ָ��ҳָ���е�����
*����         ��BlockNum:Ҫ��ȡ�Ŀ��ַ����Χ��0~��block_totalnum-1��=4095
				PageNum:Ҫ��ȡ��ҳ��ַ,��Χ:0~(block_pagenum)=63
				ColNum:Ҫ��ȡ���п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)=2112-1
				NumByteToRead:��ȡ�ֽ���(���ܿ�ҳ��)
*���         ����ȡ����ֵvalue

***********************************************************************/
u32 NAND_ReadPage(u32 BlockNum,u32 PageNum,u16 ColNum,u16 NumByteToRead)
{
    vu16 i=0;
	u8 res=0;
	u32 value=0;                                      //�����ȡ������
	BlockNum<<=6;
     *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_A;      //READ PAGE����
    //���͵�ַ������η��ͣ�
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum|BlockNum);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_AREA_TRUE1;    //READ PAGE����
	
	res=NAND_WaitRB(0);			               //�ȴ�RB=0 
    if(res)return NSTA_TIMEOUT;	               //��ʱ�˳�
	
    //����2�д����������ж�NAND�Ƿ�׼���õ�
	res=NAND_WaitRB(1);			               //�ȴ�RB=1 
    if(res)return NSTA_TIMEOUT;                //��ʱ�˳�
	if(NumByteToRead%NAND_ECC_SECTOR_SIZE)     //����NAND_ECC_SECTOR_SIZE����������������ECCУ��
	{ 
		//��ȡNAND FLASH�е�ֵ
		for(i=0;i<NumByteToRead;i++)           //NumByteToRead<4,����ȡ32λ����
		{
			value|= ((*(vu8*)NAND_ADDRESS)<<(i*8));
		}		
	}
		if(NAND_WaitForReady()==NSTA_READY) 	
		{ return value;	}                      //�ɹ�������ȡ����ֵ���� 	
}

/*********************************************************************
*������       ��NAND_WritePage()
*��������     ����NANDһҳ��д��ָ�����ֽڵ�����
*����         ��BlockNum:Ҫд��Ŀ��ַ����Χ��0~��block_totalnum-1��=4095
				PageNum:Ҫд���ҳ��ַ,��Χ:0~(block_pagenum)=63
				ColNum:Ҫд����п�ʼ��ַ(Ҳ����ҳ�ڵ�ַ),��Χ:0~(page_totalsize-1)=2112-1
				dat:Ҫд�������
				NumByteToWrite:Ҫд����ֽ�������ֵ���ܳ�����ҳʣ���ֽ���������
*���         ��0,�ɹ� 
				����,�������

***********************************************************************/
u32 NAND_WritePage(u32 BlockNum,u32 PageNum,u16 ColNum,u32 dat,u16 NumByteToRead)
{
    vu16 i=0;  
	BlockNum<<=6;
	*(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE0;    //WRITE PAGE��������η��� 
    //���͵�ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)ColNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(ColNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(PageNum|BlockNum);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
	NAND_Delay(30);                              //�ȴ�tADL   
	if(NumByteToRead%NAND_ECC_SECTOR_SIZE)       //����NAND_ECC_SECTOR_SIZE����������������ECCУ��
	{ 
		
		for(i=0;i<NumByteToRead;i++)             //��ȡNAND FLASH�е�ֵ
		{
			*(vu8*)NAND_ADDRESS= (vu8) (dat>>(i*8));
		}
	}
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_WRITE_TURE1;        //WRITE PAGE����
    if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;   //ʧ��
    return 0;                                               //�ɹ�   
}

/*********************************************************************
*������       ��NAND_EraseBlock()
*��������     ������һ����
*����         ��BlockNum:Ҫ������BLOCK���,��Χ:0-(block_totalnum-1)0~4095
*���         ��0,�����ɹ�
				����,����ʧ��

***********************************************************************/
u8 NAND_EraseBlock(u32 BlockNum)
{
	if(nand_dev.id==MT29F16G08ABABA)BlockNum<<=7;      	    //�����ַת��Ϊҳ��ַ
    else if(nand_dev.id==MT29F4G08ABADA)BlockNum<<=6;
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE0;              //ERASE BLOCK ��������η��� 
    //���Ϳ��ַ
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)BlockNum;
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>8);
    *(vu8*)(NAND_ADDRESS|NAND_ADDR)=(u8)(BlockNum>>16);
	
    *(vu8*)(NAND_ADDRESS|NAND_CMD)=NAND_ERASE1;              //ERASE BLOCK ����
	
	if(NAND_WaitForReady()!=NSTA_READY)return NSTA_ERROR;    //ʧ��
    return 0;	                                             //�ɹ�   
} 

/*********************************************************************
*������       ��NAND_EraseChip()
*��������     ��ȫƬ����NAND FLASH
*����         ����
*���         ����

***********************************************************************/
void NAND_EraseChip(void)
{
    u8 status;
    u16 i=0;
    for(i=0;i<nand_dev.block_totalnum;i++)                                   //ѭ���������еĿ�
    {
        status=NAND_EraseBlock(i);
        if(status)printf("Erase %d block fail!!��������Ϊ%d\r\n",i,status);   //����ʧ��
    }
}

/*********************************************************************
*������       ��NAND_ECC_Get_OE()
*��������     ����ȡECC������λ/ż��λ
*����         ��oe:0,ż��λ��1,����λ
				eccval:�����eccֵ
*���         ��������eccֵ(���16λ)

***********************************************************************/
u16 NAND_ECC_Get_OE(u8 oe,u32 eccval)
{
	u8 i;
	u16 ecctemp=0;
	for(i=0;i<24;i++)
	{
		if((i%2)==oe)
		{
			if((eccval>>i)&0X01)ecctemp+=1<<(i>>1); 
		}
	}
	return ecctemp;
} 

/*********************************************************************
*������       ��NAND_ECC_Correction()
*��������     ��ECCУ������
*����         ��eccrd:��ȡ����,ԭ�������ECCֵ
				ecccl:��ȡ����ʱ,Ӳ�������ECCֵ
*���         ��0,����������
				����,ECC����(�д���2��bit�Ĵ���,�޷��ָ�)

***********************************************************************/
u8 NAND_ECC_Correction(u8* data_buf,u32 eccrd,u32 ecccl)
{
	u16 eccrdo,eccrde,eccclo,ecccle;
	u16 eccchk=0;
	u16 errorpos=0; 
	u32 bytepos=0;  
	eccrdo=NAND_ECC_Get_OE(1,eccrd);	//��ȡeccrd������λ
	eccrde=NAND_ECC_Get_OE(0,eccrd);	//��ȡeccrd��ż��λ
	eccclo=NAND_ECC_Get_OE(1,ecccl);	//��ȡecccl������λ
	ecccle=NAND_ECC_Get_OE(0,ecccl); 	//��ȡecccl��ż��λ
	eccchk=eccrdo^eccrde^eccclo^ecccle;
	if(eccchk==0XFFF)	               //ȫ1,˵��ֻ��1bit ECC����
	{
		errorpos=eccrdo^eccclo; 
		printf("errorpos:%d\r\n",errorpos); 
		bytepos=errorpos/8; 
		data_buf[bytepos]^=1<<(errorpos%8);
	}else				               //����ȫ1,˵��������2bit ECC����,�޷��޸�
	{
		printf("2bit ecc error or more\r\n");
		return 1;
	} 
	return 0;
}

