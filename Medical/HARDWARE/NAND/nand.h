#ifndef __NAND_H
#define __NAND_H
#include "sys.h"

#define NAND_MAX_PAGE_SIZE			4096		//����NAND FLASH������PAGE��С��������SPARE������Ĭ��4096�ֽ�
#define NAND_ECC_SECTOR_SIZE		512			//ִ��ECC����ĵ�Ԫ��С��Ĭ��512�ֽ�

/************************NAND���Խṹ��***********************************/

typedef struct
{
    u16 page_totalsize;     	//ÿҳ�ܴ�С��main����spare���ܺ�
    u16 page_mainsize;      	//ÿҳ��main����С
    u16 page_sparesize;     	//ÿҳ��spare����С
    u8  block_pagenum;      	//ÿ���������ҳ����
    u16 plane_blocknum;     	//ÿ��plane�����Ŀ�����
    u16 block_totalnum;     	//�ܵĿ�����
    u16 good_blocknum;      	//�ÿ�����    
    u16 valid_blocknum;     	//��Ч������(���ļ�ϵͳʹ�õĺÿ�����)
    u32 id;             		//NAND FLASH ID
    u16 *lut;      			   	//LUT�������߼���-�����ת��
	u32 ecc_hard;				//Ӳ�����������ECCֵ
	u32 ecc_hdbuf[NAND_MAX_PAGE_SIZE/NAND_ECC_SECTOR_SIZE];//ECCӲ������ֵ������  	
	u32 ecc_rdbuf[NAND_MAX_PAGE_SIZE/NAND_ECC_SECTOR_SIZE];//ECC��ȡ��ֵ������
}nand_attriute;      

extern nand_attriute nand_dev;				//nand��Ҫ�����ṹ�� 

#define NAND_RB  				PDin(6)	    //NAND Flash����/æ���� 

#define NAND_ADDRESS			0X80000000	//nand flash�ķ��ʵ�ַ,��NCE3,��ַΪ:0X8000 0000
#define NAND_CMD				1<<16		//��������
#define NAND_ADDR				1<<17		//���͵�ַ

/************************NAND FLASH����***********************************/

#define NAND_READID         	0X90    	//��IDָ��
#define NAND_FEATURE			0XEF    	//��������ָ��(���� NAND ����ز���������ʱ��ģʽ)
#define NAND_RESET          	0XFF    	//��λNAND
#define NAND_READSTA        	0X70   	 	//��NAND״̬(��������жϱ��/���������Ƿ����)
#define NAND_AREA_A         	0X00        //READ PAGE��������η��� 
#define NAND_AREA_TRUE1     	0X30 
#define NAND_WRITE0        	 	0X80        //WRITE PAGE��������η���       
#define NAND_WRITE_TURE1    	0X10
#define NAND_ERASE0        	 	0X60        //ERASE BLOCK ��������η���      
#define NAND_ERASE1         	0XD0
#define NAND_MOVEDATA_CMD0  	0X00        //NAND ���ڲ������ƶ�������Ĵη���
#define NAND_MOVEDATA_CMD1  	0X35
#define NAND_MOVEDATA_CMD2  	0X85
#define NAND_MOVEDATA_CMD3  	0X10

/************************NAND FLASH״̬***********************************/

#define NSTA_READY       	   	0X40		//nand�Ѿ�׼����
#define NSTA_ERROR				0X01		//nand����
#define NSTA_TIMEOUT        	0X02		//��ʱ
#define NSTA_ECC1BITERR       	0X03		//ECC 1bit����
#define NSTA_ECC2BITERR       	0X04		//ECC 2bit���ϴ���

/*******************NAND FLASH�ͺźͶ�Ӧ��ID��*******************************/

#define MT29F4G08ABADA			0XDC909556	//MT29F4G08ABADA
#define MT29F16G08ABABA			0X48002689	//MT29F16G08ABABA

u8 NAND_Init(void);                        //��ʼ��NAND FLASH
u8 NAND_ModeSet(u8 mode);                  //��ȡNAND FLASH��ID
u32 NAND_ReadID(void);                     //��ȡNAND FLASH��ID
u8 NAND_ReadStatus(void);                  //��NAND״̬
u8 NAND_WaitForReady(void);                //�ȴ�NAND׼����
u8 NAND_Reset(void);                       //��λNAND
u8 NAND_WaitRB(vu8 rb);                    //�ȴ�RB�ź�Ϊĳ����ƽ
void NAND_Delay(vu32 i);                   //NAND��ʱ

u32 NAND_ReadPage(u32 BlockNum,u32 PageNum,u16 ColNum,u16 NumByteToRead);                      //��ȡNAND Flash��ָ��ҳָ���е�����
u32 NAND_WritePage(u32 BlockNum,u32 PageNum,u16 ColNum,u32 dat,u16 NumByteToRead);             //��NANDһҳ��д��ָ�����ֽڵ�����
u8  NAND_EraseBlock(u32 BlockNum);                                                             //����һ����
void NAND_EraseChip(void);                                                                     //ȫƬ����NAND FLASH

u16 NAND_ECC_Get_OE(u8 oe,u32 eccval);                                                         //��ȡECC������λ/ż��λ
u8 NAND_ECC_Correction(u8* data_buf,u32 eccrd,u32 ecccl);                                      //ECCУ������

#endif





