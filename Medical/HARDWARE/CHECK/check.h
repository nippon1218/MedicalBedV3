#ifndef __CHECK_H
#define __CHECK_H
#include "sys.h"

u8 Breakdown_Judge(void);            //������Ϻ���
u8 Breakdown_Treatment(void);        //������ϣ����������أ�������
u8 Uart_Breakdown_Treatment(void);   //������ϣ����������أ�������


u8 GDCheck(u8 num);
void GDCheckAll(void);

u8 GDCheckDealy(u8 GDID,u16 timedelay);
u8 GDCheckAlm(u8 GDID,u8 num);
u8 GDCheckPrint(u8 GDID,u16 usdelay,u8* str);

u8 GDSCheckAllAlm(u8 num);
u8 UsartCheck(u8* str);
u8 UsartCheck2(u8* str1,u8* str2);




#endif


