2017.12.19	
�����4�͵��5��pwm�ڵ�����
�ʣ�1���������е�4�ź�5���������Ǹ�ͷ���϶Ի�һ��
    2�����б����һ�£�������ͷ���б��cadͼ�ж�Ӧ�ı�

�Զ���������˳��
1��Uart_Washlet_Auto()				�������õĺ���
2��Push_Rod_Start��Motor_1_START	����֧�����ȶ�����
3��Uart_Back_Leg()					����֧����������ͬʱ���к���
4��Uart_Washlet(0);	            	���������			
5��Uart_Swash_Dry();  				������ϴ���			
  5-1��Uart_Push_Rod_Swash_Dry(1,2000);      ����������ϴ����Ƹ����    
  5-2��Uart_Swash_Auto();  				   ���������Զ���ϴ���
    5-2-1��(DIR_SB=1)Uart_Push_Rod_Swash(flag,swash_dry_pulse_lim);������������ִ���Զ���ϴ����
  5-3��Uart_Swash_Hand();			    ��������ִ�г�ϴ����
	5-3-1��Uart_Push_Rod_Swash()				�����Ƹ��˶�	
  5-4:Uart_Dry_Auto();  				�����Զ����2����
	5-4-1��Uart_Push_Rod_Dry()					�����Ƹ��˶�
  5-5��Uart_Dry_Hand();				��������ִ�к�ɹ���
	5-5-1�� Uart_Push_Rod_Dry					�����Ƹ��˶�

6��Uart_Washlet()					��������ر�	