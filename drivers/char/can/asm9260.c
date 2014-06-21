#include "asm9260.h"
#include "config.h"

extern stcASM9260_BufInfo ASM9260_RxBuf;
/*
********************************************************************************************************************
**����ԭ��	:  	BOOL SJA_SoftRst(BOOL bIsRst)
**����˵��	:	bIsRst	->	TURE	ASM9260���������λģʽ	
**		:	FALSE	ASM9260�˳������λģʽ
**����ֵ	:	TURE	->	�ɹ������Ӧ����
**		:	FALSE	->	����ʧ��
**˵	��	:	
********************************************************************************************************************
*/
BOOL asm_SoftRst(BOOL IsRst)
{
	uRsgMod ModTemp;
	INT8U Statu;
	
	ModTemp.Byte = Read_ASM9260(MOD)&0x1F;
	ModTemp.Bits.RM = (TRUE == IsRst)? 1:0;
	Write_ASM9260(ModTemp.Byte, MOD);
	
	ModTemp.Byte = Read_ASM9260(MOD)&0x1F;
	Statu = (ModTemp.Bits.RM == 1)? TRUE:FALSE;
	
	return (Statu == IsRst)? TRUE:FALSE;
}
/*
********************************************************************************************************************
**����ԭ��	:  	BOOL SetModeReg(INT8U Val, BOOL bIsEn)
**����˵��	:	Val	->	����ģʽֵ	MOD_LOM			ֻ��ģʽ
**		:	MOD_STM		�Բ���ģʽ
**		:	MOD_AFM_SIG	���˲���ʽ
**		:	MOD_AFM_DOB	˫�˲���ʽ
**		:	bIsEn	->	�趨��ʽ	TRUE			ʹ���趨
**		;	FALSE		�����趨
**����ֵ	:	TURE	->	�ɹ������Ӧ����
**		:	FALSE	->	����ʧ��	
**˵	��	:	�ú����趨ASM9260ģʽ,�����������λģʽ�е���,����ʧ��,ģʽֵ�ɲ���ʹ��
********************************************************************************************************************
*/
BOOL SetModeReg(INT8U Val, BOOL bIsEn)
{
	uRsgMod ModTemp;
	ModTemp.Byte = Read_ASM9260(MOD)&0x1F;
	if(ModTemp.Bits.RM)
	{
		if(Val&MOD_LOM)
		{
			ModTemp.Bits.LOM = (TRUE == bIsEn)? 1:0;
		}
		if(Val&MOD_STM)
		{
			ModTemp.Bits.STM = (TRUE == bIsEn)? 1:0;
		}
		if(Val&MOD_AFM_SIG)
		{
			ModTemp.Bits.AFM = 1;
		}
		if(Val&MOD_AFM_DOB)
		{
			ModTemp.Bits.AFM = 0;
		}
	}
	else
		return FALSE;
	Write_ASM9260(ModTemp.Byte,MOD);
	if(ModTemp.Byte == (Read_ASM9260(MOD)&0x1F))
	{
		return TRUE;
	}
	else
		return FALSE;
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL EnterSleepMod(BOOL IsEn)
**����˵��		:	IsEn	->	ASM9260������븴λģʽ
**����ֵ		:	TURE	->	�ɹ������Ӧ����
**			:	FALSE	->	����ʧ��	
**˵	��		:	�ú��������ڹ���ģʽ�µ���,����ʧ��
********************************************************************************************************************
*/
BOOL EnterSleepMod(BOOL IsEn)
{
	uRsgMod ModTemp;
	INT8U Statu;
	ModTemp.Byte = Read_ASM9260(MOD)&0x1F;
	if(ModTemp.Bits.RM) return FALSE;
	ModTemp.Bits.SM = (TRUE == IsEn)? 1:0;
	Write_ASM9260(ModTemp.Byte,MOD);
	ModTemp.Byte = Read_ASM9260(MOD)&0x1F;
	Statu = (ModTemp.Bits.SM == 1)? TRUE:FALSE;
	return (Statu == IsEn)? TRUE:FALSE;
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL SetCommand(INT8U Cmd)
**����˵��		:	Cmd		->	������	CMR_NOM_SD	��������
**			:	CMR_SIG_SD	���η���
**			:	CMR_SLF_SD	�Է�����
**			:	CMR_CL_BUF	�ͷŽ��ջ�������
**			:	CMR_CL_DATA	����������״̬
**����ֵ		:	��
**˵	��		:	ÿ�ε��øú���ֻ�ܷ���һ������ֵ
********************************************************************************************************************
*/
void SetCommand(INT8U Cmd)
{
	uRsgCmd CMRTemp;
	CMRTemp.Byte = 0;
	switch(Cmd) 
	{
		case CMR_SIG_SD:
			CMRTemp.Bits.AT = 1;
		case CMR_NOM_SD:
			if(1 == (Read_ASM9260(MOD)&0x04))
			{
				while(TRUE != asm_SoftRst(TRUE));
				while(TRUE != SetModeReg(MOD_STM,FALSE));
				while(TRUE != asm_SoftRst(FALSE));
			}
			CMRTemp.Bits.TR = 1;
			break;
		case CMR_SLF_SD:
			if(0 == (Read_ASM9260(MOD)&0x04))
			{
				while(TRUE != asm_SoftRst(TRUE));
				while(TRUE != SetModeReg(MOD_STM,TRUE));
				while(TRUE != asm_SoftRst(FALSE));
			}
			CMRTemp.Bits.SRR = 1;
			CMRTemp.Bits.AT = 1;
			break;
		case CMR_CL_BUF:
			CMRTemp.Bits.RR = 1;
			break;
		case CMR_CL_DATA:
			CMRTemp.Bits.CDO = 1;
			break;
		default:
			break;
	}
	Write_ASM9260(CMRTemp.Byte,CMR);
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL SetInterrupt(INT8U Val, BOOL IsEn)
**����˵��		:	Val		->	�ж�ֵ	IER_RC--IER_BE
**				:	IsEn	->	�Ƿ�ʹ�ܸ��ж�
**����ֵ		:	
**˵	��		:	�ж�ֵ�ɲ���ʹ��,ȫ��ʹ��,�����ж�ʱ��ʹ�� IER_ALL
********************************************************************************************************************
*/
BOOL SetInterrupt(INT8U Val, BOOL IsEn)
{
	INT8U IntVal;
	INT8U temp;
	if(TRUE == IsEn)
	{
		Write_ASM9260(Val,IER);
		temp = Read_ASM9260(IER);
		if(temp == Val)
		{
			return TRUE;
		}
		else
		{
			return FALSE;
		}
	}
	else
	{
		IntVal = Read_ASM9260(IER);
		IntVal &= ~Val;
		Write_ASM9260(IntVal,IER);
		return (IntVal == Read_ASM9260(IER))? TRUE:FALSE;
	}
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL SetBaudRate(INT16U BaudVal)
**����˵��		:	BaudVal		->	������ֵ,�� ��ʱ��0<<8|��ʱ��1 ���
**����ֵ		:	
**˵	��		:	�ú������������λģʽ�µ���
********************************************************************************************************************
*/
BOOL SetBaudRate(INT32U BaudVal)
{
        INT32U h;
	INT16U  l,J;
	if(1 == (Read_ASM9260(MOD)&0x01))
	{
		Write_ASM9260((INT8U)((BaudVal&0x00FF0000)>>16),BTR0);
		Write_ASM9260((INT8U)((BaudVal&0x0000FF00)>>8),BTR1);
		Write_ASM9260((INT8U)(BaudVal&0x000000FF),BTR2);
		
		h = Read_ASM9260(BTR0) << 16;
		l = Read_ASM9260(BTR1)<<8;
		J=  Read_ASM9260(BTR2);
		if(BaudVal == (INT32U) (h + l+J) )
		{
                    return TRUE;
                   
		}
		else
		{
                        return FALSE;
		}
	}
	else
		return FALSE;

}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL SetOutPutMod(INT8U Val)
**����˵��		:	Val		->	���ģʽֵ
**����ֵ		:	
**˵	��		:	ͨ����Ϊ0x1A
********************************************************************************************************************
*/
BOOL SetOutPutMod(INT8U Val)
{
	Write_ASM9260(Val,OCR);
	return (Val == Read_ASM9260(OCR))? TRUE:FALSE;
}


BOOL SetClkDiv(INT8U CAN_Mod,BOOL bRxINTEn,BOOL bClkOFF,INT8U Div)
{
	INT8U CDRTemp = 0;
	INT8U RxINTTemp = (bRxINTEn == TRUE)? 1:0;
	INT8U ClkOffTemp = (bClkOFF == TRUE)? 1:0;
	if(1 == (Read_ASM9260(MOD)&0x01))
	{
		CDRTemp = CAN_Mod<<7|0x01<<6|RxINTTemp<<5|ClkOffTemp<<3|Div;
		Write_ASM9260(CDRTemp,CDR);
	}
	else
		return FALSE;
	return ((CDRTemp == Read_ASM9260(CDR))? TRUE:FALSE);
}
/*
********************************************************************************************************************
**����ԭ��		:  	void SetTxBuf(stcASM9260_TxInfo *pTxInfo)
**����˵��		:	pTxInfo		->	�������ݵĽṹ��ָ��
**����ֵ		:	
**˵	��		:	���øú���д�������͵�����
********************************************************************************************************************
*/
void SetTxBuf(stcASM9260_BufInfo *pTxInfo)
{
	INT8U	i;
	INT32U	IDTemp;
	Write_ASM9260(pTxInfo->FrIf.Byte,TXBUF);
	if(1 == pTxInfo->FrIf.Bits.FF)   /*��չ(EFF)*/
	{
		IDTemp = pTxInfo->FrID<<3;
		for(i=0; i<4; i++)
		{
			Write_ASM9260((INT8U)(IDTemp>>8*(3-i)),TXBUF+1+i);
		}
		for(i=0; i<pTxInfo->FrIf.Bits.DLC; i++)
		{
			Write_ASM9260(pTxInfo->DA[i],TXBUF+5+i);
		}
	}
	else                            /*��׼֡��ʽ(SFF)*/
	{
		IDTemp = pTxInfo->FrID<<21;
		for(i=0; i<2; i++)
		{
			Write_ASM9260((INT8U)(IDTemp>>8*(3-i)),TXBUF+1+i);
		}
		for(i=0; i<pTxInfo->FrIf.Bits.DLC; i++)
		{
			Write_ASM9260(pTxInfo->DA[i],TXBUF+3+i);
		}
	}
		
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL ReadRcvBuf(stcASM9260_BufInfo *pRcvBuf)
**����˵��		:	pRcvBuf		->	����������ݵĽṹ��ָ��
**����ֵ		:	
**˵	��		:	���øú�������CAN����
********************************************************************************************************************
*/
BOOL ReadRcvBuf(stcASM9260_BufInfo *pRcvBuf)
{
	INT8U i;
	INT32U IDTemp = 0;
	if(pRcvBuf == NULL) 
		return FALSE;
	pRcvBuf->FrIf.Byte =  Read_ASM9260(RXBUF);
	pRcvBuf->FrID = 0;
	if(pRcvBuf->FrIf.Bits.FF == 1)
	{
		for(i=0; i<4; i++)
		{
			IDTemp = Read_ASM9260(RXBUF+1+i);
			IDTemp = (IDTemp<<(24-(8*i)));
			pRcvBuf->FrID |= IDTemp;
			IDTemp = 0;
		}
		pRcvBuf->FrID >>= 3;
		for(i=0; i<pRcvBuf->FrIf.Bits.DLC; i++)
		{
			pRcvBuf->DA[i] = Read_ASM9260(RXBUF+5+i);
		}
	}
	else
	{
		for(i=0; i<2; i++)
		{
			IDTemp = Read_ASM9260(RXBUF+1+i);
			IDTemp = (IDTemp<<(24-(8*i)));
			pRcvBuf->FrID |= IDTemp;
			IDTemp = 0; 
		}
		pRcvBuf->FrID >>= 21;
		for(i=0; i<pRcvBuf->FrIf.Bits.DLC; i++)
		{
			pRcvBuf->DA[i] = Read_ASM9260(RXBUF+3+i);
		}
	}
	return TRUE;
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL ACRCode(INT32U ACRCode)
**����˵��		:	ACRCode		->	���մ���ֵ	ACR0_ACR1_ACR2_ACR3
**����ֵ		:	
**˵	��		:	���øú����������մ���
********************************************************************************************************************
*/
BOOL ACRCode(INT32U ACRCode)
{
	int i;
	for(i=0; i<4; i++)
	{
		Write_ASM9260((INT8U)(ACRCode>>(24-8*i)),ACR0+i);
	}
	for (i=0; i<4; i++)
	{
		if((INT8U)(ACRCode>>(24-8*i)) != Read_ASM9260(ACR0+i))
		{
			return FALSE;
		}
	}
	return TRUE;
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL AMRCode(INT32U AMRCode)
**����˵��		:	AMRCode		->	����������	AMR0_AMR1_AMR2_AMR3
**����ֵ		:	
**˵	��		:	���øú�����������������
********************************************************************************************************************
*/
BOOL AMRCode(INT32U AMRCode)
{
	int i;
	for(i=0; i<4; i++)
	{
		Write_ASM9260((INT8U)(AMRCode>>(24-8*i)),AMR0+i);
	}
	for (i=0; i<4; i++)
	{
		if((INT8U)(AMRCode>>(24-8*i)) != Read_ASM9260(AMR0+i))
		{
			return FALSE;
		}
	}
	return TRUE;
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL SetFliter(INT32U ACRCode, INT32U AMRCode)
**����˵��		:	ACRCode		->	���մ���ֵ	ACR0_ACR1_ACR2_ACR3
**			:	AMRCode		->	����������	AMR0_AMR1_AMR2_AMR3
**����ֵ		:	
**˵	��		:	���øú���д�������͵�����
********************************************************************************************************************
*/
BOOL SetFliter(INT32U ACRCode, INT32U AMRCode)
{
	INT8U i;
	for(i=0; i<4; i++)
	{
		Write_ASM9260((INT8U)(ACRCode>>(24-8*i)),ACR0+i);
	}
	for(i=0; i<4; i++)
	{
		Write_ASM9260((INT8U)(AMRCode>>(24-8*i)),AMR0+i);
	}
	for (i=0; i<4; i++)
	{
		if((INT8U)(ACRCode>>(24-8*i)) != Read_ASM9260(ACR0+i))
		{
			return FALSE;
		}
	}
	for (i=0; i<4; i++)
	{
		if((INT8U)(AMRCode>>(24-8*i)) != Read_ASM9260(AMR0+i))
		{
			return FALSE;
		}
	}
	return TRUE;
}
/*
********************************************************************************************************************
**��	��		:  	Write_ASM9260(Val, OffSet)
**����˵��		:	Val		->	��д�������
**				:	OffSet	->	ASM9260Ƭ�ڼĴ���ƫ�Ƶ�ַ	
**˵	��		:	�ú�����дASM9260�ڲ��Ĵ���
********************************************************************************************************************
*/


void Write_ASM9260(INT8U Val, INT8U OffSet)
{
	 as3310_writeb(Val, OffSet+0x80050000);
}

/*
********************************************************************************************************************
**��	��		:  	Read_ASM9260(OffSet)
**����˵��		:	OffSet	->	ASM9260Ƭ�ڼĴ���ƫ�Ƶ�ַ	
**˵	��		:	�ú�����дASM9260�ڲ��Ĵ���
********************************************************************************************************************
*/

INT8U Read_ASM9260(INT8U OffSet)
{
	 as3310_readb(OffSet+0x80050000);
         return as3310_readb(OffSet+0x80050000);
}

/*
********************************************************************************************************************
**����ԭ��		:  	void IntEntry(void)
**����˵��		:	NULL
**����ֵ		:	NULL
**˵	��		:	�жϴ������
********************************************************************************************************************
*/
void IntEntry(INT8U IRTemp)
{
	if(IR_RC&IRTemp)
	{
		ReadRcvBuf(&ASM9260_RxBuf);
                 SetCommand(CMR_CL_BUF);
               
	}
	if(IR_TR&IRTemp)
	{
	}
	if(IR_ER&IRTemp)
	{
	}
	if(IR_DO&IRTemp)
	{
		SetCommand(CMR_CL_DATA);
	}
	if(IR_WU&IRTemp)
	{
	}
	if(IR_EP&IRTemp)
	{
	}
	if(IR_AL&IRTemp)
	{
	}
	if(IR_BE&IRTemp)
	{
	}
}
/*
********************************************************************************************************************
**����ԭ��		:  	BOOL ASM9260Init(INT8U CanMode, INT32U BaudRate,INT32U ACRCode, INT32U AMRCode)
**����˵��		:	CanMode		->	CANģʽֵ	BAIS_CAN
**				:								PELI_CAN
**				:	BaudRate	->	������ֵ
**				:	ACRCode		->	���մ���
**				:	AMRCode		->	����������
**����ֵ		:	
**˵	��		:	�ú��������������λģʽ�µ���
********************************************************************************************************************
*/
BOOL ASM9260Init(INT8U CanMode, INT32U BaudRate,INT32U ACRCode, INT32U AMRCode)
{
	if(TRUE != asm_SoftRst(TRUE))			//���븴λģʽ
	{
		printk("SJA_SoftRst Err!.\n");
		return FALSE; 
	}
	if(TRUE != SetModeReg(MOD_AFM_SIG,TRUE))	//�������չ��˷�ʽ
	{
		printk("SetModeReg Err!.\n");
		return FALSE;
	}
	if(TRUE != SetInterrupt(IER_RC,TRUE))		//�����жϣ�ֻ���Ž����ж�
	{
		printk("SetInterrupt Err!.\n");
		return FALSE;
	}
	if(TRUE != SetClkDiv(CanMode,FALSE,TRUE,0))	//���ù���ģʽ
	{
		printk("SetClkDiv Err!.\n");
		return FALSE;
	}
	if(TRUE != SetFliter(ACRCode,AMRCode))		//�������չ���ֵ
	{
		printk("SetFliter Err!.\n");
		return FALSE;
	}
	if(TRUE != SetBaudRate(BaudRate))		//���ò�����
	{
		printk("SetBaudRate Err!.\n");
		return FALSE;
	}
	if(TRUE != SetOutPutMod(0x1A))			//�����������ģʽ
	{
		printk("SetOutPutMod Err!.\n");
		return FALSE;
	}
	if(TRUE != asm_SoftRst(FALSE))			//�˳������λģʽ
	{
		printk("SJA_SoftRst Err!.\n");
		return FALSE;
	}
	return TRUE;
}
/*
*********************************************************************************************************
**                            End Of File
*********************************************************************************************************
*/
