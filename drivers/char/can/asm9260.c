#include "asm9260.h"
#include "config.h"

extern stcASM9260_BufInfo ASM9260_RxBuf;
/*
********************************************************************************************************************
**函数原型	:  	BOOL SJA_SoftRst(BOOL bIsRst)
**参数说明	:	bIsRst	->	TURE	ASM9260进入软件复位模式	
**		:	FALSE	ASM9260退出软件复位模式
**返回值	:	TURE	->	成功完成相应设置
**		:	FALSE	->	设置失败
**说	明	:	
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
**函数原型	:  	BOOL SetModeReg(INT8U Val, BOOL bIsEn)
**参数说明	:	Val	->	设置模式值	MOD_LOM			只听模式
**		:	MOD_STM		自测试模式
**		:	MOD_AFM_SIG	单滤波方式
**		:	MOD_AFM_DOB	双滤波方式
**		:	bIsEn	->	设定方式	TRUE			使能设定
**		;	FALSE		禁能设定
**返回值	:	TURE	->	成功完成相应设置
**		:	FALSE	->	设置失败	
**说	明	:	该函数设定ASM9260模式,必须在软件复位模式中调用,否则失败,模式值可并联使用
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
**函数原型		:  	BOOL EnterSleepMod(BOOL IsEn)
**参数说明		:	IsEn	->	ASM9260请求进入复位模式
**返回值		:	TURE	->	成功完成相应设置
**			:	FALSE	->	设置失败	
**说	明		:	该函数必须在工作模式下调用,否则失败
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
**函数原型		:  	BOOL SetCommand(INT8U Cmd)
**参数说明		:	Cmd		->	命令字	CMR_NOM_SD	正常发送
**			:	CMR_SIG_SD	单次发送
**			:	CMR_SLF_SD	自发自收
**			:	CMR_CL_BUF	释放接收缓缓冲区
**			:	CMR_CL_DATA	清除数据溢出状态
**返回值		:	无
**说	明		:	每次调用该函数只能发送一条命令值
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
**函数原型		:  	BOOL SetInterrupt(INT8U Val, BOOL IsEn)
**参数说明		:	Val		->	中断值	IER_RC--IER_BE
**				:	IsEn	->	是否使能该中断
**返回值		:	
**说	明		:	中断值可并联使用,全部使能,禁能中断时可使用 IER_ALL
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
**函数原型		:  	BOOL SetBaudRate(INT16U BaudVal)
**参数说明		:	BaudVal		->	波特率值,由 定时器0<<8|定时器1 组成
**返回值		:	
**说	明		:	该函必须在软件复位模式下调用
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
**函数原型		:  	BOOL SetOutPutMod(INT8U Val)
**参数说明		:	Val		->	输出模式值
**返回值		:	
**说	明		:	通常设为0x1A
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
**函数原型		:  	void SetTxBuf(stcASM9260_TxInfo *pTxInfo)
**参数说明		:	pTxInfo		->	发送数据的结构体指针
**返回值		:	
**说	明		:	调用该函数写入欲发送的数据
********************************************************************************************************************
*/
void SetTxBuf(stcASM9260_BufInfo *pTxInfo)
{
	INT8U	i;
	INT32U	IDTemp;
	Write_ASM9260(pTxInfo->FrIf.Byte,TXBUF);
	if(1 == pTxInfo->FrIf.Bits.FF)   /*扩展(EFF)*/
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
	else                            /*标准帧格式(SFF)*/
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
**函数原型		:  	BOOL ReadRcvBuf(stcASM9260_BufInfo *pRcvBuf)
**参数说明		:	pRcvBuf		->	接收输出数据的结构体指针
**返回值		:	
**说	明		:	调用该函数接收CAN数据
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
**函数原型		:  	BOOL ACRCode(INT32U ACRCode)
**参数说明		:	ACRCode		->	验收代码值	ACR0_ACR1_ACR2_ACR3
**返回值		:	
**说	明		:	调用该函数设置验收代码
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
**函数原型		:  	BOOL AMRCode(INT32U AMRCode)
**参数说明		:	AMRCode		->	验收屏蔽码	AMR0_AMR1_AMR2_AMR3
**返回值		:	
**说	明		:	调用该函数设置验收屏蔽码
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
**函数原型		:  	BOOL SetFliter(INT32U ACRCode, INT32U AMRCode)
**参数说明		:	ACRCode		->	验收代码值	ACR0_ACR1_ACR2_ACR3
**			:	AMRCode		->	验收屏蔽码	AMR0_AMR1_AMR2_AMR3
**返回值		:	
**说	明		:	调用该函数写入欲发送的数据
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
**宏	名		:  	Write_ASM9260(Val, OffSet)
**参数说明		:	Val		->	欲写入的数据
**				:	OffSet	->	ASM9260片内寄存器偏移地址	
**说	明		:	该宏用于写ASM9260内部寄存器
********************************************************************************************************************
*/


void Write_ASM9260(INT8U Val, INT8U OffSet)
{
	 as3310_writeb(Val, OffSet+0x80050000);
}

/*
********************************************************************************************************************
**宏	名		:  	Read_ASM9260(OffSet)
**参数说明		:	OffSet	->	ASM9260片内寄存器偏移地址	
**说	明		:	该宏用于写ASM9260内部寄存器
********************************************************************************************************************
*/

INT8U Read_ASM9260(INT8U OffSet)
{
	 as3310_readb(OffSet+0x80050000);
         return as3310_readb(OffSet+0x80050000);
}

/*
********************************************************************************************************************
**函数原型		:  	void IntEntry(void)
**参数说明		:	NULL
**返回值		:	NULL
**说	明		:	中断处理入口
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
**函数原型		:  	BOOL ASM9260Init(INT8U CanMode, INT32U BaudRate,INT32U ACRCode, INT32U AMRCode)
**参数说明		:	CanMode		->	CAN模式值	BAIS_CAN
**				:								PELI_CAN
**				:	BaudRate	->	波特率值
**				:	ACRCode		->	验收代码
**				:	AMRCode		->	验收屏蔽码
**返回值		:	
**说	明		:	该函数必须在软件复位模式下调用
********************************************************************************************************************
*/
BOOL ASM9260Init(INT8U CanMode, INT32U BaudRate,INT32U ACRCode, INT32U AMRCode)
{
	if(TRUE != asm_SoftRst(TRUE))			//进入复位模式
	{
		printk("SJA_SoftRst Err!.\n");
		return FALSE; 
	}
	if(TRUE != SetModeReg(MOD_AFM_SIG,TRUE))	//设置验收过滤方式
	{
		printk("SetModeReg Err!.\n");
		return FALSE;
	}
	if(TRUE != SetInterrupt(IER_RC,TRUE))		//设置中断，只开放接收中断
	{
		printk("SetInterrupt Err!.\n");
		return FALSE;
	}
	if(TRUE != SetClkDiv(CanMode,FALSE,TRUE,0))	//设置工作模式
	{
		printk("SetClkDiv Err!.\n");
		return FALSE;
	}
	if(TRUE != SetFliter(ACRCode,AMRCode))		//设置验收过滤值
	{
		printk("SetFliter Err!.\n");
		return FALSE;
	}
	if(TRUE != SetBaudRate(BaudRate))		//设置波特率
	{
		printk("SetBaudRate Err!.\n");
		return FALSE;
	}
	if(TRUE != SetOutPutMod(0x1A))			//设置驱动输出模式
	{
		printk("SetOutPutMod Err!.\n");
		return FALSE;
	}
	if(TRUE != asm_SoftRst(FALSE))			//退出软件复位模式
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
