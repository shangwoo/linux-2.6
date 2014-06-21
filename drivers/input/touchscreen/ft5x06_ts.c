/* 
 * drivers/input/touchscreen/ft5x06_ts.c
 *
 * FocalTech ft5x06 TouchScreen driver for Alpscale 9260. 
 * 
 * Copyright (c) 2013 ALPSCALE Corporation.
 * Copyright (c) 2010  Focal tech Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>   
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>

#include <mach/irqs.h>
#include <mach/hardware.h>
#include <mach/pincontrol.h>

#include "ft5x06_ts.h"


#define FTTSDEBUG 1


static u8 ft5x06_enter_factory(struct ft5x06_ts_data *ft5x0x_ts);
static u8 ft5x06_enter_work(struct ft5x06_ts_data *ft5x0x_ts);

static struct i2c_client *this_client;


#if CFG_SUPPORT_TOUCH_KEY
int tsp_keycodes[CFG_NUMOFKEYS] ={

        KEY_MENU,
        KEY_HOME,
        KEY_BACK,
        KEY_SEARCH
};

char *tsp_keyname[CFG_NUMOFKEYS] ={

        "Menu",
        "Home",
        "Back",
        "Search"
};

static bool tsp_keystatus[CFG_NUMOFKEYS];
#endif


static int ft5x06_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x06_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);
    
	return ret;
}

static int ft5x06_write_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x06_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d\n", buf[0], ret);
        return -1;
    }
    
    return 0;
}

static int ft5x06_read_reg(u8 addr, u8 *pdata)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msgs[2];

	buf[0] = addr;    //register address	
	msgs[0].addr = this_client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = buf;
	msgs[1].addr = this_client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	*pdata = buf[0];
	return ret;
  
}



static unsigned char ft5x06_read_fw_ver(void)
{
	unsigned char ver;
	ft5x06_read_reg(FT5X06_REG_FIRMID, &ver);
	return(ver);
}


#if 1  //firmware upgrade related
typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //32 bit
typedef unsigned char         FTS_BOOL;    //8 bit

typedef struct _FTS_CTP_PROJECT_SETTING_T
{
    unsigned char uc_i2C_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;    //TP panel factory ID
}FTS_CTP_PROJECT_SETTING_T;

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE               0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}


FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        pr_err("[FTS]i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}


FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        pr_err("[FTS]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}


FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}


FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/
#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
   //	#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;



    /*********Step 1:Reset  CTPM *****/
    ft5x06_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
    ft5x06_write_reg(0xfc,0x55);
	delay_qt_ms(30); 
    printk("[FTS] Step 1: Reset CTPM test\n");
   
      
    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x06_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );


    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] bootloader version = 0x%x\n", reg_val[0]);

     /*********Step 4:erase app and panel paramenter area ********************/
    cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
    delay_qt_ms(1500); 
    cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
    delay_qt_ms(100);
    printk("[FTS] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[FTS] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[FTS] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[FTS] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(300);  //make sure CTP startup normally
    
    return ERR_OK;
}

static int fts_Get_RawData(u16 RawData[][FT5x06_RX_NUM])
{
	int retval  = 0;
	int i       = 0;
	u8  devmode = 0x00;
	u8  rownum  = 0x00;
	u8 read_buffer[FT5x06_RX_NUM * 2];
	u8 read_len = 0;
    struct i2c_msg msgs;


	
	struct ft5x06_ts_data * ft5x06_ts =  i2c_get_clientdata(this_client);
	if(ft5x06_enter_factory(ft5x06_ts)<0)
	{
		pr_err("%s ERROR: could not enter factory mode", __FUNCTION__);
		retval = -1;
		goto error_return;
	}
	if(ft5x06_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	devmode |= 0x80;
	if(ft5x06_write_reg(0x00, devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	msleep(20);
	if(ft5x06_read_reg(0x00, &devmode)<0)
	{
		pr_err("%s %d ERROR: could not read register 0x00", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	if(0x00 != (devmode&0x80))
	{
		pr_err("%s %d ERROR: could not scan", __FUNCTION__, __LINE__);
		retval = -1;
		goto error_return;
	}
	pr_info("Read rawdata .......\n");
	for(rownum=0; rownum<FT5x06_TX_NUM; rownum++)
	{
		memset(read_buffer, 0x00, (FT5x06_RX_NUM * 2));

		if(ft5x06_write_reg(0x01, rownum)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}
		msleep(1);
		read_len = FT5x06_RX_NUM * 2;
		if(ft5x06_write_reg(0x10, read_len)<0)
		{
			pr_err("%s ERROR:could not write rownum", __FUNCTION__);
			retval = -1;
			goto error_return;
		}

   		msgs.addr	= this_client->addr,
   		msgs.flags	= I2C_M_RD,
   		msgs.len	= FT5x06_RX_NUM * 2,
   		msgs.buf	= read_buffer,

		retval = i2c_transfer(this_client->adapter, &msgs, 1);
		if (retval < 0) 
		{
			pr_err("%s ERROR:Could not read row %u raw data", __FUNCTION__, rownum);
			retval = -1;
			goto error_return;
		}
		for(i=0; i<FT5x06_RX_NUM; i++)
		{
			RawData[rownum][i] = (read_buffer[i<<1]<<8) + read_buffer[(i<<1)+1];
		}
	}
	
error_return:
	if(ft5x06_enter_work(ft5x06_ts)<0)
	{
		pr_err("%s ERROR:could not enter work mode ", __FUNCTION__);
		retval = -1;
	}
	
	return retval;
}

int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;


    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5x06_write_reg(0, 0x40);  
    delay_qt_ms(100);   //make sure already enter factory mode
    ft5x06_write_reg(2, 0x4);  //write command to start calibration
    delay_qt_ms(300);
    for(i=0;i<100;i++)
    {
        ft5x06_read_reg(0,&uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        delay_qt_ms(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
    ft5x06_write_reg(0, 0x40);  //goto factory mode
    delay_qt_ms(100);   //make sure already enter factory mode
    ft5x06_write_reg(2, 0x5);  //store CLB result
    delay_qt_ms(300);
    ft5x06_write_reg(0, 0x0); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
   pbt_buf = CTPM_FW;
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       printk("[FTS] upgrade failed i_ret = %d.\n", i_ret);
       //error handling ...
       //TBD
   }
   else
   {
       printk("[FTS] upgrade successfully.\n");
       fts_ctpm_auto_clb();  //start auto CLB
   }

   return i_ret;
}

unsigned char fts_ctpm_get_i_file_ver(void)
{
	unsigned int ui_sz;
	
	
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#define    FTS_SETTING_BUF_LEN        128

//update project setting
//only update these settings for COB project, or for some special case
int fts_ctpm_update_project_setting(void)
{
    unsigned char uc_i2c_addr;             //I2C slave address (8 bit address)
    unsigned char uc_io_voltage;           //IO Voltage 0---3.3v;	1----1.8v
    unsigned char uc_panel_factory_id;     //TP panel factory ID

    unsigned char buf[FTS_SETTING_BUF_LEN];
    FTS_BYTE reg_val[2] = {0};
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE  packet_buf[FTS_SETTING_BUF_LEN + 6];
    FTS_DWRD i = 0;
    int      i_ret;

    uc_i2c_addr = 0x70;
    uc_io_voltage = 0x0;
    uc_panel_factory_id = 0x5a;



    /*********Step 1:Reset  CTPM *****/
    ft5x06_write_reg(0xfc,0xaa);
    delay_qt_ms(50);
    ft5x06_write_reg(0xfc,0x55);
	delay_qt_ms(30); 
    printk("[FTS] Step 1: Reset CTPM test\n");
    

    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x06_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );
	

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[FTS] Step 3: CTPM ID,ID1 = 0x%x, ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
    }

    cmd_write(0xcd,0x0,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("bootloader version = 0x%x\n", reg_val[0]);


    /* --------- read current project setting  ---------- */
    //set read start address
    buf[0] = 0x3;
    buf[1] = 0x0;
    buf[2] = 0x78;
    buf[3] = 0x0;
    byte_write(buf, 4);
    byte_read(buf, FTS_SETTING_BUF_LEN);
    
    printk("[FTS] old setting: uc_i2c_addr = 0x%x, uc_io_voltage = %d, uc_panel_factory_id = 0x%x\n",
        buf[0],  buf[2], buf[4]);
    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
    {
        if (i % 16 == 0)     printk("\n");
        printk("0x%x, ", buf[i]);
        
    }
    printk("\n");

     /*--------- Step 4:erase project setting --------------*/
    cmd_write(0x62,0x00,0x00,0x00,1);
    delay_qt_ms(100);
   
    /*----------  Set new settings ---------------*/
    buf[0] = uc_i2c_addr;
    buf[1] = ~uc_i2c_addr;
    buf[2] = uc_io_voltage;
    buf[3] = ~uc_io_voltage;
    buf[4] = uc_panel_factory_id;
    buf[5] = ~uc_panel_factory_id;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    packet_buf[2] = 0x78;
    packet_buf[3] = 0x0;
    packet_buf[4] = 0;
    packet_buf[5] = FTS_SETTING_BUF_LEN;
    for (i = 0; i < FTS_SETTING_BUF_LEN; i++)
    {
        packet_buf[6 + i] = buf[i];
        if (i % 16 == 0)     printk("\n");
        printk("0x%x, ", buf[i]);
    }
    printk("\n");
    byte_write(&packet_buf[0],FTS_SETTING_BUF_LEN + 6);
    delay_qt_ms(100);

    /********* reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    msleep(200);

    return 0;
    
}



#if CFG_SUPPORT_AUTO_UPG

int fts_ctpm_auto_upg(void)
{
    unsigned char uc_host_fm_ver;
    unsigned char uc_tp_fm_ver;
    int           i_ret;

    uc_tp_fm_ver = ft5x06_read_fw_ver();
    uc_host_fm_ver = fts_ctpm_get_i_file_ver();
    if ( uc_tp_fm_ver == 0xa6  ||   //the firmware in touch panel maybe corrupted
         uc_tp_fm_ver < uc_host_fm_ver //the firmware in host flash is new, need upgrade
        )
    {
        msleep(100);
        printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
            uc_tp_fm_ver, uc_host_fm_ver);
        i_ret = fts_ctpm_fw_upgrade_with_i_file();    
        if (i_ret == 0)
        {
            msleep(300);
            uc_host_fm_ver = fts_ctpm_get_i_file_ver();
            printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
        }
        else
        {
            printk("[FTS] upgrade failed ret=%d.\n", i_ret);
        }
    }

    return 0;
}

#endif

#endif




static void ft5x06_ts_release(struct ft5x06_ts_data *data)
{
	input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
	input_sync(data->input_dev);
}


//read touch point information
static int ft5x06_read_data(struct ft5x06_ts_data *data)
{
    static int jk=0;
	struct ts_event *event = &data->event;
	u8 buf[CFG_POINT_READ_BUF] = {0};
	int ret = -1;
	int i;

#ifdef FTTSDEBUG
    printk("\n\n%d-- ft5x06_read_data entered...\n", jk++);
#endif	
    	
	ret = ft5x06_i2c_rxdata(buf, CFG_POINT_READ_BUF);
    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07; 

    if (event->touch_point > CFG_MAX_TOUCH_POINTS)
    {
        event->touch_point = CFG_MAX_TOUCH_POINTS;
    }


#ifdef FTTSDEBUG
    printk("touch_point: %d\n",  event->touch_point);
#endif

    for (i = 0; i < event->touch_point; i++)
    {
        event->au16_x[i] = (s16)(buf[3 + 6*i] & 0x0F)<<8 | (s16)buf[4 + 6*i];
        event->au16_y[i] = (s16)(buf[5 + 6*i] & 0x0F)<<8 | (s16)buf[6 + 6*i];
        event->au8_touch_event[i] = buf[0x3 + 6*i] >> 6;
        event->au8_finger_id[i] = (buf[5 + 6*i])>>4;
    }

    event->pressure = 200;

    return 0;
}



#if CFG_SUPPORT_TOUCH_KEY
int ft5x06_touch_key_process(struct input_dev *dev, int x, int y, int touch_event)
{
    int i;
    int key_id;
    
    if ( y < 517&&y > 497)
    {
        key_id = 1;
    }
    else if ( y < 367&&y > 347)
    {
        key_id = 0;
    }
    
    else if ( y < 217&&y > 197)
    {
        key_id = 2;
    }  
    else if (y < 67&&y > 47)
    {
        key_id = 3;
    }
    else
    {
        key_id = 0xf;
    }
    
    for(i = 0; i <CFG_NUMOFKEYS; i++ )
    {
        if(tsp_keystatus[i])
        {
            input_report_key(dev, tsp_keycodes[i], 0);
      
            printk("[FTS] %s key is release. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);

            tsp_keystatus[i] = KEY_RELEASE;
        }
        else if( key_id == i )
        {
            if( touch_event == 0)                                  // detect
            {
                input_report_key(dev, tsp_keycodes[i], 1);
                printk( "[FTS] %s key is pressed. Keycode : %d\n", tsp_keyname[i], tsp_keycodes[i]);
                tsp_keystatus[i] = KEY_PRESS;
            }
        }
    }
    return 0;
    
}    
#endif




static void ft5x06_report_value(struct ft5x06_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;

	for (i  = 0; i < event->touch_point; i++)
	{
	    if (event->au16_x[i] < SCREEN_MAX_X && event->au16_y[i] < SCREEN_MAX_Y)
	    // LCD view area
	    {
#ifdef FTTSDEBUG
            printk("--%d, x: %d, y: %d, fingerID: %d, touch_event: %d, pressure: %d\n", i, event->au16_x[i], event->au16_y[i], event->au8_finger_id[i], event->au8_touch_event[i], event->pressure);
#endif

	        input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
    		input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
    		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 1);
    		input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->au8_finger_id[i]);
    		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
    		{
    		    input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
    		}
    		else
    		{
    		    input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
    		}
	    }
	    else //maybe the touch key area
	    {
#if CFG_SUPPORT_TOUCH_KEY
            if (event->au16_x[i] >= SCREEN_MAX_X)
            {
                ft5x06_touch_key_process(data->input_dev, event->au16_x[i], event->au16_y[i], event->au8_touch_event[i]);
            }
#endif
	    }
	    			
	}
	input_sync(data->input_dev);

    if (event->touch_point == 0) {
        ft5x06_ts_release(data);
        return ; 
    }


}	


static int touch_event_handler(void *unused)
{  
     int ret = -1; 
	 struct ft5x06_ts_data *ft5x06_ts = (struct ft5x06_ts_data *)unused;

	 
     struct sched_param param = { .sched_priority = 1 };
	 sched_setscheduler(current, SCHED_FIFO, &param);

     do{
		 set_current_state(TASK_INTERRUPTIBLE); 
		 wait_event_interruptible(ft5x06_ts->waiter, ft5x06_ts->tpd_flag!=0);
						 
		 ft5x06_ts->tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);

         ret = ft5x06_read_data(ft5x06_ts);	
		 if (ret == 0) {	
			ft5x06_report_value(ft5x06_ts);
		 }
     }while(!kthread_should_stop());

     return 0;
}


static void ft5x06_ts(ulong data)
{ 
    static int TouchDetect = 0;
	struct ft5x06_ts_data *ft5x06_ts = (struct ft5x06_ts_data *)data;
	

    if ( (read_GPIO(FT5x06_INT_PORT, FT5x06_INT_PIN) == 0) && (ft5x06_ts->tpd_flag == 0 ) ){
        TouchDetect = 1;
        ft5x06_ts->tpd_flag = 1;
	    wake_up_interruptible(&ft5x06_ts->waiter);
    }
    else if( (read_GPIO(FT5x06_INT_PORT, FT5x06_INT_PIN) == 1)&&(TouchDetect == 1) ) {
        TouchDetect = 0;
        ft5x06_ts->tpd_flag = 1;
	    wake_up_interruptible(&ft5x06_ts->waiter);
    }
   
     mod_timer(&ft5x06_ts->ft5x06_timer, jiffies + HZ);//HZ/ft5x06_ts->sample_ratio_per_HZ);
     return;
}


static void ft5x06_init_timer(struct timer_list *timer)
{
	struct ft5x06_ts_data *ft5x06_ts_dataptr;	
    
	ft5x06_ts_dataptr = container_of(timer, struct ft5x06_ts_data, ft5x06_timer);
    init_timer(timer);
    timer->function = ft5x06_ts;
	timer->data = (ulong)ft5x06_ts_dataptr;
    timer->expires = jiffies + HZ;// HZ/ft5x06_ts_dataptr->sample_ratio_per_HZ;
    add_timer(timer);
}



static u8 ft5x06_enter_factory(struct ft5x06_ts_data *ft5x0x_ts)
{
	u8 regval;


	ft5x06_write_reg(0x00, 0x40);  //goto factory mode
    delay_qt_ms(100);   //make sure already enter factory mode
	if(ft5x06_read_reg(0x00, &regval)<0)
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
	else
	{
		if((regval & 0x70) != 0x40)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Factory Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	return 0;
}

static u8 ft5x06_enter_work(struct ft5x06_ts_data *ft5x0x_ts)
{
	u8 regval;
	
   	ft5x06_write_reg(0x00, 0x00); //return to normal mode 
   	msleep(100);
	
	if(ft5x06_read_reg(0x00, &regval)<0){
		pr_err("%s ERROR: could not read register\n", __FUNCTION__);
    } else{
		if((regval & 0x70) != 0x00)
		{
			pr_err("%s() - ERROR: The Touch Panel was not put in Work Mode. The Device Mode register contains 0x%02X\n", __FUNCTION__, regval);
			return -1;
		}
	}
	
	return 0;
}

static int ft5x06_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize = 0; 
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x06_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "/sdcard/%s", firmware_name);
	pr_info("filepath=%s\n", filepath);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -1;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

int fts_ctpm_fw_upgrade_with_app_file(char * firmware_name)
{
  	FTS_BYTE*     pbt_buf = FTS_NULL;
   	int i_ret; u8 fwver;

	
   	int fwsize = ft5x06_GetFirmwareSize(firmware_name);
   	if(fwsize <= 0)
   	{
   		pr_err("%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -1;
   	}
	
    //=========FW upgrade========================*/
  	pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ft5x06_ReadFirmware(firmware_name, pbt_buf))
	{
   	   pr_err("%s() - ERROR: request_firmware failed\n", __FUNCTION__);
       kfree(pbt_buf);
	   return -1;
	}
   	/*call the upgrade function*/
   	i_ret =  fts_ctpm_fw_upgrade(pbt_buf, fwsize);
   	if (i_ret != 0)
   	{
       	pr_err("%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
       //error handling ...
       //TBD
   	}
  	 else
   	{
       	pr_info("[FTS] upgrade successfully.\n");
		if(ft5x06_read_reg(FT5x06_REG_FW_VER, &fwver)>=0)
			pr_info("the new fw ver is 0x%02x\n", fwver);
      		fts_ctpm_auto_clb();  //start auto CLB
   	}
	kfree(pbt_buf);
	
   	return i_ret;
}

static ssize_t ft5x06_tpfwver_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;


	
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	if(ft5x06_read_reg(FT5x06_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&data->device_mode_mutex);
	return num_read_chars;
}

static ssize_t ft5x06_tpfwver_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x06_tprwreg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ft5x06_tprwreg_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	u16 wmreg=0;u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5];


	
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&data->device_mode_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			pr_info("please input 2 or 4 character\n");
			goto error_return;
		}
	}
	
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, (unsigned long *)&wmreg);
	
	if (0 != retval)
	{
    	pr_err("%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
    	goto error_return;
	}

	if(2 == num_read_chars)
	{
		regaddr = wmreg;
		if(ft5x06_read_reg(regaddr, &regvalue) < 0)
			pr_err("Could not read the register(0x%02x)\n", regaddr);
		else
			pr_info("the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ft5x06_write_reg(regaddr, regvalue)<0)
			pr_err("Could not write the register(0x%02x)\n", regaddr);
		else
			pr_err("Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}

error_return:
	mutex_unlock(&data->device_mode_mutex);

	return count;
}


static ssize_t ft5x06_fwupdate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from *.i
static ssize_t ft5x06_fwupdate_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	u8 uc_host_fm_ver;
	int i_ret;

	
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );	
	mutex_lock(&data->device_mode_mutex);
    i_ret = fts_ctpm_fw_upgrade_with_i_file();    
    if (i_ret == 0)
    {
        msleep(300);
        uc_host_fm_ver = fts_ctpm_get_i_file_ver();
        pr_info("%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
    }
    else
    {
        pr_err("%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
    }	
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x06_fwupgradeapp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
    return -EPERM;
}
//upgrade from app.bin
static ssize_t ft5x06_fwupgradeapp_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct ft5x06_ts_data *data = NULL;
	struct i2c_client *client;
    char fwname[128];

	
    client = container_of(dev, struct i2c_client, dev);
	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';


	mutex_lock(&data->device_mode_mutex);	
	fts_ctpm_fw_upgrade_with_app_file(fwname);
	mutex_unlock(&data->device_mode_mutex);

	return count;
}

static ssize_t ft5x06_rawdata_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = NULL;
	u16	RawData[FT5x06_TX_NUM][FT5x06_RX_NUM];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int i=0, j=0;	
	

	data = (struct ft5x06_ts_data *) i2c_get_clientdata( client );
	mutex_lock(&data->device_mode_mutex);
	if(fts_Get_RawData(RawData)<0)
	{
		sprintf(buf, "%s", "could not get rawdata\n");
	}
	else
	{
		for(i=0; i<FT5x06_TX_NUM; i++)
		{
			for(j=0; j<FT5x06_RX_NUM; j++)
			{
				num_read_chars += sprintf(&(buf[num_read_chars]), "%u ", RawData[i][j]);
			}
			buf[num_read_chars-1] = '\n';
		}
	}
	mutex_unlock(&data->device_mode_mutex);	
	
	return num_read_chars;
}

static ssize_t ft5x06_rawdata_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{

	return -EPERM;
}




/* sysfs */
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ft5x06_tpfwver_show, ft5x06_tpfwver_store);
//upgrade from *.i
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ft5x06_fwupdate_show, ft5x06_fwupdate_store);
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ft5x06_tprwreg_show, ft5x06_tprwreg_store);
//upgrade from app.bin 
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ft5x06_fwupgradeapp_show, ft5x06_fwupgradeapp_store);
static DEVICE_ATTR(ftsrawdatashow, S_IRUGO|S_IWUSR, ft5x06_rawdata_show, ft5x06_rawdata_store);


static struct attribute *ft5x06_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsrawdatashow.attr,
	NULL
};

static struct attribute_group ft5x06_attribute_group = {
	.attrs = ft5x06_attributes
};




static int 
ft5x06_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x06_ts_data *ft5x06_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value; 	
	struct task_struct *thread = NULL;
#if CFG_SUPPORT_TOUCH_KEY
    int i;
#endif
#if 1
    int jk;
#endif

	printk("[FTS] ft5x06_ts_probe, driver version is %s.\n", CFG_FTS_CTP_DRIVER_VERSION);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ft5x06_ts = kzalloc(sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!ft5x06_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	i2c_set_clientdata(this_client, ft5x06_ts);

	mutex_init(&ft5x06_ts->device_mode_mutex);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}	
	ft5x06_ts->input_dev = input_dev;

    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
    input_set_abs_params(input_dev,
			     ABS_MT_TRACKING_ID, 0, 5, 0, 0);


#if CFG_SUPPORT_TOUCH_KEY
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    input_dev->keycode = tsp_keycodes;
    for(i = 0; i < CFG_NUMOFKEYS; i++)
    {
        input_set_capability(input_dev, EV_KEY, ((int*)input_dev->keycode)[i]);
        tsp_keystatus[i] = KEY_RELEASE;
    }
#endif

	input_dev->name	= FT5X06_NAME;		//dev_name(&client->dev)
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x06_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}


#if CFG_SUPPORT_AUTO_UPG
    fts_ctpm_auto_upg();
#endif    

#if CFG_SUPPORT_UPDATE_PROJECT_SETTING
    fts_ctpm_update_project_setting();
#endif

   err = sysfs_create_group(&client->dev.kobj, &ft5x06_attribute_group);
   if (0 != err)
   {
	   dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed: %d\n", __FUNCTION__, err);
	   sysfs_remove_group(&client->dev.kobj, &ft5x06_attribute_group);
   }
   else
   {
       printk("ft5x06:%s() - sysfs_create_group() succeeded.\n", __FUNCTION__);
   }
   
    set_pin_mux(FT5x06_INT_PORT, FT5x06_INT_PIN, 0);
    read_GPIO(FT5x06_INT_PORT, FT5x06_INT_PIN);
    set_pin_mux(FT5x06_RST_PORT, FT5x06_RST_PIN, 0);
    set_GPIO(FT5x06_RST_PORT, FT5x06_RST_PIN);

#if 1
    mdelay(100);
    clear_GPIO(FT5x06_RST_PORT, FT5x06_RST_PIN);
    mdelay(100);
    set_GPIO(FT5x06_RST_PORT, FT5x06_RST_PIN);
    mdelay(100);
#endif

#if 1
    for(jk=0; jk<1; jk++)
    {
        if(ft5x06_write_reg(0, 0x40)==0)
            break;
    }
    mdelay(100);
    for(jk=0; jk<1; jk++)
    {
        if(ft5x06_write_reg(2, 0x04)==0)
          break;
    }
    mdelay(100);
    for(jk=0; jk<1; jk++)
    {
        if(ft5x06_write_reg(0, 0x00)==0)
          break;
    }
    mdelay(100);
#endif


	ft5x06_ts->tpd_flag = 0;
	init_waitqueue_head(&ft5x06_ts->waiter);
	thread = kthread_run(touch_event_handler, ft5x06_ts, FT5X06_NAME);
	if (IS_ERR(thread))
	{ 
   	  err = PTR_ERR(thread);
   	  printk(FT5X06_NAME " failed to create kernel thread.\n");	  
	} 
	ft5x06_ts->sample_ratio_per_HZ = SAMPLE_RATIO_PER_HZ; 
    ft5x06_init_timer(&ft5x06_ts->ft5x06_timer);   

 #if 0
    msleep(150);  //make sure CTP already finish startup process
    
    //get some register information
    uc_reg_value = 0x5a;
     uc_reg_value = ft5x06_read_fw_ver();
    printk("[FTS] Firmware version = 0x%x\n", uc_reg_value);
     uc_reg_value = 0x5a;
    ft5x06_read_reg(FT5X06_REG_PERIODACTIVE, &uc_reg_value);
    printk("[FTS] report rate is %dHz.\n", uc_reg_value * 10);
     uc_reg_value = 0x5a;
    ft5x06_read_reg(FT5X06_REG_THGROUP, &uc_reg_value);
    printk("[FTS] touch threshold is %d.\n", uc_reg_value * 4);
#endif

	printk("[FTS] ==probe over =\n");
    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ft5x06_ts);

exit_alloc_data_failed:
    ;

exit_check_functionality_failed:
	;

	return err;
}

static int __devexit ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *ft5x06_ts;
	
	printk("==ft5x06_ts_remove=\n");
	ft5x06_ts = i2c_get_clientdata(client);
	mutex_destroy(&ft5x06_ts->device_mode_mutex);
	input_unregister_device(ft5x06_ts->input_dev);
	kfree(ft5x06_ts);
	i2c_set_clientdata(client, NULL); 
	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{ FT5X06_NAME, 0 },{ }
};


MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

static struct i2c_driver ft5x06_ts_driver = {
	.probe		= ft5x06_ts_probe,
	.remove		= __devexit_p(ft5x06_ts_remove),
	.id_table	= ft5x06_ts_id,
	.driver	= {
		.name	= FT5X06_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x06_ts_init(void)
{
	int ret;
	
	printk("==ft5x06_ts_init==\n");
	ret = i2c_add_driver(&ft5x06_ts_driver);
	printk("ret=%d\n",ret);
	
	return ret;
}

static void __exit ft5x06_ts_exit(void)
{
	printk("==ft5x06_ts_exit==\n");
	i2c_del_driver(&ft5x06_ts_driver);
}

module_init(ft5x06_ts_init);
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL");
