/*
 * Copyright (C) 2014 Oleksij Rempel.
 *
 * Authors: Oleksij Rempel <linux@rempel-privat.de>
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


#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
//#include "sdhci-pci.h"

#include <linux/mmc/host.h>

#define DRVNAME			"au6601-pci"
#define PCI_ID_ALCOR_MICRO	0x1aea
#define PCI_ID_AU6601		0x6601

#define MHZ_TO_HZ(freq)	((freq) * 1000 * 1000)

#define DBG(f, x...) \
        printk(DRVNAME " [%s()]: " f, __func__,## x)

#define REG_00	0x00
#define REG_05	0x05
#define REG_08	0x08
#define REG_0C	0x0c
#define REG_10	0x10
#define REG_23	0x23
#define REG_24	0x24
#define REG_30	0x30
#define REG_34	0x34
#define REG_38	0x38
#define REG_3C	0x3C
#define REG_51	0x51
#define REG_52	0x52
#define REG_61	0x61
#define REG_63	0x63
#define REG_69	0x69
#define REG_6C	0x6c
#define REG_70	0x70
#define REG_72	0x72
#define REG_74	0x74
#define REG_75	0x75
#define REG_76	0x76
#define REG_77	0x77
#define REG_79	0x79
#define REG_7A	0x7a
#define REG_7B	0x7b
#define REG_7C	0x7c
#define REG_7D	0x7d
#define REG_7F	0x7f
#define REG_81	0x81
#define REG_82	0x82
#define REG_83	0x83
#define REG_84	0x84
#define REG_85	0x85
#define REG_86	0x86
#define AU6601_INT_STATUS	0x90 /* IRQ intmask */
#define AU6601_INT_ENABLE	0x94
#define REG_A1	0xa1
#define REG_A2	0xa2
#define REG_A3	0xa3
#define REG_B0	0xb0
#define REG_B4	0xb4

/* identical or almost identical with sdhci.h */
#define  AU6601_INT_RESPONSE		0x00000001	/* ok */
#define  AU6601_INT_DATA_END		0x00000002	/* fifo, ok */
#define  AU6601_INT_BLK_GAP		0x00000004
#define  AU6601_INT_DMA_END		0x00000008
#define  AU6601_INT_SPACE_AVAIL		0x00000010
#define  AU6601_INT_DATA_AVAIL		0x00000020	/* fifo, ok */
#define  AU6601_INT_CARD_REMOVE		0x00000040
#define  AU6601_INT_CARD_INSERT		0x00000080	/* 0x40 and 0x80 flip */
#define  AU6601_INT_CARD_INT		0x00000100
#define  AU6601_INT_ERROR		0x00008000	/* ok */
#define  AU6601_INT_TIMEOUT		0x00010000	/* seems to be ok */
#define  AU6601_INT_CRC			0x00020000	/* seems to be ok */
#define  AU6601_INT_END_BIT		0x00040000
#define  AU6601_INT_INDEX		0x00080000
#define  AU6601_INT_DATA_TIMEOUT	0x00100000
#define  AU6601_INT_DATA_CRC		0x00200000
#define  AU6601_INT_DATA_END_BIT	0x00400000
#define  AU6601_INT_BUS_POWER		0x00800000
#define  AU6601_INT_ACMD12ERR		0x01000000
#define  AU6601_INT_ADMA_ERROR		0x02000000

#define  AU6601_INT_NORMAL_MASK		0x00007FFF
#define  AU6601_INT_ERROR_MASK		0xFFFF8000

/* magic 0xF0001 */
#define  AU6601_INT_CMD_MASK	(AU6601_INT_RESPONSE | AU6601_INT_TIMEOUT | \
		AU6601_INT_CRC | AU6601_INT_END_BIT | AU6601_INT_INDEX)
/* magic 0x70003A */
#define  AU6601_INT_DATA_MASK	(AU6601_INT_DATA_END | AU6601_INT_DMA_END | \
		AU6601_INT_DATA_AVAIL | AU6601_INT_SPACE_AVAIL | \
		AU6601_INT_DATA_TIMEOUT | AU6601_INT_DATA_CRC | \
		AU6601_INT_DATA_END_BIT)
#define AU6601_INT_ALL_MASK	((uint32_t)-1)

#define AU6601_MIN_CLOCK		(150 * 1000)
#define AU6601_MAX_CLOCK		MHZ_TO_HZ(200)
#define AU6601_MAX_BLOCK_LENGTH		2048
#define AU6601_MAX_BLOCK_COUNT		65536

u32 reg_list[][4] = {
	{ 0, 0, 0, 0},
	{ REG_00, 0, 0, 4},
	{ REG_05, 0, 0, 2},
//	{ REG_08, 0, 0, 4},
	{ REG_0C, 0, 0, 1},
	{ REG_10, 0, 0, 4},
	{ REG_23, 0, 0, 1},
	{ REG_24, 0, 0, 4},
	{ REG_30, 0, 0, 4},
	{ REG_34, 0, 0, 4},
	{ REG_38, 0, 0, 4},
	{ REG_3C, 0, 0, 4},
	{ REG_51, 0, 0, 1},
	{ REG_52, 0, 0, 1},
	{ REG_61, 0, 0, 1},
	{ REG_63, 0, 0, 1},
	{ REG_69, 0, 0, 1},
	{ REG_6C, 0, 0, 4},
	{ REG_70, 0, 0, 1},
	{ REG_72, 0, 0, 2},
	{ REG_74, 0, 0, 2},
	{ REG_75, 0, 0, 1},
	{ REG_76, 0, 0, 1},
	{ REG_77, 0, 0, 1},
	{ REG_79, 0, 0, 1},
	{ REG_7A, 0, 0, 1},
	{ REG_7B, 0, 0, 1},
	{ REG_7C, 0, 0, 1},
	{ REG_7D, 0, 0, 1},
	{ REG_7F, 0, 0, 1},
	{ REG_81, 0, 0, 1},
	{ REG_82, 0, 0, 1},
	{ REG_83, 0, 0, 1},
	{ REG_84, 0, 0, 2},
	{ REG_85, 0, 0, 1},
	{ REG_86, 0, 0, 1},
	{ AU6601_INT_STATUS, 0, 0, 4},
	{ AU6601_INT_ENABLE, 0, 0, 4},
	{ REG_A1, 0, 0, 1},
	{ REG_A2, 0, 0, 1},
	{ REG_A3, 0, 0, 1},
	{ REG_B0, 0, 0, 4},
	{ REG_B4, 0, 0, 4},
};

struct au6601_host {
	struct pci_dev *pdev;
	void __iomem *iobase;

	struct mmc_host *mmc;
	struct mmc_request *mrq;
	struct mmc_command *cmd;
	struct mmc_data *data;

	spinlock_t lock;

	struct tasklet_struct card_tasklet;
	struct tasklet_struct finish_tasklet;

	struct timer_list timer;
};

static const struct pci_device_id pci_ids[] = {
	{
		.vendor         = PCI_ID_ALCOR_MICRO,
		.device         = PCI_ID_AU6601,
		.subvendor      = PCI_ANY_ID,
		.subdevice      = PCI_ANY_ID,
	},
	{ /* end: all zeroes */ },
};
MODULE_DEVICE_TABLE(pci, pci_ids);

static unsigned char au6601_readb(struct au6601_host *host,
			  unsigned int reg)
{
	unsigned char val = readb(host->iobase + reg);
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	return val;
}

#if 0
static unsigned short au6601_readw(struct au6601_host *host,
			  unsigned int reg)
{
	unsigned short val = readw(host->iobase + reg);
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	return val;
}
#endif

static unsigned int au6601_readl(struct au6601_host *host,
			  unsigned int reg)
{
	unsigned int val = readl(host->iobase + reg);
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	return val;
}

static void au6601_writeb(struct au6601_host *host,
			  u8 val, unsigned int reg)
{
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	writeb(val, host->iobase + reg);
}

static void au6601_writew(struct au6601_host *host,
			  u16 val, unsigned int reg)
{
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	writew(val, host->iobase + reg);
}

static void au6601_writel(struct au6601_host *host,
			  u32 val, unsigned int reg)
{
	printk("%s: addr = %x, data = %x\n", __func__, reg, val);
	writel(val, host->iobase + reg);
}

static void au6601_reg_snap(struct au6601_host *host)
{
	int a, b;

	b = reg_list[0][1] ? 2 : 1;
	reg_list[0][b] = 1;

	/* grub all needed regs */
	for (a = 1; a < ARRAY_SIZE(reg_list); a++) {
		if (reg_list[a][3] == 1)
			reg_list[a][b] = readb(host->iobase + reg_list[a][0]);
		else if (reg_list[a][3] == 2)
			reg_list[a][b] = readw(host->iobase + reg_list[a][0]);
		else if (reg_list[a][3] == 4)
			reg_list[a][b] = readl(host->iobase + reg_list[a][0]);
		else
			printk("-- wrong lenght\n");
	}

	/* if we have two version, compare them */
	if (reg_list[0][1] && reg_list[0][2]) {
		for (a = 1; a < ARRAY_SIZE(reg_list); a++) {
			if (reg_list[a][1] != reg_list[a][2])
				printk("-- reg %02x: %08x %s %08x\n",
					reg_list[a][0],
					reg_list[a][1],
					b == 1 ? "<" : ">",
					reg_list[a][2]);
		}

		if (b == 1)
			reg_list[0][2] = 0;
		else
			reg_list[0][1] = 0;
	}
}

#if 0
static void au6601_regdump(struct au6601_host *host)
{
	int i, a = 0;
	for (i = 0; i < 0xff; i++) {
		printk("0x%02x ", readb(host->iobase + i));
		if (a == 15) {
			printk("\n");
			a = 0;
		} else
			a++;
	}
}
#endif

static void au6601_clear_set_irqs(struct au6601_host *host, u32 clear, u32 set)
{
	u32 ier;

	ier = au6601_readl(host, AU6601_INT_ENABLE);
	ier &= ~clear;
	ier |= set;
	au6601_writel(host, ier, AU6601_INT_ENABLE);
}

#if 0
static void au6601_unmask_irqs(struct au6601_host *host, u32 irqs)
{
	au6601_clear_set_irqs(host, 0, irqs);
}

static void au6601_mask_irqs(struct au6601_host *host, u32 irqs)
{
	au6601_clear_set_irqs(host, irqs, 0);
}
# endif

/* val = 0x1 abort command; 0x8 abort data? */
static void au6601_wait_reg_79(struct au6601_host *host, u8 val)
{
	int i;
	au6601_writeb(host, val | 0x80, REG_79);
	/* what is bets value here? 500? */
	for (i = 0; i < 500; i++) {
		if (!(au6601_readb(host, REG_79) & val))
			return;
		msleep(1);
	}
	printk("%s: timeout\n", __func__);
}

/*
 * 0x1 + set - pull up CMD pin
 * 0x8 + set - pull up VCC pin
 */

static void au6601_toggle_reg70(struct au6601_host *host, unsigned int value, unsigned int set)
{
	u8 tmp1, tmp2;
 
	tmp1 = au6601_readb(host, REG_70);
	tmp2 = au6601_readb(host, REG_7A);
	if (set) {
		au6601_writeb(host, tmp1 | value, REG_70);
		au6601_writeb(host, tmp2 | value, REG_7A);
	} else {
		au6601_writeb(host, tmp2 & ~value, REG_7A);
		au6601_writeb(host, tmp1 & ~value, REG_70);
	}
}

static void au6601_set_freg_pre(struct au6601_host *host)
{
	u8 tmp;
	au6601_toggle_reg70(host, 0x8, 0);
	au6601_writeb(host, 0, REG_85);
	au6601_writeb(host, 0x31, REG_7B);
	au6601_writeb(host, 0x33, REG_7C);
//	au6601_writeb(host, 0, REG_7D);
	au6601_writeb(host, 1, REG_75);
	au6601_writeb(host, 0, REG_85);
	au6601_writeb(host, 0x30, REG_86);
	au6601_writeb(host, 0, REG_82);

	/* other possible variant tmp | 0xc0 */
	tmp = au6601_readb(host, REG_86);
	au6601_writeb(host, tmp & 0x3f, REG_86);
}

static void au6601_set_clock(struct au6601_host *host, unsigned int clock)
{
	unsigned int div, mult;

	/* FIXME: mesuered and calculated values are different.
	 * the clock is unstable in some mult/div combinations.
	 */
	if (clock >= MHZ_TO_HZ(208)) {
		mult = 0xb0;	/* 30 * ? / 2 = ?MHz */
		div = 2;
	} else if (clock >= MHZ_TO_HZ(194)) {
		mult = 0x30;	/* 30 * 14 / 2 = 210MHz */
		div = 2;
	} else if (clock >= MHZ_TO_HZ(130)) {
		mult = 0x30;	/* 30 * 14 / 3 = 140MHz */
		div = 3;
	} else if (clock >= MHZ_TO_HZ(100)) {
		mult = 0x30;	/* 30 * 14 / 4 = 105MHz */
		div = 4;
	} else if (clock >= MHZ_TO_HZ(80)) {
		mult = 0x30;	/* 30 * 14 / 5 = 84MHz */
		div = 5;
	} else if (clock >= MHZ_TO_HZ(60)) {
		mult = 0x30;	/* 30 * 14 / 7 = 60MHz */
		div = 7;
	} else if (clock >= MHZ_TO_HZ(50)) {
		mult = 0x10;	/* 30 * 2 / 1 = 60MHz */
		div = 1;
	} else if (clock >= MHZ_TO_HZ(40)) {
		mult = 0x30;	/* 30 * 14 / 10 = 42MHz */
		div = 10;
	} else if (clock >= MHZ_TO_HZ(25)) {
		mult = 0x10;	/* 30 * 2 / 2 = 30MHz */
		div = 2;
	} else if (clock >= MHZ_TO_HZ(20)) {
		mult = 0x20;	/* 30 * 4 / 7 = 17MHz */
		div = 7;
	} else if (clock >= MHZ_TO_HZ(10)) {
		mult = 0x10;	/* 30 * 2 / 5 = 12MHz */
		div = 5;
	} else if (clock >= MHZ_TO_HZ(5)) {
		mult = 0x10;	/* 30 * 2 / 10 = 6MHz */
		div = 10;
	} else if (clock >= MHZ_TO_HZ(1)) {
		mult = 0x0;	/* 30 / 16 = 1,8 MHz */
		div = 16;
	} else {
		mult = 0x0;	/* reversed 150 * 200 = 30MHz */
		div = 200;	/* 150 KHZ mesured */
	}
	au6601_writew(host, (div - 1) << 8 | 1 | mult, REG_72);
}

static void au6601_finish_command(struct au6601_host *host)
{
	struct mmc_command *cmd = host->cmd;
        u32 dwdata0 = au6601_readl(host, REG_30);
        u32 dwdata1 = au6601_readl(host, REG_30);
        u32 dwdata2 = au6601_readl(host, REG_30);
        u32 dwdata3 = au6601_readl(host, REG_30);

        if (cmd->flags & MMC_RSP_136) {
                cmd->resp[0] = ((u8) (dwdata1)) |
                    (((u8) (dwdata0 >> 24)) << 8) |
                    (((u8) (dwdata0 >> 16)) << 16) |
                    (((u8) (dwdata0 >> 8)) << 24);

                cmd->resp[1] = ((u8) (dwdata2)) |
                    (((u8) (dwdata1 >> 24)) << 8) |
                    (((u8) (dwdata1 >> 16)) << 16) |
                    (((u8) (dwdata1 >> 8)) << 24);

                cmd->resp[2] = ((u8) (dwdata3)) |
                    (((u8) (dwdata2 >> 24)) << 8) |
                    (((u8) (dwdata2 >> 16)) << 16) |
                    (((u8) (dwdata2 >> 8)) << 24);

                cmd->resp[3] = 0xff |
                    ((((u8) (dwdata3 >> 24))) << 8) |
                    (((u8) (dwdata3 >> 16)) << 16) |
                    (((u8) (dwdata3 >> 8)) << 24);
        } else {
                dwdata0 >>= 8;
                cmd->resp[0] = ((dwdata0 & 0xff) << 24) |
                    (((dwdata0 >> 8) & 0xff) << 16) |
                    (((dwdata0 >> 16) & 0xff) << 8) | (dwdata1 & 0xff);

                dwdata1 >>= 8;
                cmd->resp[1] = ((dwdata1 & 0xff) << 24) |
                    (((dwdata1 >> 8) & 0xff) << 16) |
                    (((dwdata1 >> 16) & 0xff) << 8);
        }
}

static void au6601_send_cmd(struct au6601_host *host,
			    struct mmc_command *cmd)
{
	u8 ctrl; /*some mysterious flags and control */
	unsigned long timeout;

        timeout = jiffies;
        if (!cmd->data && cmd->cmd_timeout_ms > 9000)
                timeout += DIV_ROUND_UP(cmd->cmd_timeout_ms, 1000) * HZ + HZ;
        else
                timeout += 10 * HZ;
        mod_timer(&host->timer, timeout);

        host->cmd = cmd;

	/* FIXME in some cases we write here REG_6C and REG_83 */

	//writel(0x200, host->iobase + REG_6C);
	//au6601_writeb(0x80, host->iobase + REG_83);

	au6601_writeb(host, cmd->opcode | 0x40, REG_23);
	au6601_writel(host, cmd->arg, REG_24);
	//au6601_writel(host, cpu_to_be32(cmd->arg), REG_24);

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		ctrl |= 0x0;
		break;
	case MMC_RSP_R1:
		ctrl |= 0x40;
		break;
	case MMC_RSP_R1B:
		ctrl |= 0x40 | 0x10;
		break;
	case MMC_RSP_R2:
		ctrl |= 0xc0;
		break;
	case MMC_RSP_R3:
		ctrl |= 0x80;
		break;
	default:
		pr_err("%s: cmd->flag is not valid\n", mmc_hostname(host->mmc));
		break;
	}

	/* VIA_CRDR_SDCTRL_STOP? */
	//if (cmd->opcode == 12)
	//	ctrl |= 0x10;

	au6601_writeb(host, ctrl | 0x20, REG_81);
	/* should be handled by interrupt, not msleep */
	//msleep(10);
	//au6601_wait_reg_79(host, 0x1);
}

static void some_seq(struct au6601_host *host)
{
	au6601_writeb(host, 0x0, REG_05);
	au6601_writeb(host, 0x1, REG_75);
	au6601_clear_set_irqs(host, AU6601_INT_ALL_MASK,
		AU6601_INT_CMD_MASK | AU6601_INT_DATA_MASK |
		AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE |
		AU6601_INT_CARD_INT | AU6601_INT_BUS_POWER);
	au6601_writel(host, 0x0, REG_82);
}

#if 0
static void testing(struct au6601_host *host)
{
	struct mmc_command cmd;
	int i;

	/* this sequence happens on forst command */
	au6601_writeb(host, 0x80, REG_83);
	au6601_writeb(host, 0x7d, REG_69);
	au6601_readb(host, REG_74);

	cmd.opcode = 0;
	cmd.arg = 0;
	au6601_send_cmd(host, &cmd, 0x0);

	some_seq(host);
	cmd.opcode = 8;
	cmd.arg = 0xaa010000;
	au6601_send_cmd(host, &cmd, 0x80);

	for (i = 0; i < 20; i++) {
	//some_seq(host);
	cmd.opcode = 0x77;
	cmd.arg = 0x0;
	au6601_send_cmd(host, &cmd, 0x40);

	//some_seq(host);
	cmd.opcode = 0x69;
	cmd.arg = 0xfc51;
	au6601_send_cmd(host, &cmd, 0xa0);
	}

	cmd.opcode = 2;
	cmd.arg = 0x0;
	au6601_send_cmd(host, &cmd, 0xc0);

	cmd.opcode = 3;
	cmd.arg = 0x0;
	au6601_send_cmd(host, &cmd, 0x60);

	cmd.opcode = 0x49;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0xe0);

	cmd.opcode = 0x47;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0x60);

	cmd.opcode = 0x4d;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0x60);

	cmd.opcode = 0x77;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0x60);

	au6601_writeb(host, 0x8, REG_6C);
	au6601_writeb(host, 0x1, REG_83);
	cmd.opcode = 0x73;
	cmd.arg = 0x0;
	au6601_send_cmd(host, &cmd, 0x60);
	au6601_readl(host, REG_08);
	au6601_readl(host, REG_08);
	msleep(1);

	cmd.opcode = 0x77;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0x60);

	cmd.opcode = 0x46;
	cmd.arg = 0x2000000;
	au6601_send_cmd(host, &cmd, 0x60);

	au6601_writel(host, 0x9949000, REG_00);
	au6601_readl(host, REG_00);
	cmd.opcode = 0x4d;
	cmd.arg = 0x0700;
	au6601_send_cmd(host, &cmd, 0x60);

	au6601_writeb(host, 0x0, REG_83);
	cmd.opcode = 0x51;
	cmd.arg = 0x0;
	au6601_send_cmd(host, &cmd, 0x60);

	au6601_writel(host, 0x200, REG_6C);
	au6601_writeb(host, 0x41, REG_83);
}
#endif

/*****************************************************************************\
 *                                                                           *
 * Interrupt handling                                                        *
 *                                                                           *
\*****************************************************************************/

static void au6601_cmd_irq(struct au6601_host *host, u32 intmask)
{
	BUG_ON(intmask == 0);

	if (!host->cmd) {
		pr_err("%s: Got command interrupt 0x%08x even "
			"though no command operation was in progress.\n",
			mmc_hostname(host->mmc), (unsigned)intmask);
		//sdhci_dumpregs(host);
		return;
	}

	if (intmask & AU6601_INT_TIMEOUT)
		host->cmd->error = -ETIMEDOUT;
	else if (intmask & (AU6601_INT_CRC | AU6601_INT_END_BIT |
			AU6601_INT_INDEX))
		host->cmd->error = -EILSEQ;

	if (host->cmd->error) {
		tasklet_schedule(&host->finish_tasklet);
		return;
	}

	/*
	 * The host can send and interrupt when the busy state has
	 * ended, allowing us to wait without wasting CPU cycles.
	 * Unfortunately this is overloaded on the "data complete"
	 * interrupt, so we need to take some care when handling
	 * it.
	 *
	 * Note: The 1.0 specification is a bit ambiguous about this
	 *       feature so there might be some problems with older
	 *       controllers.
	 */
	if (host->cmd->flags & MMC_RSP_BUSY) {
		if (host->cmd->data)
			printk("Cannot wait for busy signal when also "
				"doing a data transfer");
	}

	if (intmask & AU6601_INT_RESPONSE)
		au6601_finish_command(host);
}


static irqreturn_t au6601_irq(int irq, void *d)
{
	struct au6601_host *host = d;
	irqreturn_t ret = IRQ_HANDLED;
	u32 intmask;

	//disable_irq_nosync(irq);
	spin_lock(&host->lock);

	au6601_reg_snap(host);
	intmask = au6601_readl(host, AU6601_INT_STATUS);

	/* some thing bad */
	if (!intmask || intmask == 0xffffffff) {
		ret = IRQ_NONE;
		goto exit;
	}

	if (intmask & (AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE)) {
		/* this check can be remove */
		if (intmask & AU6601_INT_CARD_REMOVE) {
			printk("IRQ for removed card\n");
		} else {
			printk("IRQ for inserted card\n");
		}
		au6601_writeb(host, intmask & (AU6601_INT_CARD_INSERT |
			      AU6601_INT_CARD_REMOVE), AU6601_INT_STATUS);
		intmask &= ~(AU6601_INT_CARD_INSERT | AU6601_INT_CARD_REMOVE);
		//tasklet_schedule(&host->card_tasklet);
	}
	if (intmask & 0x110000)
		printk("0x110000 (DATA/CMD timeout) got IRQ with %x\n", intmask);

	if (intmask & AU6601_INT_CMD_MASK) {
		printk("0xF0001 (CMD) got IRQ with %x ", intmask);
		if (intmask & 0x1)
			printk("ACK\n");
		else if (intmask & 0x10000)
			printk("REJECT\n");
		else
			printk("Uknown\n");

		au6601_writel(host, intmask & AU6601_INT_CMD_MASK,
			      AU6601_INT_STATUS);
		intmask &= ~AU6601_INT_CMD_MASK;
		au6601_cmd_irq(host, intmask & AU6601_INT_CMD_MASK);
	}

	if (intmask & AU6601_INT_DATA_MASK) {
		printk("0x70003A (DATA/FIFO) got IRQ with %x", intmask);
		au6601_writel(host, intmask & AU6601_INT_DATA_MASK,
			      AU6601_INT_STATUS);
		intmask &= ~AU6601_INT_DATA_MASK;
		//au6601_data_irq(host, intmask & AU6601_INT_DATA_MASK);
	}

	if (intmask & 0x100) {
		printk("0x100 (card INT) got IRQ with %x\n", intmask);
		au6601_writel(host, 0x100, AU6601_INT_STATUS);
		intmask &= ~0x100;
	}

	if (intmask & 0xFFFF7FFF) {
		printk("0xFFFF7FFF got IRQ with %x\n", intmask);
		au6601_writel(host, intmask & 0xFFFF7FFF, AU6601_INT_STATUS);
	}

exit:
	spin_unlock(&host->lock);
	//enable_irq(irq);
	return ret;
}

static void au6601_init(struct au6601_host *host)
{
//	u32 scratch;
//	u16 v3 = 0xa, Valuea = 0x10;

	au6601_writeb(host, 0, REG_7F);
	au6601_readb(host, REG_7F);
	au6601_writeb(host, 1, REG_7F);
	au6601_readb(host, REG_7F);

	au6601_readb(host, REG_77);

	au6601_writeb(host, 0, REG_74);
	/* set ExtPadDrive size */
	au6601_writeb(host, 68, REG_7B);
	au6601_writeb(host, 68, REG_7C);
//	au6601_writeb(host, 0, REG_7D);

	//au6601_toggle_reg70(host, 0x8, 1);


	/* why do we need this read? */
	au6601_readb(host, REG_76);
	au6601_writeb(host, 0, REG_76);
	/* disable DlinkMode? disabled by default. */
	au6601_writeb(host, 0x80, REG_76);
	au6601_readb(host, REG_76);

	au6601_wait_reg_79(host, 0x1);

	/* first sequence after reg_79 check. Same sequence is used on
	 * olmost every command. */
	some_seq(host);
	au6601_wait_reg_79(host, 0x8);

	au6601_writeb(host, 0x0, REG_05);
	au6601_writeb(host, 0x0, REG_85);
	au6601_writeb(host, 0x8, REG_75);
	au6601_writel(host, 0x3d00fa, REG_B4);
//	au6601_writeb(host, 0x0, REG_61);
//	au6601_writeb(host, 0x0, REG_63);

	/* if sd was detected */
	au6601_toggle_reg70(host, 0x1, 0);

}

static void au6601_sdc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct au6601_host *host;
	unsigned long flags;

	host = mmc_priv(mmc);
	spin_lock_irqsave(&host->lock, flags);

	/* check if card is present? then send command and data */
	au6601_send_cmd(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void au6601_sdc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct au6601_host *host;
	unsigned long flags;

	host = mmc_priv(mmc);
	spin_lock_irqsave(&host->lock, flags);

	au6601_set_freg_pre(host);
	au6601_set_clock(host, ios->clock);

	spin_unlock_irqrestore(&host->lock, flags);

	if (ios->power_mode != MMC_POWER_OFF)
		au6601_toggle_reg70(host, 0x1, 1);
	else
		au6601_toggle_reg70(host, 0x1, 0);
}

#if 0
static int au6601_sdc_get_ro(struct mmc_host *mmc)
{
	return 0;
}
#endif

static const struct mmc_host_ops au6601_sdc_ops = {
        .request = au6601_sdc_request,
        .set_ios = au6601_sdc_set_ios,
       // .get_ro = au6601_sdc_get_ro,
};

/*****************************************************************************\
 *                                                                           *
 * Tasklets                                                                  *
 *                                                                           *
\*****************************************************************************/

static void au6601_tasklet_card(unsigned long param)
{
	struct au6601_host *host = (struct au6601_host*)param;

	//sdhci_card_event(host->mmc);

	mmc_detect_change(host->mmc, msecs_to_jiffies(200));
}

static void au6601_tasklet_finish(unsigned long param)
{
	struct au6601_host *host;
	unsigned long flags;
	struct mmc_request *mrq;

	host = (struct au6601_host*)param;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * If this tasklet gets rescheduled while running, it will
	 * be run again afterwards but without any active request.
	 */
	if (!host->mrq) {
		spin_unlock_irqrestore(&host->lock, flags);
		return;
	}

	del_timer(&host->timer);

	mrq = host->mrq;

	/*
	 * The controller needs a reset of internal state machines
	 * upon error conditions.
	 */
	if ((mrq->cmd && mrq->cmd->error) ||
		 (mrq->data && (mrq->data->error ||
		  (mrq->data->stop && mrq->data->stop->error)))) {

		// do reset over reg_79?
//		sdhci_reset(host, SDHCI_RESET_CMD);
//		sdhci_reset(host, SDHCI_RESET_DATA);
	}

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

        mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);

	mmc_request_done(host->mmc, mrq);
}

static void au6601_timeout_timer(unsigned long data)
{
	struct au6601_host *host;
	unsigned long flags;

	host = (struct au6601_host*)data;

	spin_lock_irqsave(&host->lock, flags);

	if (host->mrq) {
		pr_err("%s: Timeout waiting for hardware "
			"interrupt.\n", mmc_hostname(host->mmc));
		//sdhci_dumpregs(host);

		if (host->data) {
			host->data->error = -ETIMEDOUT;
			//sdhci_finish_data(host);
		} else {
			if (host->cmd)
				host->cmd->error = -ETIMEDOUT;
			else
				host->mrq->cmd->error = -ETIMEDOUT;

			tasklet_schedule(&host->finish_tasklet);
		}
	}

	mmiowb();
	spin_unlock_irqrestore(&host->lock, flags);
}



static void au6601_init_host(struct au6601_host *host)
{
	struct mmc_host *mmc = host->mmc;
	//void __iomem *addrbase;
	//u32 lenreg;
	//u32 status;

	//init_timer(&host->timer);
	//host->timer.data = (unsigned long)host;
	//host->timer.function = via_sdc_timeout;

	spin_lock_init(&host->lock);

	mmc->f_min = AU6601_MIN_CLOCK;
	mmc->f_max = AU6601_MAX_CLOCK;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	//mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	mmc->caps = MMC_CAP_4_BIT_DATA;
	mmc->ops = &au6601_sdc_ops;

	/*Hardware cannot do scatter lists*/
	mmc->max_segs = 1;

	mmc->max_blk_size = AU6601_MAX_BLOCK_LENGTH;
	mmc->max_blk_count = AU6601_MAX_BLOCK_COUNT;

	mmc->max_seg_size = mmc->max_blk_size * mmc->max_blk_count;
	mmc->max_req_size = mmc->max_seg_size;
}


static int au6601_pci_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct mmc_host *mmc;
	struct au6601_host *host;
        int ret, bar;
	//u32 scratch_32;

        BUG_ON(pdev == NULL);
        BUG_ON(ent == NULL);

        dev_info(&pdev->dev, "AU6601 controller found [%04x:%04x] (rev %x)\n",
                 (int)pdev->vendor, (int)pdev->device, (int)pdev->revision);

        if (!(pci_resource_flags(pdev, bar) & IORESOURCE_MEM)) {
                dev_err(&pdev->dev, "BAR %d is not iomem. Aborting.\n", bar);
                return -ENODEV;
        }


        ret = pcim_enable_device(pdev);
        if (ret)
                return ret;

        host = devm_kzalloc(&pdev->dev, sizeof(struct au6601_host),
			    GFP_KERNEL);
        if (!host)
		return -ENOMEM;

	/* FIXME: are there no managed version of mmc_alloc_host? */
	mmc = mmc_alloc_host(0, &pdev->dev);
	if (!mmc)
		return -ENOMEM;
	host->mmc = mmc;
        host->pdev = pdev;

        ret = pci_request_region(pdev, bar, DRVNAME);
        if (ret) {
                dev_err(&pdev->dev, "cannot request region\n");
		return -ENOMEM;
	}

//	host->iobase = pci_ioremap_bar(pdev, bar);
	host->iobase = pcim_iomap(pdev, bar, 0);
	if (!host->iobase)
		return -ENOMEM;

	ret = devm_request_irq(&pdev->dev, 17, au6601_irq,
				IRQF_TRIGGER_FALLING, "au6601 host",
				host);

	if (ret) {
		dev_err(&pdev->dev, "failed to get irq for data line\n");
		return -ENOMEM;
	}

        pci_set_drvdata(pdev, host);

	/*
	 * Init tasklets.
	 */
	tasklet_init(&host->card_tasklet,
		au6601_tasklet_card, (unsigned long)host);
	tasklet_init(&host->finish_tasklet,
		au6601_tasklet_finish, (unsigned long)host);
	setup_timer(&host->timer, au6601_timeout_timer, (unsigned long)host);

	au6601_init_host(host);
	au6601_reg_snap(host);
	au6601_init(host);

	mmc_add_host(mmc);

	//au6601_set_ios(host);
	//au6601_regdump(host);
	//testing(host);
	//au6601_reg_snap(host);


        return 0;
}


static void au6601_pci_remove(struct pci_dev *pdev)
{
	struct au6601_host *host;

	host = pci_get_drvdata(pdev);

	au6601_writeb(host, 0x0, REG_76);
	au6601_clear_set_irqs(host, AU6601_INT_ALL_MASK, 0);

	au6601_toggle_reg70(host, 0x1, 0);

	au6601_writeb(host, 0x0, REG_85);
	au6601_writeb(host, 0x0, REG_B4);

	au6601_toggle_reg70(host, 0x8, 0);

	del_timer_sync(&host->timer);
	tasklet_kill(&host->card_tasklet);
	tasklet_kill(&host->finish_tasklet);

	mmc_free_host(host->mmc);
	// do some thing here
}


static struct pci_driver au6601_driver = {
        .name =         DRVNAME,
        .id_table =     pci_ids,
        .probe =        au6601_pci_probe,
        .remove =       au6601_pci_remove,
};

module_pci_driver(au6601_driver);

MODULE_AUTHOR("Oleksij Rempel <linux@rempel-privat.de>");
MODULE_DESCRIPTION("Alcor Micro AU6601 Secure Digital Host Controller Interface PCI driver");
MODULE_LICENSE("GPL");
