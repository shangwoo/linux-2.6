/*
 * linux/include/asm-arm/arch-as3310/io.h
 *
 */

#ifndef __ASM_ARM_ARCH_IO_H
#define __ASM_ARM_ARCH_IO_H


//#define IO_SPACE_LIMIT 0xffffffff

/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
//#define __io(a)			((void __iomem *)(PCIO_BASE + (a)))
//#define __mem_pci(a)		(a)
//#define __mem_isa(a)		(a)

/*
 * ----------------------------------------------------------------------------
 * I/O mapping
 * ----------------------------------------------------------------------------
 */

#define PCIO_BASE	0

#define IO_PHYS		0x80000000	/* Virtual IO = 0xf0000000 */
#define IO_SIZE		0x00800000
#define IO_VIRADDR  0xf0000000
#define IO_BASE     IOMEM(IO_VIRADDR)
#define IO_OFFSET   (IO_VIRADDR-IO_PHYS)
#define IO_ADDRESS(pa)    (IO_OFFSET+(pa))
#define io_p2v(pa)  ((pa)+IO_OFFSET)
#define io_v2p(va)  ((va)-IO_OFFSET)

#ifndef __ASSEMBLER__

/*
 * Functions to access the as3310 IO region
 *
 * NOTE: - Use omap_read/write[bwl] for physical register addresses
 *	 - Use __raw_read/write[bwl]() for virtual register addresses
 *	 - Use IO_ADDRESS(phys_addr) to convert registers to virtual addresses
 *	 - DO NOT use hardcoded virtual addresses to allow changing the
 *	   IO address space again if needed
 */

#define as3310_readb(a)		(*(volatile unsigned char  *)IO_ADDRESS(a))
#define as3310_readw(a)		(*(volatile unsigned short *)IO_ADDRESS(a))
#define as3310_readl(a)		(*(volatile unsigned int   *)IO_ADDRESS(a))

#define as3310_writeb(v,a)	(*(volatile unsigned char  *)IO_ADDRESS(a) = (v))
#define as3310_writew(v,a)	(*(volatile unsigned short *)IO_ADDRESS(a) = (v))
#define as3310_writel(v,a)	(*(volatile unsigned int   *)IO_ADDRESS(a) = (v))


/*LCD releated*/
#define as3310_readb_lcd(a)	(*(volatile unsigned char  *)(0xF0A00000 - 0x80800000 + a))
#define as3310_readw_lcd(a)	(*(volatile unsigned short *)(0xF0A00000 - 0x80800000 + a))
#define as3310_readl_lcd(a)    (*(volatile unsigned int   *)(0xF0A00000 - 0x80800000 + a))

#define as3310_writeb_lcd(v,a)	(*(volatile unsigned char  *)(0xF0A00000 - 0x80800000 + a) = (v))
#define as3310_writew_lcd(v,a)	(*(volatile unsigned short *)(0xF0A00000 - 0x80800000 + a) = (v))
#define as3310_writel_lcd(v,a)	(*(volatile unsigned int   *)(0xF0A00000 - 0x80800000 + a) = (v))



#define as3310_readb_gpio(a)	(*(volatile unsigned char  *)(0xF0800000 - 0x50000000 + a))
#define as3310_readw_gpio(a)	(*(volatile unsigned short *)(0xF0800000 - 0x50000000 + a))
#define as3310_readl_gpio(a)    (*(volatile unsigned int   *)(0xF0800000 - 0x50000000 + a))

#define as3310_writeb_gpio(v,a)	(*(volatile unsigned char  *)(0xF0800000 - 0x50000000 + a) = (v))
#define as3310_writew_gpio(v,a)	(*(volatile unsigned short *)(0xF0800000 - 0x50000000 + a) = (v))
#define as3310_writel_gpio(v,a)	(*(volatile unsigned int   *)(0xF0800000 - 0x50000000 + a) = (v))

/* bit field operation functions */
/* use set(+4) and clear(+8) reg feature */
#define as3310_set_bit(addr,n)     as3310_writel((0x00000001 << (n)),(addr) + 4)
#define as3310_clear_bit(addr,n)   as3310_writel((0x00000001 << (n)),(addr) + 8)

/* operate with concrete regs */
#define as3310_set_u8reg(reg, i)      \
    do { as3310_readb(reg) |= (1 << (i)); } while(0)

#define as3310_set_u16reg(reg, i)     \
    do { as3310_readw(reg) |= (1 << (i)); } while(0)

#define as3310_set_u32reg(reg, i)       \
    do { as3310_readl(reg) |= (1 << (i)); } while(0)

#define as3310_clr_u8reg(reg, i)      \
    do { as3310_readb(reg) &= (~(1 << (i))); } while(0) 

#define as3310_clr_u16reg(reg, i)     \
    do { as3310_readw(reg) &= (~(1 << (i))); } while(0) 

#define as3310_clr_u32reg(reg, i)       \
    do { as3310_readl(reg) &= (~(1<<(i))); } while(0) 


/* check if the required bit is already set or cleared */
#define check_u8reg_assert(reg,i)  ( (as3310_readb(reg) & (1 << (i))) != 0 )
#define check_u16reg_assert(reg,i) ( (as3310_readw(reg) & (1 << (i))) != 0 )
#define check_u32reg_assert(reg,i) ( (as3310_readl(reg) & (1 << (i))) != 0 )
                                                        
#define check_u8reg_deassert(reg,i)  ( (as3310_readb(reg) & (1 << (i))) == 0 )
#define check_u16reg_deassert(reg,i) ( (as3310_readw(reg) & (1 << (i))) == 0 )
#define check_u32reg_deassert(reg,i) ( (as3310_readl(reg) & (1 << (i))) == 0 )


/* 16 bit uses LDRH/STRH, base +/- offset_8 */
typedef struct { volatile u16 offset[256]; } __regbase16;
#define __REGV16(vaddr)		((__regbase16 *)((vaddr)&~0xff)) \
					->offset[((vaddr)&0xff)>>1]
#define __REG16(paddr)          __REGV16(io_p2v(paddr))

/* 8/32 bit uses LDR/STR, base +/- offset_12 */
typedef struct { volatile u8 offset[4096]; } __regbase8;
#define __REGV8(vaddr)		((__regbase8  *)((vaddr)&~4095)) \
					->offset[((vaddr)&4095)>>0]
#define __REG8(paddr)		__REGV8(io_p2v(paddr))

typedef struct { volatile u32 offset[4096]; } __regbase32;
#define __REGV32(vaddr)		((__regbase32 *)((vaddr)&~4095)) \
					->offset[((vaddr)&4095)>>2]
#define __REG32(paddr)		__REGV32(io_p2v(paddr))

#else

#define __REG8(paddr)		io_p2v(paddr)
#define __REG16(paddr)		io_p2v(paddr)
#define __REG32(paddr)		io_p2v(paddr)

#endif

/* there are 4*32bit registers for one internal register */
#define REG_VAL 0
#define REG_SET 1
#define REG_CLR 2
#define REG_TOG 3

#endif
