
/*
 *  linux/include/asm-arm/arch-omap/serial.h
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_SERIAL_H
#define __ASM_ARCH_SERIAL_H

#include <asm/arch/hardware.h>
#include <asm/io.h>

extern int TTEESSTT;
            
void putc(int c)
{
    unsigned long status;
    while(((status=inl(HW_UARTAPP_SATA))&0x08000000)==0);
    outl(c,HW_UARTAPP_DATA);
    if (c=='\n') {
        putc('\r');
    }
}

void flush(void)
{
    do
    {
    }while(0);
}
#endif
