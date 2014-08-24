#ifndef __XPT2046_TS_H__
#define __XPT2046_TS_H__

/*due to the usage of asm9260 spi controller's DMA functon, we check the CACHE-LINE*/
#define CACHE_LINE_BYTES 32


struct xpt2046_hw_info {
    int SPI_bus_num;
    int SPI_clk; //hz
    int penIRQport;
    int penIRQpin;
    int sample_ratio_HZ;
};

#endif

