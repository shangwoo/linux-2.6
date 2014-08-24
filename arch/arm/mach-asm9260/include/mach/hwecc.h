/*

linux/arch/arm/mach-as3310/hwecc.h
 
from Alpha Scale AS3310X IronPalms Console
He Yong, AlpScale Software Engineering, heyong@alpscale.com

------------------- Version 1.0  ----------------------
Create File
 He Yong 2007-03-11

*/


/*
Alpha Scale AS3310X Booter, H-BOOT
Zhao Haiyuan, AlpScale Software Engineering, zhaoy@alpscale.com.cn
 
------------------- Version 1.0  ----------------------
Create File, 
    Support Nand ECC 
    Zhao Haiyuan 2007-04-29

*/


#ifndef __HWECC_H__
#define __HWECC_H__


#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

//#include <asm/hardware.h>

#define AS3310_HWECC_TIME_OUT 0x1000000

/*  register addrs */
#define AS3310_HWECC_CTRL       0x80008000

/* define error codes */
#define AS3310_ECC_UNCORRECTABLE   -1             
#define AS3310_ECC_ALLZEROES       -2              
#define AS3310_ECC_NOERROR         0              
#define AS3310_ECC_CORRECTED       1              

/* define status codes */
#define AS3310_HWECC_OK         1
#define AS3310_HWECC_NOTREADY   0
#define AS3310_HWECC_CHERROR    (-1)


struct as3310_hwecc_info {
    struct completion * cmplt;  /* complete signal */
};

struct as3310_hwecc_encode_info {
    int status;
    dma_addr_t data;            /* physical address of data for encode */
    dma_addr_t parity_paddr;    /* physical address of parity data to store at */
    dma_addr_t error_code;
    struct as3310_dma_chain * encode_chain;   /* encode dma chain */
};


struct as3310_hwecc_decode_info {
    int status;
    dma_addr_t data;            /* physical address of data for encode */
    dma_addr_t parity_paddr;    /* physical address of parity data to store at */
    dma_addr_t report;          /* physical address of 40 bytes report */ 
    struct as3310_dma_chain * decode_chain;   /* decode dma chain */
};

 /*
request_as3310_hwecc_encode
inputs:
struct device * dev the device which use this hw ecc
dma_addr_t data,      data physical address
dma_addr_t parity,    parity data physical address to store at
*/
struct as3310_hwecc_encode_info * request_as3310_hwecc_encode(struct device * dev);
 /*
request_as3310_hwecc_decode
inputs:
struct device * dev the device which use this hw ecc
dma_addr_t data,      data physical address
dma_addr_t parity,    parity data physical address to fetch at
*/
struct as3310_hwecc_decode_info * request_as3310_hwecc_decode(struct device * dev);

/*
as3310_hwecc_encode. 
Encode the data from the address "data" with the byte length of "Ecclth" eg 512.
Write the status code to "error_code" and write the parity code to "parity_paddr" 
*/
int as3310_hwecc_encode(struct as3310_hwecc_encode_info * info);

/*
as3310_hwecc_decode. 
decode the data from the address "data" 
with the byte length of "Ecclth" eg 512.
based on the party code from parityadd
write the EccReport to "Report" 9*32 bit

*/
int as3310_hwecc_decode(struct as3310_hwecc_decode_info * info);
/*
Ecc correct data. Based on the EccReport from "Report" Check the data from "datafrom". Check the parity code from "parityadd". 
*/
int as3310_hwecc_correct_error(char * datafrom,char * parityadd,int * Report);

#endif //__HWECC_H__
