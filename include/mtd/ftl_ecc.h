/*
 * YAFFS: Yet another Flash File System . A NAND-flash specific file system. 
 *
 * Copyright (C) 2002-2007 Aleph One Ltd.
 *   for Toby Churchill Ltd and Brightstar Engineering
 *
 * Created by Charles Manning <charles@aleph1.co.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1 as
 * published by the Free Software Foundation.
 *
 * Note: Only YAFFS headers are LGPL, YAFFS C code is covered by GPL.
 */

 /*
  * This code implements the ECC algorithm used in SmartMedia.
  *
  * The ECC comprises 22 bits of parity information and is stuffed into 3 bytes. 
  * The two unused bit are set to 1.
  * The ECC can correct single bit errors in a 256-byte page of data. Thus, two such ECC 
  * blocks are used on a 512-byte NAND page.
  *
  */

#ifndef __FTL_ECC_H__
#define __FTL_ECC_H__

typedef struct ftl_ecc_data {
	unsigned char colParity;
	unsigned char lineParity;
	unsigned char lineParityPrime;
} __attribute__((packed)) ftl_ECC ;

void ftl_ECCCalculate(const unsigned char *data, unsigned nBytes,
			     ftl_ECC * ecc);
int ftl_ECCCorrect(unsigned char *data, unsigned nBytes,
			  ftl_ECC * read_ecc,
			  const ftl_ECC * test_ecc);
#endif

