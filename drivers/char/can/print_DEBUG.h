#ifndef PRINT_DEBUG_H
#define PRINT_DEBUG_H

#undef PDEBUG
#ifdef TXX_VIEW_DEBUG
	#ifdef __KERNEL__
		#define PDEBUG(fmt, args...) printk(KERN_NOTICE fmt, ##args)
	#else
		#define PDEBUG(fmt, args...) fprintf(stderr, fmt ,##args)
	#endif
#else
	#define PDEBUG(fmt, args...)
#endif

#undef  PDEBUGG
#define PDEBUGG(fmt, args...)

#endif
/*
*********************************************************************************************************
**                            End Of File
*********************************************************************************************************
*/
