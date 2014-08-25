

#define HW_IR           0x0000 /* RW. Interrupt */
#define BM_IR_CR0	BIT(4)
#define BM_IR_MR3	BIT(3)
#define BM_IR_MR2	BIT(2)
#define BM_IR_MR1	BIT(1)
#define BM_IR_MR0	BIT(0)

#define HW_TCR		0x0010 /* RW. Timer controller */
/* BM_C*_RST
 * Timer Counter and the Prescale Counter are synchronously reset on the
 * next positive edge of PCLK. The counters remain reset until TCR[1] is
 * returned to zero. */
#define BM_C3_RST	BIT(7)
#define BM_C2_RST	BIT(6)
#define BM_C1_RST	BIT(5)
#define BM_C0_RST	BIT(4)
/* BM_C*_EN
 * 1 - Timer Counter and Prescale Counter are enabled for counting
 * 0 - counters are disabled */
#define BM_C3_EN	BIT(3)
#define BM_C2_EN	BIT(2)
#define BM_C1_EN	BIT(1)
#define BM_C0_EN	BIT(0)

#define HW_DIR		0x0020 /* RW. Direction? */
/* 00 - count up
 * 01 - count down 
 * 10 - ?? 2^n/2 */
#define BM_DIR0_SHIFT	0
#define BM_DIR1_SHIFT	4
#define BM_DIR2_SHIFT	8
#define BM_DIR3_SHIFT	12

#define HW_TC0		0x0030 /* RO. Timer counter 0 */
/* HW_TC*. Timer counter owerflow (0xffff.ffff to 0x0000.0000) do not generate
 * interrupt. This registers can be used to detect overflow */
#define HW_TC1          0x0040
#define HW_TC2		0x0050
#define HW_TC3		0x0060

#define HW_PR		0x0070 /* RW. prescaler */
#define HW_PC		0x0080 /* RO. Prescaler counter */
#define HW_MCR		0x0090 /* RW. Match control */
#define HW_MR0		0x00a0 /* RW. Match reg */
#define HW_MR1		0x00b0
#define HW_MR2		0x00C0
#define HW_MR3		0x00D0
#define HW_CCR		0x00E0 /* RW. Capture control */
#define HW_CR0		0x00F0 /* RO. Capture reg */
#define HW_CR1               0x0100
#define HW_CR2               0x0110
#define HW_CR3               0x0120
#define HW_EMR               0x0130 /* RW. External Match */
#define HW_PWMTH0            0x0140 /* RW. PWM width */
#define HW_PWMTH1            0x0150
#define HW_PWMTH2            0x0160
#define HW_PWMTH3            0x0170
#define HW_CTCR              0x0180 /* Counter control */
#define HW_PWMC              0x0190 /* PWM control */

