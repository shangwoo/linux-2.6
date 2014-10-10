
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>

/* pinctrl register */
#define IOCON_PIO0_0			0x0000
/* only two modes are supported NONE and PULL UP */
#define IOCON_MODE_MASK			(0x3 << 3)
#define IOCON_MODE_PULL_UP		(0x2 << 3)
#define IOCON_MODE_PULL_NONE		(0x0 << 3)
/* up to 8 functions per pin */
#define IOCON_PINMUX_MASK		(0x7 << 0)

#define ASM9260_PINCTRL_PIN(pin)	PINCTRL_PIN(pin, #pin)
#define PINID(bank, pin)		((bank) * 32 + (pin) * 4)

/*
 * all pinctrl register offsets are based on GPIO names. So we will
 * use GPIOs for pins to be close to documentation.
 */
/*      PIN Name;   PinID = Reg Offset; Pin number */
enum asm9260_pin_enum {
	GPIO0_0		= PINID(0, 0), /* 120 */
	GPIO0_1		= PINID(0, 1), /* 121 */
	GPIO0_2		= PINID(0, 2), /* 122 */
	GPIO0_3		= PINID(0, 3), /* 123 */
	GPIO0_4		= PINID(0, 4), /* 124 */

	GPIO1_4		= PINID(1, 4), /* 128 */
	GPIO1_5		= PINID(1, 5), /* 129 */
	GPIO1_6		= PINID(1, 6), /* 130 */
	GPIO1_7		= PINID(1, 7), /* 131 */

	GPIO2_0		= PINID(2, 0), /* 132 */
	GPIO2_1		= PINID(2, 1), /* 133 */
	GPIO2_2		= PINID(2, 2), /* 134 */
	GPIO2_3		= PINID(2, 3), /* 135 */
	GPIO2_4		= PINID(2, 4), /* 136 */
	GPIO2_5		= PINID(2, 5), /* 137 */
	GPIO2_6		= PINID(2, 6), /* 138 */
	GPIO2_7		= PINID(2, 7), /* 139 */

	GPIO3_0		= PINID(3, 0), /* 140 */
	GPIO3_1		= PINID(3, 1), /* 141 */
	GPIO3_2		= PINID(3, 2), /* 142 */
	GPIO3_3		= PINID(3, 3), /* 143 */
	GPIO3_4		= PINID(3, 4), /* 144 */
	GPIO3_5		= PINID(3, 5), /* 145 */
	GPIO3_6		= PINID(3, 6), /* 146 */
	GPIO3_7		= PINID(3, 7), /* 147 */

	GPIO4_0		= PINID(4, 0), /* 151 */
	GPIO4_1		= PINID(4, 1), /* 152 */
	GPIO4_2		= PINID(4, 2), /* 153 */
	GPIO4_3		= PINID(4, 3), /* 154 */
	GPIO4_4		= PINID(4, 4), /* 155 */
	GPIO4_5		= PINID(4, 5), /* 156 */
	GPIO4_6		= PINID(4, 6), /* 157 */
	GPIO4_7		= PINID(4, 7), /* 158 */

	GPIO5_0		= PINID(5, 0), /* 169 */
	GPIO5_1		= PINID(5, 1), /* 170 */
	GPIO5_2		= PINID(5, 2), /* 171 */
	GPIO5_3		= PINID(5, 3), /* 172 */
	GPIO5_4		= PINID(5, 4), /* 173 */

	GPIO8_1		= PINID(8, 1), /* 51 */
	GPIO8_2		= PINID(8, 2), /* 52 */
	GPIO8_3		= PINID(8, 3), /* 53 */
	GPIO8_4		= PINID(8, 4), /* 54 */
	GPIO8_5		= PINID(8, 5), /* 55 */
	GPIO8_6		= PINID(8, 6), /* 56 */
	GPIO8_7		= PINID(8, 7), /* 57 */

	GPIO9_0		= PINID(9, 0), /* 45 */
	GPIO9_1		= PINID(9, 1), /* 46 */
	GPIO9_2		= PINID(9, 2), /* 47 */
	GPIO9_3		= PINID(9, 3), /* 48 */
	GPIO9_4		= PINID(9, 4), /* 49 */
	GPIO9_5		= PINID(9, 5), /* 50 */

	GPIO10_0	= PINID(10, 0), /* 4 */
	GPIO10_1	= PINID(10, 1), /* 5 */
	GPIO10_2	= PINID(10, 2), /* 6 */
	GPIO10_3	= PINID(10, 3), /* 7 */
	GPIO10_4	= PINID(10, 4), /* 8 */
	GPIO10_5	= PINID(10, 5), /* 9 */
	GPIO10_6	= PINID(10, 6), /* 10 */
	GPIO10_7	= PINID(10, 7), /* 11 */

	GPIO11_0	= PINID(11, 0), /* 12 */
	GPIO11_1	= PINID(11, 1), /* 13 */
	GPIO11_2	= PINID(11, 2), /* 14 */
	GPIO11_3	= PINID(11, 3), /* 15 */
	GPIO11_4	= PINID(11, 4), /* 16 */
	GPIO11_5	= PINID(11, 5), /* 17 */
	GPIO11_6	= PINID(11, 6), /* 18 */
	GPIO11_7	= PINID(11, 7), /* 19 */

	GPIO12_0	= PINID(12, 0), /* 23 */
	GPIO12_1	= PINID(12, 1), /* 24 */
	GPIO12_2	= PINID(12, 2), /* 25 */
	GPIO12_3	= PINID(12, 3), /* 26 */
	GPIO12_4	= PINID(12, 4), /* 27 */
	GPIO12_5	= PINID(12, 5), /* 28 */
	GPIO12_6	= PINID(12, 6), /* 29 */
	GPIO12_7	= PINID(12, 7), /* 30 */

	GPIO13_4	= PINID(13, 4), /* 31 */
	GPIO13_5	= PINID(13, 5), /* 32 */
	GPIO13_6	= PINID(13, 6), /* 33 */
	GPIO13_7	= PINID(13, 7), /* 34 */

	GPIO14_0	= PINID(14, 0), /* 38 */
	GPIO14_1	= PINID(14, 1), /* 39 */
	GPIO14_2	= PINID(14, 2), /* 40 */
	GPIO14_3	= PINID(14, 3), /* 41 */
	GPIO14_4	= PINID(14, 4), /* 42 */
	GPIO14_5	= PINID(14, 5), /* 43 */

	GPIO15_0	= PINID(15, 0), /* 44 */
	GPIO15_1	= PINID(15, 1), /* 61 */
	GPIO15_2	= PINID(15, 2), /* 62 */
	GPIO15_3	= PINID(15, 3), /* 63 */
	GPIO15_4	= PINID(15, 4), /* 64 */
	GPIO15_5	= PINID(15, 5), /* 65 */
	GPIO15_6	= PINID(15, 6), /* 66 */
	GPIO15_7	= PINID(15, 7), /* 67 */

	GPIO16_0	= PINID(16, 0), /* 73 */
	GPIO16_1	= PINID(16, 1), /* 74 */
	GPIO16_2	= PINID(16, 2), /* 75 */
	GPIO16_3	= PINID(16, 3), /* 76 */
	GPIO16_4	= PINID(16, 4), /* 77 */
	GPIO16_5	= PINID(16, 5), /* 78 */
	GPIO16_6	= PINID(16, 6), /* 79 */
	GPIO16_7	= PINID(16, 7), /* 80 */

	GPIO17_0	= PINID(17, 0), /* 81 */
	GPIO17_1	= PINID(17, 1), /* 82 */
	GPIO17_2	= PINID(17, 2), /* 83 */
	GPIO17_3	= PINID(17, 3), /* 84 */
	GPIO17_4	= PINID(17, 4), /* 85 */
	GPIO17_5	= PINID(17, 5), /* 86 */
	GPIO17_6	= PINID(17, 6), /* 87 */
	GPIO17_7	= PINID(17, 7), /* 88 */
}
