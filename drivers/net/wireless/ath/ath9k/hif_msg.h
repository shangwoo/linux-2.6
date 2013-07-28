/*
 * Copyright (c) 2013 Oleksij Rempel <linux@rempel-privat.de>
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * HIF framre header related information. Should be keeped in sync
 * with firmware.
 */

#ifndef __HIF_MSG_H__
#define __HIF_MSG_H__

#define HIF_ENDPOINT 0xFF
#define EP4_BUG_SIZE (64 * 2)		/* to trigger EP4 bug, we need buffer
					 * bigger then 64 Bytes (size of
					 * EP4 fifo) */

enum hif_cmd_id {
	REBOOT = 0,
	COLD_REBOOT,			/* completly reinit chip */
	TRANSFER_SIZE_TEST		/* Test if usb EP can handle
					 * big packets. */
};

struct hif_msg_hdr {
	u8 endpoint_id;			/* to keep it compatible with
					 * HTC commands. We will use here 
					 * unused HTC ENDPOINT 0xFF */

	u8 cmd_id;			/* HIF specific command number */

	u16 length;			/* Length of payload. Needed only
					 * for transfer size test. */
} __packed;

#endif /* __HIF_MSG_H__ */
