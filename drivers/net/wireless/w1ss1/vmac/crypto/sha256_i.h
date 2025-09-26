/*
 * SHA-256 internal definitions
 * Copyright (c) 2003-2011, Jouni Malinen <j@w1.fi>
 *
 * This software may be distributed under the terms of the BSD license.
 * See README for more details.
 */

#ifndef SHA256_I_H
#define SHA256_I_H

#define SHA256_BLOCK_SIZE 64

struct sha256_state_i {
	u64 length;
	u32 state[8], curlen;
	u8 buf[SHA256_BLOCK_SIZE];
};

void sha256_init_i(struct sha256_state_i *md);
int sha256_process_i(struct sha256_state_i *md, const unsigned char *in,
		   unsigned long inlen);
int sha256_done_i(struct sha256_state_i *md, unsigned char *out);

#endif /* SHA256_I_H */
