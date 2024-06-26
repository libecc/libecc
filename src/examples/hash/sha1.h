/*
 *  Copyright (C) 2021 - This file is part of libecc project
 *
 *  Authors:
 *      Ryad BENADJILA <ryadbenadjila@gmail.com>
 *      Arnaud EBALARD <arnaud.ebalard@ssi.gouv.fr>
 *
 *  This software is licensed under a dual BSD and GPL v2 license.
 *  See LICENSE file at the root folder of the project.
 */
#ifndef __SHA1_H__
#define __SHA1_H__

/* Include libec for useful types and macros */
#include <libecc/libec.h>

/****************************************************/
/*
 * 32-bit integer manipulation macros
 */
#ifndef GET_UINT32_BE
#define GET_UINT32_BE(n, b, i)			  	\
do {						    	\
	(n) =     ( ((u32) (b)[(i)    ]) << 24 )   	\
		| ( ((u32) (b)[(i) + 1]) << 16 )	\
		| ( ((u32) (b)[(i) + 2]) <<  8 )	\
		| ( ((u32) (b)[(i) + 3])       );       \
} while( 0 )
#endif
#ifndef GET_UINT32_LE
#define GET_UINT32_LE(n, b, i)			  	\
do {						    	\
	(n) =     ( ((u32) (b)[(i) + 3]) << 24 )   	\
		| ( ((u32) (b)[(i) + 2]) << 16 )	\
		| ( ((u32) (b)[(i) + 1]) <<  8 )	\
		| ( ((u32) (b)[(i)    ])       );       \
} while( 0 )
#endif


#ifndef PUT_UINT32_BE
#define PUT_UINT32_BE(n, b, i)		  	\
do {					    	\
	(b)[(i)    ] = (u8) ( (n) >> 24 );      \
	(b)[(i) + 1] = (u8) ( (n) >> 16 );      \
	(b)[(i) + 2] = (u8) ( (n) >>  8 );      \
	(b)[(i) + 3] = (u8) ( (n)       );      \
} while( 0 )
#endif

#ifndef PUT_UINT32_LE
#define PUT_UINT32_LE(n, b, i)		  	\
do {					    	\
	(b)[(i) + 3] = (u8) ( (n) >> 24 );      \
	(b)[(i) + 2] = (u8) ( (n) >> 16 );      \
	(b)[(i) + 1] = (u8) ( (n) >>  8 );      \
	(b)[(i)    ] = (u8) ( (n)       );      \
} while( 0 )
#endif

/*
 * 64-bit integer manipulation macros
 */
#ifndef PUT_UINT64_BE
#define PUT_UINT64_BE(n,b,i)            \
do {                                    \
    (b)[(i)    ] = (u8) ( (n) >> 56 );  \
    (b)[(i) + 1] = (u8) ( (n) >> 48 );  \
    (b)[(i) + 2] = (u8) ( (n) >> 40 );  \
    (b)[(i) + 3] = (u8) ( (n) >> 32 );  \
    (b)[(i) + 4] = (u8) ( (n) >> 24 );  \
    (b)[(i) + 5] = (u8) ( (n) >> 16 );  \
    (b)[(i) + 6] = (u8) ( (n) >>  8 );  \
    (b)[(i) + 7] = (u8) ( (n)       );  \
} while( 0 )
#endif /* PUT_UINT64_BE */

#ifndef PUT_UINT64_LE
#define PUT_UINT64_LE(n,b,i)            \
do {                                    \
    (b)[(i) + 7] = (u8) ( (n) >> 56 );  \
    (b)[(i) + 6] = (u8) ( (n) >> 48 );  \
    (b)[(i) + 5] = (u8) ( (n) >> 40 );  \
    (b)[(i) + 4] = (u8) ( (n) >> 32 );  \
    (b)[(i) + 3] = (u8) ( (n) >> 24 );  \
    (b)[(i) + 2] = (u8) ( (n) >> 16 );  \
    (b)[(i) + 1] = (u8) ( (n) >>  8 );  \
    (b)[(i)    ] = (u8) ( (n)       );  \
} while( 0 )
#endif /* PUT_UINT64_LE */

#define SHA1_STATE_SIZE   5
#define SHA1_BLOCK_SIZE   64
#define SHA1_DIGEST_SIZE  20
#define SHA1_DIGEST_SIZE_BITS  160

#define SHA1_HASH_MAGIC ((word_t)(0x1120387132308110ULL))
#define SHA1_HASH_CHECK_INITIALIZED(A, ret, err) \
	MUST_HAVE((((void *)(A)) != NULL) && ((A)->magic == SHA1_HASH_MAGIC), ret, err)

typedef struct {
	/* Number of bytes processed */
	u64 sha1_total;
	/* Internal state */
	u32 sha1_state[SHA1_STATE_SIZE];
	/* Internal buffer to handle updates in a block */
	u8 sha1_buffer[SHA1_BLOCK_SIZE];
	/* Initialization magic value */
	word_t magic;
} sha1_context;

/* Init hash function. Returns 0 on success, -1 on error. */
ATTRIBUTE_WARN_UNUSED_RET int sha1_init(sha1_context *ctx);

ATTRIBUTE_WARN_UNUSED_RET int sha1_update(sha1_context *ctx, const u8 *input, u32 ilen);

/* Finalize. Returns 0 on success, -1 on error.*/
ATTRIBUTE_WARN_UNUSED_RET int sha1_final(sha1_context *ctx, u8 output[SHA1_DIGEST_SIZE]);

/*
 * Scattered version performing init/update/finalize on a vector of buffers
 * 'inputs' with the length of each buffer passed via 'ilens'. The function
 * loops on pointers in 'inputs' until it finds a NULL pointer. The function
 * returns 0 on success, -1 on error.
 */
ATTRIBUTE_WARN_UNUSED_RET int sha1_scattered(const u8 **inputs, const u32 *ilens,
		      u8 output[SHA1_DIGEST_SIZE]);

/*
 * Single call version performing init/update/final on given input.
 * Returns 0 on success, -1 on error.
 */
ATTRIBUTE_WARN_UNUSED_RET int sha1(const u8 *input, u32 ilen, u8 output[SHA1_DIGEST_SIZE]);

#endif /* __SHA1_H__ */
