/*
 *  Copyright (C) 2023 - This file is part of libecc project
 *
 *  Authors:
 *      Ryad BENADJILA <ryadbenadjila@gmail.com>
 *	Karim KHALFALLAH <karim.khalfallah@ssi.gouv.fr>
 *
 *  This software is licensed under a dual BSD and GPL v2 license.
 *  See LICENSE file at the root folder of the project.
 */

#if defined(WITH_EC_HW_ACCELERATOR)

#include "hw_accelerator_glue.h"
/* The low level driver for the HW accelerator */
#include "hw_accelerator_driver.h"

/* Mutex locking/unlocking primitives for multi-threading */
#if defined(WITH_EC_HW_LOCKING)
#include <pthread.h>
static pthread_mutex_t _global_mutex;             
#endif

static inline void mutex_lock(void)
{
#if defined(WITH_EC_HW_LOCKING)
        pthread_mutex_lock(&_global_mutex);
	return;
#else
	return;
#endif
}
        
static inline void mutex_unlock(void)
{
#if defined(WITH_EC_HW_LOCKING)
	pthread_mutex_unlock(&_global_mutex);
	return;
#else
	return;
#endif
}

/* This is the "glue" for driving the hardware accelerated operations
 * for ECC and arithmetic operations.
 *
 */


static inline bitcnt_t nn_bit_len(nn_src_t a) {
	bitcnt_t len;

	if(nn_bitlen(a, &len)){
		return 0;
	}

	return len;
}

#define _NN_BYTE_CEIL(b) (((b) == 0) ? 1 : (((b) + 8 - 1) / 8))
/* 
 * Return the optimized byte length size removing
 * the prepending zero in constant time
 */
static inline u16 nn_byte_len(nn_src_t a) {
	/* For the specific case of zero, return
	 * at least one byte that will be zero padded.
	 */
	return (u16)_NN_BYTE_CEIL(nn_bit_len(a));
}

/*
 * FIXME: the following operations handling point at infinity
 * that are NOT constant time. This should be fixed.
 */
#define CHECK_INFINITY(in, idx) do {						\
	int iszero, isone;							\
	/* Check for infinity */						\
	ret = prj_pt_iszero((in), &iszero); EG(ret, err);			\
	if(iszero){								\
		/* Infinity point case: set in hardware the infinity flag */	\
		ret = hw_driver_point_zero((idx)); EG(ret, err);		\
	}									\
	else{									\
		/* Clear the infinity flag in hardware */			\
		ret = hw_driver_point_unzero((idx)); EG(ret, err);		\
		/* Check that projective points are in fact affine */		\
		ret = nn_isone(&((in)->Z.fp_val), &isone); EG(ret, err);	\
		MUST_HAVE(isone, ret, err);					\
	}									\
} while(0)

#define SET_INFINITY(out, idx) do {						\
	int iszero;								\
	/* If the result is point at infinity, set it */			\
	ret = hw_driver_point_iszero((idx), &iszero); EG(ret, err);		\
	if(iszero){								\
		ret = prj_pt_zero((out)); EG(ret, err);				\
	}									\
} while(0)

/*
 * FIXME: this is a somewhat cumbersome way of testing if the current curve is
 * set in hardware. We might be able to find a better way while caching the curve.
 */
#define SET_CURVE(current, set, c) do {								\
	int check;										\
	/* Set the curve if necessary */							\
	if((set) == 0){										\
		check = 0;									\
	}											\
	else{											\
		ret = are_equal((current), (c), sizeof(ec_shortw_crv), &check); EG(ret, err1);	\
	}											\
	if(!check){										\
		ret = hw_set_curve((c)); EG(ret, err1);						\
		(*current) = (*c);								\
		set = 1;									\
	}											\
err1:												\
	EG(ret, err);										\
} while(0)


/* Current curve */
static volatile u8 current_curve_set;
static ec_shortw_crv current_curve;

int hw_set_curve(ec_shortw_crv_src_t curve)
{
	int ret;

	u8 a[NN_MAX_BYTE_LEN], b[NN_MAX_BYTE_LEN], p[NN_MAX_BYTE_LEN], q[NN_MAX_BYTE_LEN];
	nn_src_t a_nn, b_nn, p_nn, q_nn;
	
	/* NOTE: no need to lock the mutex here as it should have been locked
	 * in the caller!
	 */
	ret = ec_shortw_crv_check_initialized(curve); EG(ret, err);

	/* Now serialize our big numbers to big endian for the hardware driver */
	a_nn = &(curve->a.fp_val);
	ret = nn_export_to_buf(a, nn_byte_len(a_nn), a_nn); EG(ret, err);
	b_nn = &(curve->b.fp_val);
	ret = nn_export_to_buf(b, nn_byte_len(b_nn), b_nn); EG(ret, err);
	p_nn = &(curve->a.ctx->p);
	ret = nn_export_to_buf(p, nn_byte_len(p_nn), p_nn); EG(ret, err);
	q_nn = &(curve->order);
	ret = nn_export_to_buf(q, nn_byte_len(q_nn), q_nn); EG(ret, err);

	ret = hw_driver_set_curve(a, nn_byte_len(a_nn), b, nn_byte_len(b_nn),
				  p, nn_byte_len(p_nn), q, nn_byte_len(q_nn));

err:
	return ret;
}

int hw_is_on_shortw_curve(fp_src_t X, fp_src_t Y, ec_shortw_crv_src_t curve, int *on_curve)
{
	int ret;
	u8 x[NN_MAX_BYTE_LEN], y[NN_MAX_BYTE_LEN];
	nn_src_t x_nn, y_nn;

	mutex_lock();

	MUST_HAVE((on_curve != NULL), ret, err);
	ret = fp_check_initialized(X);  EG(ret, err);
	ret = fp_check_initialized(Y);  EG(ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, curve);

	/* Now serialize our big numbers to big endian for the hardware driver */
	x_nn = &(X->fp_val);
	ret = nn_export_to_buf(x, nn_byte_len(x_nn), x_nn); EG(ret, err);
	y_nn = &(Y->fp_val);
	ret = nn_export_to_buf(y, nn_byte_len(y_nn), y_nn); EG(ret, err);

	ret = hw_driver_is_on_curve(x, nn_byte_len(x_nn), y, nn_byte_len(y_nn), on_curve);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_is_on_curve(prj_pt_src_t in, int *on_curve)
{
	int ret;
	u8 x[NN_MAX_BYTE_LEN], y[NN_MAX_BYTE_LEN];
	nn_src_t x_nn, y_nn;

	mutex_lock();

	MUST_HAVE((on_curve != NULL), ret, err);
	ret = prj_pt_check_initialized(in);  EG(ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in->crv);

	CHECK_INFINITY(in, 0);

	/* Now serialize our big numbers to big endian for the hardware driver */
	x_nn = &(in->X.fp_val);
	ret = nn_export_to_buf(x, nn_byte_len(x_nn), x_nn); EG(ret, err);
	y_nn = &(in->Y.fp_val);
	ret = nn_export_to_buf(y, nn_byte_len(y_nn), y_nn); EG(ret, err);

	ret = hw_driver_is_on_curve(x, nn_byte_len(x_nn), y, nn_byte_len(y_nn), on_curve);

err:
	mutex_unlock();
	return ret;
}

int hw_ec_shortw_aff_eq_or_opp(aff_pt_src_t in1, aff_pt_src_t in2, int *aff_is_eq_or_opp)
{
	int ret, eq, opp;
	u8 x1[NN_MAX_BYTE_LEN], y1[NN_MAX_BYTE_LEN];
	u8 x2[NN_MAX_BYTE_LEN], y2[NN_MAX_BYTE_LEN];
	nn_src_t x1_nn, y1_nn, x2_nn, y2_nn;

	mutex_lock();

	MUST_HAVE((aff_is_eq_or_opp != NULL), ret, err);
	ret = aff_pt_check_initialized(in1); EG(ret, err);
	ret = aff_pt_check_initialized(in2); EG(ret, err);	

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in1->crv);

	/* Now serialize our big numbers to big endian for the hardware driver */
	x1_nn = &(in1->x.fp_val);
	ret = nn_export_to_buf(x1, nn_byte_len(x1_nn), x1_nn); EG(ret, err);
	y1_nn = &(in1->y.fp_val);
	ret = nn_export_to_buf(y1, nn_byte_len(y1_nn), y1_nn); EG(ret, err);
	x2_nn = &(in2->x.fp_val);
	ret = nn_export_to_buf(x2, nn_byte_len(x2_nn), x2_nn); EG(ret, err);
	y2_nn = &(in2->y.fp_val);
	ret = nn_export_to_buf(y2, nn_byte_len(y2_nn), y2_nn); EG(ret, err);

	/* Check equality */
	ret = hw_driver_eq(x1, nn_byte_len(x1_nn), y1, nn_byte_len(y1_nn),
			   x2, nn_byte_len(x2_nn), y2, nn_byte_len(y2_nn), &eq); EG(ret, err);
	/* Check opposite */
	ret = hw_driver_opp(x1, nn_byte_len(x1_nn), y1, nn_byte_len(y1_nn),
			    x2, nn_byte_len(x2_nn), y2, nn_byte_len(y2_nn), &opp); EG(ret, err);

	(*aff_is_eq_or_opp) = (eq | opp);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_eq_or_opp(prj_pt_src_t in1, prj_pt_src_t in2, int *eq_or_opp)
{
	int ret, eq, opp;
	u8 x1[NN_MAX_BYTE_LEN], y1[NN_MAX_BYTE_LEN];
	u8 x2[NN_MAX_BYTE_LEN], y2[NN_MAX_BYTE_LEN];
	nn_src_t x1_nn, y1_nn, x2_nn, y2_nn;

	mutex_lock();

	MUST_HAVE((eq_or_opp != NULL), ret, err);
	ret = prj_pt_check_initialized(in1); EG(ret, err);
	ret = prj_pt_check_initialized(in2); EG(ret, err);	

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in1->crv);

	CHECK_INFINITY(in1, 0);
	CHECK_INFINITY(in2, 1);

	/* Now serialize our big numbers to big endian for the hardware driver */
	x1_nn = &(in1->X.fp_val);
	ret = nn_export_to_buf(x1, nn_byte_len(x1_nn), x1_nn); EG(ret, err);
	y1_nn = &(in1->Y.fp_val);
	ret = nn_export_to_buf(y1, nn_byte_len(y1_nn), y1_nn); EG(ret, err);
	x2_nn = &(in2->X.fp_val);
	ret = nn_export_to_buf(x2, nn_byte_len(x2_nn), x2_nn); EG(ret, err);
	y2_nn = &(in2->Y.fp_val);
	ret = nn_export_to_buf(y2, nn_byte_len(y2_nn), y2_nn); EG(ret, err);

	/* Check equality */
	ret = hw_driver_eq(x1, nn_byte_len(x1_nn), y1, nn_byte_len(y1_nn),
			   x2, nn_byte_len(x2_nn), y2, nn_byte_len(y2_nn), &eq); EG(ret, err);
	/* Check opposite */
	ret = hw_driver_opp(x1, nn_byte_len(x1_nn), y1, nn_byte_len(y1_nn),
			    x2, nn_byte_len(x2_nn), y2, nn_byte_len(y2_nn), &opp); EG(ret, err);

	(*eq_or_opp) = (eq | opp);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_neg(prj_pt_t out, prj_pt_src_t in)
{
	int ret;
	u8 x[NN_MAX_BYTE_LEN], y[NN_MAX_BYTE_LEN];
	nn_src_t x_nn, y_nn;
	u8 out_x[NN_MAX_BYTE_LEN], out_y[NN_MAX_BYTE_LEN];
	unsigned int out_x_sz, out_y_sz;

	mutex_lock();

	MUST_HAVE((out != NULL), ret, err);
	ret = prj_pt_check_initialized(in); EG(ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in->crv);

	CHECK_INFINITY(in, 0);

	/* Initialize our output point if necessary */
	if(out != in){
		ret = prj_pt_init(out, in->crv); EG(ret, err);
		ret = fp_one(&(out->Z)); EG(ret, err);
	}

	/* Now serialize our big numbers to big endian for the hardware driver */
	x_nn = &(in->X.fp_val);
	ret = nn_export_to_buf(x, nn_byte_len(x_nn), x_nn); EG(ret, err);
	y_nn = &(in->Y.fp_val);
	ret = nn_export_to_buf(y, nn_byte_len(y_nn), y_nn); EG(ret, err);

	/* Perform the operation */
	out_x_sz = sizeof(out_x);
	out_y_sz = sizeof(out_y);
	ret = hw_driver_neg(x, nn_byte_len(x_nn), y, nn_byte_len(y_nn),
			    out_x, &out_x_sz, out_y, &out_y_sz); EG(ret, err);

	/* Import to the output */
	MUST_HAVE((out_x_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->X.fp_val), out_x, (u16)out_x_sz); EG(ret, err);
	MUST_HAVE((out_y_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->Y.fp_val), out_y, (u16)out_y_sz); EG(ret, err);

	SET_INFINITY(out, 1);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_dbl(prj_pt_t out, prj_pt_src_t in)
{
	int ret;
	u8 x[NN_MAX_BYTE_LEN], y[NN_MAX_BYTE_LEN];
	nn_src_t x_nn, y_nn;
	u8 out_x[NN_MAX_BYTE_LEN], out_y[NN_MAX_BYTE_LEN];
	unsigned int out_x_sz, out_y_sz;

	mutex_lock();

	MUST_HAVE((out != NULL), ret, err);
	ret = prj_pt_check_initialized(in); EG(ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in->crv);

	CHECK_INFINITY(in, 0);

	/* Initialize our output point if necessary */
	if(out != in){
		ret = prj_pt_init(out, in->crv); EG(ret, err);
		ret = fp_one(&(out->Z)); EG(ret, err);
	}

	/* Now serialize our big numbers to big endian for the hardware driver */
	x_nn = &(in->X.fp_val);
	ret = nn_export_to_buf(x, nn_byte_len(x_nn), x_nn); EG(ret, err);
	y_nn = &(in->Y.fp_val);
	ret = nn_export_to_buf(y, nn_byte_len(y_nn), y_nn); EG(ret, err);

	/* Perform the operation */
	out_x_sz = sizeof(out_x);
	out_y_sz = sizeof(out_y);
	ret = hw_driver_dbl(x, nn_byte_len(x_nn), y, nn_byte_len(y_nn),
			    out_x, &out_x_sz, out_y, &out_y_sz); EG(ret, err);

	/* Import to the output */
	MUST_HAVE((out_x_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->X.fp_val), out_x, (u16)out_x_sz); EG(ret, err);
	MUST_HAVE((out_y_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->Y.fp_val), out_y, (u16)out_y_sz); EG(ret, err);

	SET_INFINITY(out, 1);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_add(prj_pt_t out, prj_pt_src_t in1, prj_pt_src_t in2)
{
	int ret;
	u8 x1[NN_MAX_BYTE_LEN], y1[NN_MAX_BYTE_LEN];
	u8 x2[NN_MAX_BYTE_LEN], y2[NN_MAX_BYTE_LEN];
	nn_src_t x1_nn, y1_nn, x2_nn, y2_nn;
	u8 out_x[NN_MAX_BYTE_LEN], out_y[NN_MAX_BYTE_LEN];
	unsigned int out_x_sz, out_y_sz;

	mutex_lock();

	MUST_HAVE((out != NULL), ret, err);
	ret = prj_pt_check_initialized(in1); EG(ret, err);
	ret = prj_pt_check_initialized(in2); EG(ret, err);

	/* Check consistency */
	MUST_HAVE((in1->crv == in2->crv), ret, err);
	MUST_HAVE((in1->X.ctx == in1->Y.ctx), ret, err);
	MUST_HAVE((in2->X.ctx == in2->Y.ctx), ret, err);
	MUST_HAVE((in1->X.ctx == in2->X.ctx), ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in1->crv);

	CHECK_INFINITY(in1, 0);
	CHECK_INFINITY(in2, 1);

	/* Initialize our output point if necessary */
	if((out != in1) && (out != in2)){
		ret = prj_pt_init(out, in1->crv); EG(ret, err);
		ret = fp_one(&(out->Z)); EG(ret, err);
	}

	/* Now serialize our big numbers to big endian for the hardware driver */
	x1_nn = &(in1->X.fp_val);
	ret = nn_export_to_buf(x1, nn_byte_len(x1_nn), x1_nn); EG(ret, err);
	y1_nn = &(in1->Y.fp_val);
	ret = nn_export_to_buf(y1, nn_byte_len(y1_nn), y1_nn); EG(ret, err);
	x2_nn = &(in2->X.fp_val);
	ret = nn_export_to_buf(x2, nn_byte_len(x2_nn), x2_nn); EG(ret, err);
	y2_nn = &(in2->Y.fp_val);
	ret = nn_export_to_buf(y2, nn_byte_len(y2_nn), y2_nn); EG(ret, err);

	/* Perform the operation */
	out_x_sz = sizeof(out_x);
	out_y_sz = sizeof(out_y);
	ret = hw_driver_add(x1, nn_byte_len(x1_nn), y1, nn_byte_len(y1_nn),
			    x2, nn_byte_len(x2_nn), y2, nn_byte_len(y2_nn),
			    out_x, &out_x_sz, out_y, &out_y_sz); EG(ret, err);

	/* Import to the output */
	MUST_HAVE((out_x_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->X.fp_val), out_x, (u16)out_x_sz); EG(ret, err);
	MUST_HAVE((out_y_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->Y.fp_val), out_y, (u16)out_y_sz); EG(ret, err);

	SET_INFINITY(out, 1);

err:
	mutex_unlock();
	return ret;
}

/* Internal function without multi-threading locking */
static int _hw_prj_pt_mul_ltr(prj_pt_t out, nn_src_t m, prj_pt_src_t in)
{
	int ret;
	u8 x[NN_MAX_BYTE_LEN], y[NN_MAX_BYTE_LEN];
	nn_src_t x_nn, y_nn;
	u8 scalar[NN_MAX_BYTE_LEN];
	u8 out_x[NN_MAX_BYTE_LEN], out_y[NN_MAX_BYTE_LEN];
	unsigned int out_x_sz, out_y_sz;

	MUST_HAVE((out != NULL), ret, err);
	ret = prj_pt_check_initialized(in); EG(ret, err);
	ret = nn_check_initialized(m); EG(ret, err);

	/* Set the curve if necessary */
	SET_CURVE(&current_curve, current_curve_set, in->crv);

	/* NOTE: for the specific case of scalar multiplication, the input
	 * point is at index 1 in the underlying hardware.
	 */
	CHECK_INFINITY(in, 1);

	/* Initialize our output point if necessary */
	if(in != out){
		ret = prj_pt_init(out, in->crv); EG(ret, err);
		ret = fp_one(&(out->Z)); EG(ret, err);
	}

	/* Now serialize our big numbers to big endian for the hardware driver */
	x_nn = &(in->X.fp_val);
	ret = nn_export_to_buf(x, nn_byte_len(x_nn), x_nn); EG(ret, err);
	y_nn = &(in->Y.fp_val);
	ret = nn_export_to_buf(y, nn_byte_len(y_nn), y_nn); EG(ret, err);
	/* NOTE: we do not want to leak the scalar length here, so we
	 * send our maximum size of curve order.
	 */
	ret = nn_export_to_buf(scalar, nn_byte_len(&(in->crv->order)), m); EG(ret, err);

	/* Perform the operation */
	out_x_sz = sizeof(out_x);
	out_y_sz = sizeof(out_y);
	ret = hw_driver_mul(x, nn_byte_len(x_nn), y, nn_byte_len(y_nn),
			    scalar, nn_byte_len(&(in->crv->order)),
			    out_x, &out_x_sz, out_y, &out_y_sz); EG(ret, err);

	/* Import to the output */
	MUST_HAVE((out_x_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->X.fp_val), out_x, (u16)out_x_sz); EG(ret, err);
	MUST_HAVE((out_y_sz < 0xffff), ret, err);
	ret = nn_init_from_buf(&(out->Y.fp_val), out_y, (u16)out_y_sz); EG(ret, err);

	SET_INFINITY(out, 1);

err:
	return ret;
}


int hw_prj_pt_mul_ltr(prj_pt_t out, nn_src_t m, prj_pt_src_t in)
{
	int ret;

	mutex_lock();

	ret = _hw_prj_pt_mul_ltr(out, m, in);

	mutex_unlock();
	return ret;
}

int hw_prj_pt_mul_ltr_blind(prj_pt_t out, nn_src_t m, prj_pt_src_t in)
{
	int ret;

	mutex_lock();

	ret = prj_pt_check_initialized(in); EG(ret, err);

	/* Activate the blinding.
	 * NOTE: we set the blinding at the size of the curve order
	 * for maximum security.
	 */
	ret = hw_driver_set_blinding(nn_bit_len(&(in->crv->order))); EG(ret, err);

	/* Perform the scalar multiplication */
	ret = _hw_prj_pt_mul_ltr(out, m, in); EG(ret, err);

	/* Deactivate the blinding */
	ret = hw_driver_set_blinding(0);

err:
	mutex_unlock();
	return ret;
}

int hw_prj_pt_mul_ltr_small_scalar(prj_pt_t out, nn_src_t m, prj_pt_src_t in)
{
	int ret;
	bitcnt_t len;
		
	mutex_lock();

	ret = nn_bitlen(m, &len); EG(ret, err);

	/* Set the scalar size to be used, less than the nn size so
	 * that the hardware can optimize the computation.
	 * This is usually useful for public scalar multiplications with
	 * small scalars (such as cofactors and so on).
	 * */
	ret = hw_driver_set_small_scalar_size(len); EG(ret, err);

	/* Perform the multiplication */
	ret = _hw_prj_pt_mul_ltr(out, m, in);

err:
	mutex_unlock();
	return ret;
}

#else
/*
 * Dummy definition to avoid the empty translation unit ISO C warning
 */
typedef int dummy;
#endif /* WITH_EC_HW_ACCELERATOR */
