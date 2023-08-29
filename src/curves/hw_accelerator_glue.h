/*
 *  Copyright (C) 2023 - This file is part of libecc project
 *
 *  Authors:
 *      Ryad BENADJILA <ryadbenadjila@gmail.com>
 *      Karim KHALFALLAH <karim.khalfallah@ssi.gouv.fr>
 *
 *  This software is licensed under a dual BSD and GPL v2 license.
 *  See LICENSE file at the root folder of the project.
 */

#ifndef __HW_ACCELERATOR_GLUE_H__
#define __HW_ACCELERATOR_GLUE_H__

#if defined(WITH_EC_HW_ACCELERATOR)

/* Hardware/external accelerator driver abstraction */
#include <libecc/curves/prj_pt.h>
#include <libecc/curves/aff_pt.h>
#include <libecc/nn/nn_logical.h>

int hw_set_curve(ec_shortw_crv_src_t curve);

int hw_is_on_shortw_curve(fp_src_t x, fp_src_t y, ec_shortw_crv_src_t curve, int *on_curve);

int hw_ec_shortw_aff_eq_or_opp(aff_pt_src_t in1, aff_pt_src_t in2, int *aff_is_eq_or_opp);

int hw_prj_pt_is_on_curve(prj_pt_src_t in, int *on_curve);

int hw_prj_pt_eq_or_opp(prj_pt_src_t in1, prj_pt_src_t in2, int *eq_or_opp);

int hw_prj_pt_neg(prj_pt_t out, prj_pt_src_t in);

int hw_prj_pt_dbl(prj_pt_t out, prj_pt_src_t in);

int hw_prj_pt_add(prj_pt_t out, prj_pt_src_t in1, prj_pt_src_t in2);

int hw_prj_pt_mul_ltr(prj_pt_t out, nn_src_t m, prj_pt_src_t in);

int hw_prj_pt_mul_ltr_blind(prj_pt_t out, nn_src_t m, prj_pt_src_t in);

int hw_prj_pt_mul_ltr_small_scalar(prj_pt_t out, nn_src_t m, prj_pt_src_t in);

#endif

#endif /* __HW_ACCELERATOR_GLUE_H__ */
