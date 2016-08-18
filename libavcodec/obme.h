/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

 /**
 * @file obme.h
 * @brief Overlapped block motion estimation functions
 */

#ifndef AVCODEC_OBME_H
#define AVCODEC_OBME_H

#include "obmemc.h"

#define FF_ME_ITER 50

 /**
 * Inits OBMC context parameters needed for encoding process
 *
 * @param[in,out] s OBMC context to init
 * @param[in] avctx Codec context to retrieve some initial values
 */
int ff_obmc_encode_init(OBMCContext *s, AVCodecContext *avctx);

 /**
 * Prepares OBMC context for block encoding for each frame
 *
 * @param[in,out] f OBMC context to prepare
 * @param[in] avctx Codec context to retrieve some required values
 * @param[in] pict Frame to encode
 */
int ff_obmc_pre_encode_frame(OBMCContext *f, AVCodecContext *avctx, const AVFrame *pict);

 /**
 * Starts encoding blocks
 *
 * @param[in,out] s OBMC context
 * @param[in] search Defines if reference blocks should be searched for
 */
void ff_obmc_encode_blocks(OBMCContext *s, int search);

#endif /* AVCODEC_OBME_H */
