/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (C) 2006 Robert Edele <yartrebo@earthlink.net>
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
 
#ifndef AVCODEC_OBMCENC_H
#define AVCODEC_OBMCENC_H

#include "obmc.h"

#define FF_ME_ITER 50

int obmc_encode_init(OBMCContext *s, AVCodecContext *avctx);
int obmc_pre_encode_frame(OBMCContext *f, AVCodecContext *avctx, const AVFrame *pict);
void obmc_encode_blocks(OBMCContext *s, int search);

#endif /* AVCODEC_OBMCENC_H */