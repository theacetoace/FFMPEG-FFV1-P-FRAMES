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

 #include "obmc.h"

int ff_obmc_decode_init(OBMCContext *f) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(f->avctx->pix_fmt);
    if (!desc)
        return AVERROR_INVALIDDATA;
    int i;
    f->nb_planes = 0;
    for (i = 0; i < desc->nb_components; i++)
        f->nb_planes = FFMAX(f->nb_planes, desc->comp[i].plane + 1);

    avcodec_get_chroma_sub_sample(f->avctx->pix_fmt, &f->chroma_h_shift, &f->chroma_v_shift);

    return 0;
}

int ff_obmc_predecode_frame(OBMCContext *f) {
    int plane_index, ret;
    for(plane_index=0; plane_index < f->nb_planes; plane_index++){
       PlaneObmc *pc= &f->plane[plane_index];
       pc->fast_mc= pc->diag_mc && pc->htaps==6 && pc->hcoeff[0]==40
                                             && pc->hcoeff[1]==-10
                                             && pc->hcoeff[2]==2;
    }

    if ((ret = ff_obmc_alloc_blocks(f)) < 0)
        return ret;

    if ((ret = ff_obmc_frame_start(f)) < 0)
        return ret;

    f->current_picture->pict_type = f->keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    av_assert0(!f->avmv);
    if (f->avctx->flags2 & AV_CODEC_FLAG2_EXPORT_MVS) {
        f->avmv = av_malloc_array(f->b_width * f->b_height, sizeof(AVMotionVector) << (f->block_max_depth*2));
    }
    f->avmv_index = 0;

    return 0;
}
