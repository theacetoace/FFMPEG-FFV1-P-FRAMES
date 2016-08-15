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
 
 #include "obmcdec.h"

int obmc_decode_init(OBMCContext *f) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(f->avctx->pix_fmt);
    int i;
    f->nb_planes = 0;
    for (i = 0; i < desc->nb_components; i++)
        f->nb_planes = FFMAX(f->nb_planes, desc->comp[i].plane + 1);
        
    avcodec_get_chroma_sub_sample(f->avctx->pix_fmt, &f->chroma_h_shift, &f->chroma_v_shift);
        
    return 0;
}

static int decode_q_branch(OBMCContext *s, int level, int x, int y){
    ObmcCoderContext *const c = &s->obmc_coder;
    const int w= s->b_width << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    int trx= (x+1)<<rem_depth;
    const BlockNode *left  = x ? &s->block[index-1] : &null_block;
    const BlockNode *top   = y ? &s->block[index-w] : &null_block;
    const BlockNode *tl    = y && x ? &s->block[index-w-1] : left;
    const BlockNode *tr    = y && trx<w && ((x&1)==0 || level==0) ? &s->block[index-w+(1<<rem_depth)] : tl; //FIXME use lt
    int s_context= 2*left->level + 2*top->level + tl->level + tr->level;
    int res;

    if(s->key_frame){
        set_blocks(s, level, x, y, null_block.color[0], null_block.color[1], null_block.color[2], null_block.mx, null_block.my, null_block.ref, BLOCK_INTRA);
        return 0;
    }

    if(level==s->block_max_depth || c->get_level_break(c, 4 + s_context)){
        int type, mx, my;
        int l = left->color[0];
        int cb= left->color[1];
        int cr= left->color[2];
        unsigned ref = 0;
        int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
        int mx_context= av_log2(2*FFABS(left->mx - top->mx)) + 0*av_log2(2*FFABS(tr->mx - top->mx));
        int my_context= av_log2(2*FFABS(left->my - top->my)) + 0*av_log2(2*FFABS(tr->my - top->my));

        type= c->get_block_type(c, 1 + left->type + top->type) ? BLOCK_INTRA : 0;

        if(type){
            pred_mv(s, &mx, &my, 0, left, top, tr);
            c->get_block_color(c, 32, 64, 96, &l, &cb, &cr);
        }else{
            if(s->ref_frames > 1)
                ref= c->get_best_ref(c, 128 + 1024 + 32*ref_context);
            if (ref >= s->ref_frames) {
                av_log(s->avctx, AV_LOG_ERROR, "Invalid ref\n");
                return AVERROR_INVALIDDATA;
            }
            pred_mv(s, &mx, &my, ref, left, top, tr);
            c->get_block_mv(c, 
                128 + 32*(mx_context + 16*!!ref), 128 + 32*(my_context + 16*!!ref),
                &mx, &my
            );
        }
        set_blocks(s, level, x, y, l, cb, cr, mx, my, ref, type);
    }else{
        if ((res = decode_q_branch(s, level+1, 2*x+0, 2*y+0)) < 0 ||
            (res = decode_q_branch(s, level+1, 2*x+1, 2*y+0)) < 0 ||
            (res = decode_q_branch(s, level+1, 2*x+0, 2*y+1)) < 0 ||
            (res = decode_q_branch(s, level+1, 2*x+1, 2*y+1)) < 0)
            return res;
    }
    return 0;
}

static int decode_blocks(OBMCContext *s){
    int x, y;
    int w= s->b_width;
    int h= s->b_height;
    int res;

    for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            if ((res = decode_q_branch(s, 0, x, y)) < 0)
                return res;
        }
    }

    return 0;
}

int obmc_decode_frame(OBMCContext *f) {
    int plane_index, ret;
    for(plane_index=0; plane_index < f->nb_planes; plane_index++){
       PlaneObmc *pc= &f->plane[plane_index];
       pc->fast_mc= pc->diag_mc && pc->htaps==6 && pc->hcoeff[0]==40
                                             && pc->hcoeff[1]==-10
                                             && pc->hcoeff[2]==2;
    }

    ff_obmc_alloc_blocks(f);

    if ((ret = ff_obmc_frame_start(f)) < 0)
        return ret;

    f->current_picture->pict_type = f->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;
    
    av_assert0(!f->avmv);
    if (f->avctx->flags2 & AV_CODEC_FLAG2_EXPORT_MVS) {
        f->avmv = av_malloc_array(f->b_width * f->b_height, sizeof(AVMotionVector) << (f->block_max_depth*2));
    }
    f->avmv_index = 0;

    if ((ret = decode_blocks(f)) < 0)
        return ret;
        
    return 0;
}