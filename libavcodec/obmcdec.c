#include "obmcdec.h"

int obmc_decode_init(OBMCContext *f) {
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(f->avctx->pix_fmt);
    f->nb_planes = 0;
    for (i = 0; i < desc->nb_components; i++)
        f->nb_planes = FFMAX(f->nb_planes, desc->comp[i].plane + 1);
        
    return 0;
}

static int decode_q_branch(OBMCContext *s, int level, int x, int y){
    RangeCoder *const c = s->c;
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

    if(level==s->block_max_depth || get_rac(c, &s->block_state[4 + s_context])){
        int type, mx, my;
        int l = left->color[0];
        int cb= left->color[1];
        int cr= left->color[2];
        unsigned ref = 0;
        int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
        int mx_context= av_log2(2*FFABS(left->mx - top->mx)) + 0*av_log2(2*FFABS(tr->mx - top->mx));
        int my_context= av_log2(2*FFABS(left->my - top->my)) + 0*av_log2(2*FFABS(tr->my - top->my));

        type= get_rac(c, &s->block_state[1 + left->type + top->type]) ? BLOCK_INTRA : 0;

        if(type){
            pred_mv(s, &mx, &my, 0, left, top, tr);
            l += get_symbol(c, &s->block_state[32], 1);
            if (s->nb_planes > 2) {
                cb+= get_symbol(c, &s->block_state[64], 1);
                cr+= get_symbol(c, &s->block_state[96], 1);
            }
        }else{
            if(s->ref_frames > 1)
                ref= get_symbol(c, &s->block_state[128 + 1024 + 32*ref_context], 0);
            if (ref >= s->ref_frames) {
                av_log(s->avctx, AV_LOG_ERROR, "Invalid ref\n");
                return AVERROR_INVALIDDATA;
            }
            pred_mv(s, &mx, &my, ref, left, top, tr);
            mx+= get_symbol(c, &s->block_state[128 + 32*(mx_context + 16*!!ref)], 1);
            my+= get_symbol(c, &s->block_state[128 + 32*(my_context + 16*!!ref)], 1);
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
    for(plane_index=0; plane_index < f->nb_planes; plane_index++){
       Plane *pc= &f->plane[plane_index];
       pc->fast_mc= pc->diag_mc && pc->htaps==6 && pc->hcoeff[0]==40
                                             && pc->hcoeff[1]==-10
                                             && pc->hcoeff[2]==2;
    }

    ff_obmc_alloc_blocks(f);

    if ((ret = ff_obmc_frame_start(f)) < 0)
        return ret;

    f->current_picture->pict_type = f->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

    if ((ret = decode_blocks(f)) < 0)
        return ret;
        
    return 0;
}