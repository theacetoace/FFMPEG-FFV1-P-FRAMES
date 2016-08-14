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

#ifndef AVCODEC_OBMC_H
#define AVCODEC_OBMC_H

#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/motion_vector.h"

#include "rangecoder.h"

#include "hpeldsp.h"
#include "me_cmp.h"
#include "qpeldsp.h"
#include "snow_dwt.h"

#define FF_MPV_OFFSET(x) (offsetof(MpegEncContext, x) + offsetof(OBMCContext, m))
#include "mpegvideo.h"
#include "h264qpel.h"

#define MID_STATE 128

#define MAX_PLANES 4
#define QSHIFT 5
#define QROOT (1<<QSHIFT)
#define LOSSLESS_QLOG -128
#define FRAC_BITS 4
#define MAX_REF_FRAMES 8

#define LOG2_OBMC_MAX 8
#define OBMC_MAX (1<<(LOG2_OBMC_MAX))
typedef struct BlockNode{
    int16_t mx;         ///< Motion vector component X, see mv_scale
    int16_t my;         ///< Motion vector component Y, see mv_scale
    uint8_t ref;        ///< Reference frame index
    uint8_t color[3];   ///< Color for intra
    uint8_t type;       ///< Bitfield of BLOCK_*
//#define TYPE_SPLIT    1
#define BLOCK_INTRA   1
#define BLOCK_OPT     2
//#define TYPE_NOCOLOR  4
    uint8_t level; //FIXME merge into type?
}BlockNode;

static const BlockNode null_block= { //FIXME add border maybe
    .color= {128,128,128},
    .mx= 0,
    .my= 0,
    .ref= 0,
    .type= 0,
    .level= 0,
};

#define LOG2_MB_SIZE 4
#define MB_SIZE (1<<LOG2_MB_SIZE)
#define ENCODER_EXTRA_BITS 4
#define HTAPS_MAX 8

typedef struct PlaneObmc{
    int width;
    int height;

    int htaps;
    int8_t hcoeff[HTAPS_MAX/2];
    int diag_mc;
    int fast_mc;

    int last_htaps;
    int8_t last_hcoeff[HTAPS_MAX/2];
    int last_diag_mc;
} PlaneObmc;

typedef struct OBMCContext {
    AVCodecContext *avctx;
    RangeCoder *c;
    int key_frame;
    int chroma_h_shift, chroma_v_shift;
    
    int (*get_symbol)(RangeCoder *, uint8_t *, int );
    void (*put_symbol)(RangeCoder *, uint8_t *, int, int);
    
    /* ME/MC part */
    MECmpContext mecc;
    HpelDSPContext hdsp;
    QpelDSPContext qdsp;
    VideoDSPContext vdsp;
    H264QpelContext h264qpel;
    MpegvideoEncDSPContext mpvencdsp;
    SnowDWTContext dwt;

    AVFrame *input_picture;              ///< new_picture with the internal linesizes
    AVFrame *current_picture;
    AVFrame *last_pictures[MAX_REF_FRAMES];
    uint8_t *halfpel_plane[MAX_REF_FRAMES][4][4];
    AVFrame *mconly_picture;

    MpegEncContext m; // needed for motion estimation, should not be used for anything else, the idea is to eventually make the motion estimation independent of MpegEncContext, so this will be removed then (FIXME/XXX)

    uint8_t *scratchbuf;
    uint8_t *emu_edge_buffer;

    AVMotionVector *avmv;
    int avmv_index;

    int motion_est;
    int intra_penalty;
    int lambda;
    int lambda2;
    int max_ref_frames;
    int ref_frames;

    uint8_t block_state[128 + 32*128];
    BlockNode *block;
    int b_width;
    int b_height;
    int block_max_depth;
    int last_block_max_depth;

    uint32_t *ref_scores[MAX_REF_FRAMES];
    int16_t (*ref_mvs[MAX_REF_FRAMES])[2];

#define ME_CACHE_SIZE 1024
    unsigned me_cache[ME_CACHE_SIZE];
    unsigned me_cache_generation;
    int mv_scale;
    int last_mv_scale;
    int iterative_dia_size;

    IDWTELEM *spatial_idwt_buffer;
    
    PlaneObmc plane[MAX_PLANES];
    
    int nb_planes;
} OBMCContext;

/* Tables */
extern const uint8_t * const ff_obmc_tab[4];
extern uint8_t ff_qexp[QROOT];
extern int ff_scale_mv_ref[MAX_REF_FRAMES][MAX_REF_FRAMES];

int ff_obmc_get_buffer(OBMCContext *s, AVFrame *frame);
void ff_obmc_release_buffer(OBMCContext *s);
int ff_obmc_common_init_after_header(OBMCContext *s);
int ff_obmc_frame_start(OBMCContext *f);
int ff_obmc_alloc_blocks(OBMCContext *s);
void ff_obmc_reset_contexts(OBMCContext *s);
int ff_obmc_common_init(OBMCContext *s, AVCodecContext *avctx);
int ff_obmc_close(OBMCContext *s);
void ff_obmc_pred_block(OBMCContext *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride,
                     int sx, int sy, int b_w, int b_h, const BlockNode *block,
                     int plane_index, int w, int h);

static inline void pred_mv(OBMCContext *s, int *mx, int *my, int ref,
                           const BlockNode *left, const BlockNode *top, const BlockNode *tr){
    if(s->ref_frames == 1){
        *mx = mid_pred(left->mx, top->mx, tr->mx);
        *my = mid_pred(left->my, top->my, tr->my);
    }else{
        const int *scale = ff_scale_mv_ref[ref];
        *mx = mid_pred((left->mx * scale[left->ref] + 128) >>8,
                       (top ->mx * scale[top ->ref] + 128) >>8,
                       (tr  ->mx * scale[tr  ->ref] + 128) >>8);
        *my = mid_pred((left->my * scale[left->ref] + 128) >>8,
                       (top ->my * scale[top ->ref] + 128) >>8,
                       (tr  ->my * scale[tr  ->ref] + 128) >>8);
    }
}

static av_always_inline int same_block(BlockNode *a, BlockNode *b){
    if((a->type&BLOCK_INTRA) && (b->type&BLOCK_INTRA)){
        return !((a->color[0] - b->color[0]) | (a->color[1] - b->color[1]) | (a->color[2] - b->color[2]));
    }else{
        return !((a->mx - b->mx) | (a->my - b->my) | (a->ref - b->ref) | ((a->type ^ b->type)&BLOCK_INTRA));
    }
}

static av_always_inline void add_yblock(OBMCContext *s, int sliced, slice_buffer *sb, IDWTELEM *dst, uint8_t *dst8, const uint8_t *obmc, int src_x, int src_y, int b_w, int b_h, int w, int h, int dst_stride, int src_stride, int obmc_stride, int b_x, int b_y, int add, int offset_dst, int plane_index){
    const int b_width = s->b_width  << s->block_max_depth;
    const int b_height= s->b_height << s->block_max_depth;
    const int b_stride= b_width;
    BlockNode *lt= &s->block[b_x + b_y*b_stride];
    BlockNode *rt= lt+1;
    BlockNode *lb= lt+b_stride;
    BlockNode *rb= lb+1;
    uint8_t *block[4];
    // When src_stride is large enough, it is possible to interleave the blocks.
    // Otherwise the blocks are written sequentially in the tmp buffer.
    int tmp_step= src_stride >= 7*MB_SIZE ? MB_SIZE : MB_SIZE*src_stride;
    uint8_t *tmp = s->scratchbuf;
    uint8_t *ptmp;
    int x,y;

    if(b_x<0){
        lt= rt;
        lb= rb;
    }else if(b_x + 1 >= b_width){
        rt= lt;
        rb= lb;
    }
    if(b_y<0){
        lt= lb;
        rt= rb;
    }else if(b_y + 1 >= b_height){
        lb= lt;
        rb= rt;
    }

    if(src_x<0){ //FIXME merge with prev & always round internal width up to *16
        obmc -= src_x;
        b_w += src_x;
        if(!sliced && !offset_dst)
            dst -= src_x;
        src_x=0;
    }
    if(src_x + b_w > w){
        b_w = w - src_x;
    }
    if(src_y<0){
        obmc -= src_y*obmc_stride;
        b_h += src_y;
        if(!sliced && !offset_dst)
            dst -= src_y*dst_stride;
        src_y=0;
    }
    if(src_y + b_h> h){
        b_h = h - src_y;
    }

    if(b_w<=0 || b_h<=0) return;

    if(!sliced && offset_dst)
        dst += src_x + src_y*dst_stride;
    dst8+= src_x + src_y*src_stride;
//    src += src_x + src_y*src_stride;

    ptmp= tmp + 3*tmp_step;
    block[0]= ptmp;
    ptmp+=tmp_step;
    ff_obmc_pred_block(s, block[0], tmp, src_stride, src_x, src_y, b_w, b_h, lt, plane_index, w, h);

    if(same_block(lt, rt)){
        block[1]= block[0];
    }else{
        block[1]= ptmp;
        ptmp+=tmp_step;
        ff_obmc_pred_block(s, block[1], tmp, src_stride, src_x, src_y, b_w, b_h, rt, plane_index, w, h);
    }

    if(same_block(lt, lb)){
        block[2]= block[0];
    }else if(same_block(rt, lb)){
        block[2]= block[1];
    }else{
        block[2]= ptmp;
        ptmp+=tmp_step;
        ff_obmc_pred_block(s, block[2], tmp, src_stride, src_x, src_y, b_w, b_h, lb, plane_index, w, h);
    }

    if(same_block(lt, rb) ){
        block[3]= block[0];
    }else if(same_block(rt, rb)){
        block[3]= block[1];
    }else if(same_block(lb, rb)){
        block[3]= block[2];
    }else{
        block[3]= ptmp;
        ff_obmc_pred_block(s, block[3], tmp, src_stride, src_x, src_y, b_w, b_h, rb, plane_index, w, h);
    }
    if(sliced){
        s->dwt.inner_add_yblock(obmc, obmc_stride, block, b_w, b_h, src_x,src_y, src_stride, sb, add, dst8);
    }else{
        for(y=0; y<b_h; y++){
            //FIXME ugly misuse of obmc_stride
            const uint8_t *obmc1= obmc + y*obmc_stride;
            const uint8_t *obmc2= obmc1+ (obmc_stride>>1);
            const uint8_t *obmc3= obmc1+ obmc_stride*(obmc_stride>>1);
            const uint8_t *obmc4= obmc3+ (obmc_stride>>1);
            for(x=0; x<b_w; x++){
                int v=   obmc1[x] * block[3][x + y*src_stride]
                        +obmc2[x] * block[2][x + y*src_stride]
                        +obmc3[x] * block[1][x + y*src_stride]
                        +obmc4[x] * block[0][x + y*src_stride];

                v <<= 8 - LOG2_OBMC_MAX;
                if(FRAC_BITS != 8) {
                    v >>= 8 - FRAC_BITS;
                }
                if(add) {
                    v += dst[x + y*dst_stride];
                    v = (v + (1<<(FRAC_BITS-1))) >> FRAC_BITS;
                    if(v&(~255)) v= ~(v>>31);
                    dst8[x + y*src_stride] = v;
                } else {
                    dst[x + y*dst_stride] -= v;
                }
            }
        }
    }
}

static av_always_inline void predict_slice(OBMCContext *s, IDWTELEM *buf, int plane_index, int add, int mb_y){
    PlaneObmc *p= &s->plane[plane_index];
    const int mb_w= s->b_width  << s->block_max_depth;
    const int mb_h= s->b_height << s->block_max_depth;
    int x, y, mb_x;
    int block_size = MB_SIZE >> s->block_max_depth;
    int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? ff_obmc_tab[s->block_max_depth+s->chroma_h_shift] : ff_obmc_tab[s->block_max_depth];
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    int ref_stride= s->current_picture->linesize[plane_index];
    uint8_t *dst8= s->current_picture->data[plane_index];
    int w= p->width;
    int h= p->height;
    av_assert2(s->chroma_h_shift == s->chroma_v_shift); // obmc params assume squares
    if(s->key_frame || (s->avctx->debug&512)){
        if(mb_y==mb_h)
            return;

        if(add){
            for(y=block_h*mb_y; y<FFMIN(h,block_h*(mb_y+1)); y++){
                for(x=0; x<w; x++){
                    int v= buf[x + y*w] + (128<<FRAC_BITS) + (1<<(FRAC_BITS-1));
                    v >>= FRAC_BITS;
                    if(v&(~255)) v= ~(v>>31);
                    dst8[x + y*ref_stride]= v;
                }
            }
        }else{
            for(y=block_h*mb_y; y<FFMIN(h,block_h*(mb_y+1)); y++){
                for(x=0; x<w; x++){
                    buf[x + y*w]-= 128<<FRAC_BITS;
                }
            }
        }

        return;
    }

    for(mb_x=0; mb_x<=mb_w; mb_x++){
        add_yblock(s, 0, NULL, buf, dst8, obmc,
                   block_w*mb_x - block_w/2,
                   block_h*mb_y - block_h/2,
                   block_w, block_h,
                   w, h,
                   w, ref_stride, obmc_stride,
                   mb_x - 1, mb_y - 1,
                   add, 1, plane_index);
    }
}

static av_always_inline void predict_slice_buffered(OBMCContext *s, slice_buffer * sb, IDWTELEM * old_buffer, int plane_index, int add, int mb_y){
    PlaneObmc *p= &s->plane[plane_index];
    const int mb_w= s->b_width  << s->block_max_depth;
    const int mb_h= s->b_height << s->block_max_depth;
    int x, y, mb_x;
    int block_size = MB_SIZE >> s->block_max_depth;
    int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? ff_obmc_tab[s->block_max_depth+s->chroma_h_shift] : ff_obmc_tab[s->block_max_depth];
    int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    int ref_stride= s->current_picture->linesize[plane_index];
    uint8_t *dst8= s->current_picture->data[plane_index];
    int w= p->width;
    int h= p->height;

    if(s->key_frame || (s->avctx->debug&512)){
        if(mb_y==mb_h)
            return;

        if(add){
            for(y=block_h*mb_y; y<FFMIN(h,block_h*(mb_y+1)); y++){
                IDWTELEM * line = sb->line[y];
                for(x=0; x<w; x++){
                    int v= line[x] + (128<<FRAC_BITS) + (1<<(FRAC_BITS-1));
                    v >>= FRAC_BITS;
                    if(v&(~255)) v= ~(v>>31);
                    dst8[x + y*ref_stride]= v;
                }
            }
        }else{
            for(y=block_h*mb_y; y<FFMIN(h,block_h*(mb_y+1)); y++){
                IDWTELEM * line = sb->line[y];
                for(x=0; x<w; x++){
                    line[x] -= 128 << FRAC_BITS;
                }
            }
        }

        return;
    }

    for(mb_x=0; mb_x<=mb_w; mb_x++){
        add_yblock(s, 1, sb, old_buffer, dst8, obmc,
                   block_w*mb_x - block_w/2,
                   block_h*mb_y - block_h/2,
                   block_w, block_h,
                   w, h,
                   w, ref_stride, obmc_stride,
                   mb_x - 1, mb_y - 1,
                   add, 0, plane_index);
    }

    if(s->avmv && mb_y < mb_h && plane_index == 0)
        for(mb_x=0; mb_x<mb_w; mb_x++){
            AVMotionVector *avmv = s->avmv + s->avmv_index;
            const int b_width = s->b_width  << s->block_max_depth;
            const int b_stride= b_width;
            BlockNode *bn= &s->block[mb_x + mb_y*b_stride];

            if (bn->type)
                continue;

            s->avmv_index++;

            avmv->w = block_w;
            avmv->h = block_h;
            avmv->dst_x = block_w*mb_x - block_w/2;
            avmv->dst_y = block_h*mb_y - block_h/2;
            avmv->motion_scale = 8;
            avmv->motion_x = bn->mx * s->mv_scale;
            avmv->motion_y = bn->my * s->mv_scale;
            avmv->src_x = avmv->dst_x + avmv->motion_x / 8;
            avmv->src_y = avmv->dst_y + avmv->motion_y / 8;
            avmv->source= -1 - bn->ref;
            avmv->flags = 0;
        }
}

static av_always_inline void predict_plane(OBMCContext *s, IDWTELEM *buf, int plane_index, int add){
    const int mb_h= s->b_height << s->block_max_depth;
    int mb_y;
    for(mb_y=0; mb_y<=mb_h; mb_y++)
        predict_slice(s, buf, plane_index, add, mb_y);
}

static inline void set_blocks(OBMCContext *s, int level, int x, int y, int l, int cb, int cr, int mx, int my, int ref, int type)
{
    const int w= s->b_width << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    const int block_w= 1<<rem_depth;
    const int block_h= 1<<rem_depth; //FIXME "w!=h"
    BlockNode block;
    int i,j;

    block.color[0]= l;
    block.color[1]= cb;
    block.color[2]= cr;
    block.mx= mx;
    block.my= my;
    block.ref= ref;
    block.type= type;
    block.level= level;

    for(j=0; j<block_h; j++){
        for(i=0; i<block_w; i++){
            s->block[index + i + j*w]= block;
        }
    }
}

static inline void init_ref(MotionEstContext *c, uint8_t *src[3], uint8_t *ref[3], uint8_t *ref2[3], int x, int y, int ref_index){
    OBMCContext *s = c->avctx->priv_data;
    const int offset[3]= {
          y*c->  stride + x,
        ((y*c->uvstride + x)>>s->chroma_h_shift),
        ((y*c->uvstride + x)>>s->chroma_h_shift),
    };
    int i;
    for(i=0; i<3; i++){
        c->src[0][i]= src [i];
        c->ref[0][i]= ref [i] + offset[i];
    }
    av_assert2(!ref_index);
}

#endif /* AVCODEC_OBMC_H */
