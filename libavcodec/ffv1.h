/*
 * FFV1 codec for libavcodec
 *
 * Copyright (c) 2003-2012 Michael Niedermayer <michaelni@gmx.at>
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

#ifndef AVCODEC_FFV1_H
#define AVCODEC_FFV1_H

/**
 * @file
 * FF Video Codec 1 (a lossless codec)
 */

#include "libavutil/avassert.h"
#include "libavutil/crc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timer.h"
#include "avcodec.h"
#include "get_bits.h"
#include "internal.h"
#include "mathops.h"
#include "put_bits.h"
#include "rangecoder.h"
#include "thread.h"

#include "libavutil/motion_vector.h"

#include "hpeldsp.h"
#include "me_cmp.h"
#include "qpeldsp.h"
#include "snow_dwt.h"

#define FF_MPV_OFFSET(x) (offsetof(MpegEncContext, x) + offsetof(FFV1Context, m))
#include "mpegvideo.h"
#include "h264qpel.h"

#define MID_STATE 128

#ifdef __INTEL_COMPILER
#undef av_flatten
#define av_flatten
#endif

#define MAX_PLANES 4
#define CONTEXT_SIZE 32
#define MAX_REF_FRAMES 8
#define FRAC_BITS 4

#define MAX_QUANT_TABLES 8
#define MAX_CONTEXT_INPUTS 5

#define AC_GOLOMB_RICE          0
#define AC_RANGE_DEFAULT_TAB    1
#define AC_RANGE_CUSTOM_TAB     2
#define AC_RANGE_DEFAULT_TAB_FORCE -2

#define QSHIFT 5
#define QROOT (1<<QSHIFT)

typedef struct VlcState {
    int16_t drift;
    uint16_t error_sum;
    int8_t bias;
    uint8_t count;
} VlcState;

#define HTAPS_MAX 8

typedef struct PlaneContext {
    int width;
    int height;
    //SubBand band[MAX_DECOMPOSITIONS][4];

    int htaps;
    int8_t hcoeff[HTAPS_MAX/2];
    int diag_mc;
    int fast_mc;

    int last_htaps;
    int8_t last_hcoeff[HTAPS_MAX/2];
    int last_diag_mc;

    int16_t quant_table[MAX_CONTEXT_INPUTS][256];
    int quant_table_index;
    int context_count;
    uint8_t (*state)[CONTEXT_SIZE];
    VlcState *vlc_state;
    uint8_t interlace_bit_state[2];
} PlaneContext;

#define LOG2_OBMC_MAX 8
#define OBMC_MAX (1<<(LOG2_OBMC_MAX))
typedef struct BlockNode {
    int16_t mx;
    int16_t my;
    uint8_t ref;
    uint8_t color[3];
    uint8_t type;
//#define TYPE_SPLIT    1
#define BLOCK_INTRA   1
#define BLOCK_OPT     2
//#define TYPE_NOCOLOR  4
    uint8_t level; //FIXME merge into type?
} BlockNode;

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

#define MAX_NLMeansImages 32

typedef struct
{
    uint16_t* img; // point to logical origin (0;0)
    int stride;

    int w,h;
    int border; // extra border on all four sides

    uint16_t* mem_start; // start of allocated memory
} MonoImage;

typedef struct
{
    MonoImage   plane[3];
} ColorImage;

struct PixelSum
{
    float weight_sum;
    float pixel_sum;
};

typedef struct {
    double h_param;
    int    patch_size;
    int    range;      // search range (must be odd number)
    int    n_frames;   // temporal search depth
    
    int hsub, vsub;
    
    struct PixelSum *tmp_data;
    uint32_t *integral_mem;

    ColorImage images[MAX_NLMeansImages];
    int     image_available[MAX_NLMeansImages];
} NLMContext;

#define MAX_SLICES 256

typedef struct FFV1Context {
    AVClass *class;
    AVCodecContext *avctx;
    RangeCoder c;
    GetBitContext gb;
    PutBitContext pb;
    uint64_t rc_stat[256][2];
    uint64_t (*rc_stat2[MAX_QUANT_TABLES])[32][2];
    int version;
    int micro_version;
    int width, height;
    int chroma_planes;
    int chroma_h_shift, chroma_v_shift;
    int transparency;
    int flags;
    int picture_number;
    int key_frame;
    ThreadFrame picture, last_picture, residual;
    struct FFV1Context *fsrc;

    AVFrame *cur;
    int plane_count;
    int ac;                              ///< 1=range coder <-> 0=golomb rice
    int ac_byte_count;                   ///< number of bytes used for AC coding
    PlaneContext plane[MAX_PLANES];
    int16_t quant_table[MAX_CONTEXT_INPUTS][256];
    int16_t quant_tables[MAX_QUANT_TABLES][MAX_CONTEXT_INPUTS][256];
    int context_count[MAX_QUANT_TABLES];
    uint8_t state_transition[256];
    uint8_t (*initial_states[MAX_QUANT_TABLES])[32];
    int run_index;
    int colorspace;
    int16_t *sample_buffer;

    uint16_t *p_image_line_buf, *c_image_line_buf;

    int ec;
    int intra;
    int slice_damaged;
    int key_frame_ok;
    int context_model;
    int p_frame;

    int bits_per_raw_sample;
    int packed_at_lsb;

    int gob_count;
    int quant_table_count;

    struct FFV1Context *slice_context[MAX_SLICES];
    int slice_count;
    int max_slice_count;
    int num_v_slices;
    int num_h_slices;
    int slice_width;
    int slice_height;
    int slice_x;
    int slice_y;
    int slice_reset_contexts;
    int slice_coding_mode;
    int slice_rct_by_coef;
    int slice_rct_ry_coef;

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

    uint32_t *ref_scores[MAX_REF_FRAMES];
    int16_t (*ref_mvs[MAX_REF_FRAMES])[2];

#define ME_CACHE_SIZE 1024
    unsigned me_cache[ME_CACHE_SIZE];
    unsigned me_cache_generation;
    int mv_scale;
    int iterative_dia_size;

    IDWTELEM *spatial_idwt_buffer;
    
    int nb_planes;
    
    NLMContext nlm;

} FFV1Context;

/* Tables */
extern const uint8_t * const ff_ffv1_obmc_tab[4];
extern uint8_t ff_qexp[QROOT];
extern int ff_scale_mv_ref[MAX_REF_FRAMES][MAX_REF_FRAMES];

int ff_ffv1_common_init(AVCodecContext *avctx);
int ff_ffv1_init_slice_state(FFV1Context *f, FFV1Context *fs);
int ff_ffv1_init_slices_state(FFV1Context *f);
int ff_ffv1_init_slice_contexts(FFV1Context *f);
int ff_ffv1_allocate_initial_states(FFV1Context *f);
int ff_ffv1_alloc_blocks(FFV1Context *s);
int ff_ffv1_common_init_after_header(AVCodecContext *avctx);
void ff_ffv1_release_buffer(AVCodecContext *avctx);
int ff_ffv1_get_buffer(FFV1Context *s, AVFrame *frame);
void ff_ffv1_clear_slice_state(FFV1Context *f, FFV1Context *fs);
void ff_ffv1_reset_contexts(FFV1Context *s);
int ff_ffv1_close(AVCodecContext *avctx);
int ff_ffv1_frame_start(FFV1Context *s);
void ff_ffv1_pred_block(FFV1Context *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride,
                     int sx, int sy, int b_w, int b_h, const BlockNode *block,
                     int plane_index, int w, int h);
void ff_ffv1_filter_frame(FFV1Context *f, AVFrame *frame);
int ff_ffv1_nlm_init(FFV1Context *f);

static inline void pred_mv(FFV1Context *s, int *mx, int *my, int ref,
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

static av_always_inline void add_yblock(FFV1Context *s, int sliced, slice_buffer *sb, IDWTELEM *dst, uint8_t *dst8, const uint8_t *obmc, int src_x, int src_y, int b_w, int b_h, int w, int h, int dst_stride, int src_stride, int obmc_stride, int b_x, int b_y, int add, int offset_dst, int plane_index){
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
    ff_ffv1_pred_block(s, block[0], tmp, src_stride, src_x, src_y, b_w, b_h, lt, plane_index, w, h);

    if(same_block(lt, rt)){
        block[1]= block[0];
    }else{
        block[1]= ptmp;
        ptmp+=tmp_step;
        ff_ffv1_pred_block(s, block[1], tmp, src_stride, src_x, src_y, b_w, b_h, rt, plane_index, w, h);
    }

    if(same_block(lt, lb)){
        block[2]= block[0];
    }else if(same_block(rt, lb)){
        block[2]= block[1];
    }else{
        block[2]= ptmp;
        ptmp+=tmp_step;
        ff_ffv1_pred_block(s, block[2], tmp, src_stride, src_x, src_y, b_w, b_h, lb, plane_index, w, h);
    }

    if(same_block(lt, rb) ){
        block[3]= block[0];
    }else if(same_block(rt, rb)){
        block[3]= block[1];
    }else if(same_block(lb, rb)){
        block[3]= block[2];
    }else{
        block[3]= ptmp;
        ff_ffv1_pred_block(s, block[3], tmp, src_stride, src_x, src_y, b_w, b_h, rb, plane_index, w, h);
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

static av_always_inline void predict_slice(FFV1Context *s, IDWTELEM *buf, int plane_index, int add, int mb_y){
    PlaneContext *p= &s->plane[plane_index];
    const int mb_w= s->b_width  << s->block_max_depth;
    const int mb_h= s->b_height << s->block_max_depth;
    int x, y, mb_x;
    int block_size = MB_SIZE >> s->block_max_depth;
    int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? ff_ffv1_obmc_tab[s->block_max_depth+s->chroma_h_shift] : ff_ffv1_obmc_tab[s->block_max_depth];
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

static av_always_inline void predict_plane(FFV1Context *s, IDWTELEM *buf, int plane_index, int add){
    const int mb_h= s->b_height << s->block_max_depth;
    int mb_y;
    for(mb_y=0; mb_y<=mb_h; mb_y++)
        predict_slice(s, buf, plane_index, add, mb_y);
}

static inline void set_blocks(FFV1Context *s, int level, int x, int y, int l, int cb, int cr, int mx, int my, int ref, int type)
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
    FFV1Context *s = c->avctx->priv_data;
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

static av_always_inline int fold(int diff, int bits)
{
    if (bits == 8)
        diff = (int8_t)diff;
    else {
        diff +=  1 << (bits  - 1);
        diff  = av_mod_uintp2(diff, bits);
        diff -=  1 << (bits  - 1);
    }

    return diff;
}

static inline int predict(int16_t *src, int16_t *last)
{
    const int LT = last[-1];
    const int T  = last[0];
    const int L  = src[-1];

    return mid_pred(L, L + T - LT, T);
}

static inline int get_context(PlaneContext *p, int16_t *src,
                              int16_t *last, int16_t *last2)
{
    const int LT = last[-1];
    const int T  = last[0];
    const int RT = last[1];
    const int L  = src[-1];

    if (p->quant_table[3][127]) {
        const int TT = last2[0];
        const int LL = src[-2];
        return p->quant_table[0][(L - LT) & 0xFF] +
               p->quant_table[1][(LT - T) & 0xFF] +
               p->quant_table[2][(T - RT) & 0xFF] +
               p->quant_table[3][(LL - L) & 0xFF] +
               p->quant_table[4][(TT - T) & 0xFF];
    } else
        return p->quant_table[0][(L - LT) & 0xFF] +
               p->quant_table[1][(LT - T) & 0xFF] +
               p->quant_table[2][(T - RT) & 0xFF];
}

static inline void update_vlc_state(VlcState *const state, const int v)
{
    int drift = state->drift;
    int count = state->count;
    state->error_sum += FFABS(v);
    drift            += v;

    if (count == 128) { // FIXME: variable
        count            >>= 1;
        drift            >>= 1;
        state->error_sum >>= 1;
    }
    count++;

    if (drift <= -count) {
        if (state->bias > -128)
            state->bias--;

        drift += count;
        if (drift <= -count)
            drift = -count + 1;
    } else if (drift > 0) {
        if (state->bias < 127)
            state->bias++;

        drift -= count;
        if (drift > 0)
            drift = 0;
    }

    state->drift = drift;
    state->count = count;
}

#endif /* AVCODEC_FFV1_H */
