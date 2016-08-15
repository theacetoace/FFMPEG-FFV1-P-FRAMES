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

#ifndef AVCODEC_SNOW_H
#define AVCODEC_SNOW_H

#include "libavutil/motion_vector.h"

#include "hpeldsp.h"
#include "me_cmp.h"
#include "qpeldsp.h"
#include "snow_dwt.h"

#include "rangecoder.h"
#include "mathops.h"

#define FF_MPV_OFFSET(x) (offsetof(MpegEncContext, x) + offsetof(SnowContext, obmc.m))
#include "obmc.h"

#define MID_STATE 128

typedef struct x_and_coeff{
    int16_t x;
    uint16_t coeff;
} x_and_coeff;

typedef struct SubBand{
    int level;
    int stride;
    int width;
    int height;
    int qlog;        ///< log(qscale)/log[2^(1/6)]
    DWTELEM *buf;
    IDWTELEM *ibuf;
    int buf_x_offset;
    int buf_y_offset;
    int stride_line; ///< Stride measured in lines, not pixels.
    x_and_coeff * x_coeff;
    struct SubBand *parent;
    uint8_t state[/*7*2*/ 7 + 512][32];
}SubBand;

typedef struct Plane{
    int width;
    int height;
    SubBand band[MAX_DECOMPOSITIONS][4];
} Plane;

typedef struct SnowContext{
    AVClass *class;
    AVCodecContext *avctx;
    RangeCoder c;
//     uint8_t q_context[16];
    uint8_t header_state[32];
    uint8_t block_state[128 + 32*128];
    int keyframe;
    int always_reset;
    int version;
    int spatial_decomposition_type;
    int last_spatial_decomposition_type;
    int temporal_decomposition_type;
    int spatial_decomposition_count;
    int last_spatial_decomposition_count;
    int temporal_decomposition_count;
    DWTELEM *spatial_dwt_buffer;
    DWTELEM *temp_dwt_buffer;
    //IDWTELEM *spatial_idwt_buffer;
    IDWTELEM *temp_idwt_buffer;
    int *run_buffer;
    int colorspace_type;
    int chroma_h_shift;
    int chroma_v_shift;
    int spatial_scalability;
    int qlog;
    int last_qlog;
    int pass1_rc;
    int qbias;
    int last_qbias;
#define QBIAS_SHIFT 3
    int nb_planes;
    Plane plane[MAX_PLANES];
    slice_buffer sb;
    int memc_only;
    int no_bitstream;
    int scenechange_threshold;
    
    uint64_t encoding_error[AV_NUM_DATA_POINTERS];
    
    OBMCContext obmc;

    int pred;    
}SnowContext;

/* C bits used by mmx/sse2/altivec */

static av_always_inline void snow_interleave_line_header(int * i, int width, IDWTELEM * low, IDWTELEM * high){
    (*i) = (width) - 2;

    if (width & 1){
        low[(*i)+1] = low[((*i)+1)>>1];
        (*i)--;
    }
}

static av_always_inline void snow_interleave_line_footer(int * i, IDWTELEM * low, IDWTELEM * high){
    for (; (*i)>=0; (*i)-=2){
        low[(*i)+1] = high[(*i)>>1];
        low[*i] = low[(*i)>>1];
    }
}

static av_always_inline void snow_horizontal_compose_lift_lead_out(int i, IDWTELEM * dst, IDWTELEM * src, IDWTELEM * ref, int width, int w, int lift_high, int mul, int add, int shift){
    for(; i<w; i++){
        dst[i] = src[i] - ((mul * (ref[i] + ref[i + 1]) + add) >> shift);
    }

    if((width^lift_high)&1){
        dst[w] = src[w] - ((mul * 2 * ref[w] + add) >> shift);
    }
}

static av_always_inline void snow_horizontal_compose_liftS_lead_out(int i, IDWTELEM * dst, IDWTELEM * src, IDWTELEM * ref, int width, int w){
        for(; i<w; i++){
            dst[i] = src[i] + ((ref[i] + ref[(i+1)]+W_BO + 4 * src[i]) >> W_BS);
        }

        if(width&1){
            dst[w] = src[w] + ((2 * ref[w] + W_BO + 4 * src[w]) >> W_BS);
        }
}

/* common code */

int ff_snow_common_init(AVCodecContext *avctx);
int ff_snow_common_init_after_header(AVCodecContext *avctx);
void ff_snow_common_end(SnowContext *s);
void ff_snow_reset_contexts(SnowContext *s);
/* common inline functions */
//XXX doublecheck all of them should stay inlined

/*static inline void snow_set_blocks(SnowContext *s, int level, int x, int y, int l, int cb, int cr, int mx, int my, int ref, int type){
    const int w= s->b_width << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    const int block_w= 1<<rem_depth;
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

    for(j=0; j<block_w; j++){
        for(i=0; i<block_w; i++){
            s->block[index + i + j*w]= block;
        }
    }
}*/

/* bitstream functions */

extern const int8_t ff_quant3bA[256];

#define QEXPSHIFT (7-FRAC_BITS+8) //FIXME try to change this to 0

static inline void put_symbol(RangeCoder *c, uint8_t *state, int v, int is_signed){
    int i;

    if(v){
        const int a= FFABS(v);
        const int e= av_log2(a);
        const int el= FFMIN(e, 10);
        put_rac(c, state+0, 0);

        for(i=0; i<el; i++){
            put_rac(c, state+1+i, 1);  //1..10
        }
        for(; i<e; i++){
            put_rac(c, state+1+9, 1);  //1..10
        }
        put_rac(c, state+1+FFMIN(i,9), 0);

        for(i=e-1; i>=el; i--){
            put_rac(c, state+22+9, (a>>i)&1); //22..31
        }
        for(; i>=0; i--){
            put_rac(c, state+22+i, (a>>i)&1); //22..31
        }

        if(is_signed)
            put_rac(c, state+11 + el, v < 0); //11..21
    }else{
        put_rac(c, state+0, 1);
    }
}

static inline int get_symbol(RangeCoder *c, uint8_t *state, int is_signed){
    if(get_rac(c, state+0))
        return 0;
    else{
        int i, e, a;
        e= 0;
        while(get_rac(c, state+1 + FFMIN(e,9))){ //1..10
            e++;
            if (e > 31)
                return AVERROR_INVALIDDATA;
        }

        a= 1;
        for(i=e-1; i>=0; i--){
            a += a + get_rac(c, state+22 + FFMIN(i,9)); //22..31
        }

        e= -(is_signed && get_rac(c, state+11 + FFMIN(e,10))); //11..21
        return (a^e)-e;
    }
}

static inline void put_symbol2(RangeCoder *c, uint8_t *state, int v, int log2){
    int i;
    int r= log2>=0 ? 1<<log2 : 1;

    av_assert2(v>=0);
    av_assert2(log2>=-4);

    while(v >= r){
        put_rac(c, state+4+log2, 1);
        v -= r;
        log2++;
        if(log2>0) r+=r;
    }
    put_rac(c, state+4+log2, 0);

    for(i=log2-1; i>=0; i--){
        put_rac(c, state+31-i, (v>>i)&1);
    }
}

static inline int get_symbol2(RangeCoder *c, uint8_t *state, int log2){
    int i;
    int r= log2>=0 ? 1<<log2 : 1;
    int v=0;

    av_assert2(log2>=-4);

    while(log2<28 && get_rac(c, state+4+log2)){
        v+= r;
        log2++;
        if(log2>0) r+=r;
    }

    for(i=log2-1; i>=0; i--){
        v+= get_rac(c, state+31-i)<<i;
    }

    return v;
}

static inline void unpack_coeffs(SnowContext *s, SubBand *b, SubBand * parent, int orientation){
    const int w= b->width;
    const int h= b->height;
    int x,y;

    int run, runs;
    x_and_coeff *xc= b->x_coeff;
    x_and_coeff *prev_xc= NULL;
    x_and_coeff *prev2_xc= xc;
    x_and_coeff *parent_xc= parent ? parent->x_coeff : NULL;
    x_and_coeff *prev_parent_xc= parent_xc;

    runs= get_symbol2(&s->c, b->state[30], 0);
    if(runs-- > 0) run= get_symbol2(&s->c, b->state[1], 3);
    else           run= INT_MAX;

    for(y=0; y<h; y++){
        int v=0;
        int lt=0, t=0, rt=0;

        if(y && prev_xc->x == 0){
            rt= prev_xc->coeff;
        }
        for(x=0; x<w; x++){
            int p=0;
            const int l= v;

            lt= t; t= rt;

            if(y){
                if(prev_xc->x <= x)
                    prev_xc++;
                if(prev_xc->x == x + 1)
                    rt= prev_xc->coeff;
                else
                    rt=0;
            }
            if(parent_xc){
                if(x>>1 > parent_xc->x){
                    parent_xc++;
                }
                if(x>>1 == parent_xc->x){
                    p= parent_xc->coeff;
                }
            }
            if(/*ll|*/l|lt|t|rt|p){
                int context= av_log2(/*FFABS(ll) + */3*(l>>1) + (lt>>1) + (t&~1) + (rt>>1) + (p>>1));

                v=get_rac(&s->c, &b->state[0][context]);
                if(v){
                    v= 2*(get_symbol2(&s->c, b->state[context + 2], context-4) + 1);
                    v+=get_rac(&s->c, &b->state[0][16 + 1 + 3 + ff_quant3bA[l&0xFF] + 3*ff_quant3bA[t&0xFF]]);
                    if ((uint16_t)v != v) {
                        av_log(s->avctx, AV_LOG_ERROR, "Coefficient damaged\n");
                        v = 1;
                    }
                    xc->x=x;
                    (xc++)->coeff= v;
                }
            }else{
                if(!run){
                    if(runs-- > 0) run= get_symbol2(&s->c, b->state[1], 3);
                    else           run= INT_MAX;
                    v= 2*(get_symbol2(&s->c, b->state[0 + 2], 0-4) + 1);
                    v+=get_rac(&s->c, &b->state[0][16 + 1 + 3]);
                    if ((uint16_t)v != v) {
                        av_log(s->avctx, AV_LOG_ERROR, "Coefficient damaged\n");
                        v = 1;
                    }

                    xc->x=x;
                    (xc++)->coeff= v;
                }else{
                    int max_run;
                    run--;
                    v=0;
                    av_assert2(run >= 0);
                    if(y) max_run= FFMIN(run, prev_xc->x - x - 2);
                    else  max_run= FFMIN(run, w-x-1);
                    if(parent_xc)
                        max_run= FFMIN(max_run, 2*parent_xc->x - x - 1);
                    av_assert2(max_run >= 0 && max_run <= run);

                    x+= max_run;
                    run-= max_run;
                }
            }
        }
        (xc++)->x= w+1; //end marker
        prev_xc= prev2_xc;
        prev2_xc= xc;

        if(parent_xc){
            if(y&1){
                while(parent_xc->x != parent->width+1)
                    parent_xc++;
                parent_xc++;
                prev_parent_xc= parent_xc;
            }else{
                parent_xc= prev_parent_xc;
            }
        }
    }

    (xc++)->x= w+1; //end marker
}

#endif /* AVCODEC_SNOW_H */
