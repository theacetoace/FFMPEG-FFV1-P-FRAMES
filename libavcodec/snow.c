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

#include "libavutil/intmath.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "me_cmp.h"
#include "snow_dwt.h"
#include "internal.h"
#include "snow.h"

#include "rangecoder.h"
#include "mathops.h"
#include "h263.h"


void ff_snow_inner_add_yblock(const uint8_t *obmc, const int obmc_stride, uint8_t * * block, int b_w, int b_h,
                              int src_x, int src_y, int src_stride, slice_buffer * sb, int add, uint8_t * dst8){
    int y, x;
    IDWTELEM * dst;
    for(y=0; y<b_h; y++){
        //FIXME ugly misuse of obmc_stride
        const uint8_t *obmc1= obmc + y*obmc_stride;
        const uint8_t *obmc2= obmc1+ (obmc_stride>>1);
        const uint8_t *obmc3= obmc1+ obmc_stride*(obmc_stride>>1);
        const uint8_t *obmc4= obmc3+ (obmc_stride>>1);
        dst = slice_buffer_get_line(sb, src_y + y);
        for(x=0; x<b_w; x++){
            int v=   obmc1[x] * block[3][x + y*src_stride]
                    +obmc2[x] * block[2][x + y*src_stride]
                    +obmc3[x] * block[1][x + y*src_stride]
                    +obmc4[x] * block[0][x + y*src_stride];

            v <<= 8 - LOG2_OBMC_MAX;
            if(FRAC_BITS != 8){
                v >>= 8 - FRAC_BITS;
            }
            if(add){
                v += dst[x + src_x];
                v = (v + (1<<(FRAC_BITS-1))) >> FRAC_BITS;
                if(v&(~255)) v= ~(v>>31);
                dst8[x + y*src_stride] = v;
            }else{
                dst[x + src_x] -= v;
            }
        }
    }
}

void ff_snow_reset_contexts(SnowContext *s){ //FIXME better initial contexts
    int plane_index, level, orientation;

    for(plane_index=0; plane_index<3; plane_index++){
        for(level=0; level<MAX_DECOMPOSITIONS; level++){
            for(orientation=level ? 1:0; orientation<4; orientation++){
                memset(
                    s->plane[plane_index].band[level][orientation].state, 
                    MID_STATE, 
                    sizeof(s->plane[plane_index].band[level][orientation].state));
            }
        }
    }
    memset(s->header_state, MID_STATE, sizeof(s->header_state));
    ff_obmc_reset_contexts(&s->obmc);
}

av_cold int ff_snow_common_init(AVCodecContext *avctx){
    SnowContext *s = avctx->priv_data;
    int width, height;

    s->avctx= avctx;
    s->spatial_decomposition_count = 1;

//    dec += FFMAX(s->chroma_h_shift, s->chroma_v_shift);

    width= s->avctx->width;
    height= s->avctx->height;

    FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->spatial_dwt_buffer,  width, height * sizeof(DWTELEM),  fail); //FIXME this does not belong here
    FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->temp_dwt_buffer,     width, sizeof(DWTELEM),  fail);
    FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->temp_idwt_buffer,    width, sizeof(IDWTELEM), fail);
    FF_ALLOC_ARRAY_OR_GOTO(avctx,  s->run_buffer,          ((width + 1) >> 1), ((height + 1) >> 1) * sizeof(*s->run_buffer), fail);
    
    ff_obmc_common_init(&s->obmc, avctx);

    return 0;
fail:
    return AVERROR(ENOMEM);
}

int ff_snow_common_init_after_header(AVCodecContext *avctx) {
    SnowContext *s = avctx->priv_data;
    int plane_index, level, orientation;
    
    ff_obmc_common_init_after_header(&s->obmc);

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        int w= s->avctx->width;
        int h= s->avctx->height;
        
        if(plane_index){
            w = AV_CEIL_RSHIFT(w, s->chroma_h_shift);
            h = AV_CEIL_RSHIFT(h, s->chroma_v_shift);
        }
        s->plane[plane_index].width = w;
        s->plane[plane_index].height= h;

        for(level=s->spatial_decomposition_count-1; level>=0; level--){
            for(orientation=level ? 1 : 0; orientation<4; orientation++){
                SubBand *b= &s->plane[plane_index].band[level][orientation];

                b->buf= s->spatial_dwt_buffer;
                b->level= level;
                b->stride= s->plane[plane_index].width << (s->spatial_decomposition_count - level);
                b->width = (w + !(orientation&1))>>1;
                b->height= (h + !(orientation>1))>>1;

                b->stride_line = 1 << (s->spatial_decomposition_count - level);
                b->buf_x_offset = 0;
                b->buf_y_offset = 0;

                if(orientation&1){
                    b->buf += (w+1)>>1;
                    b->buf_x_offset = (w+1)>>1;
                }
                if(orientation>1){
                    b->buf += b->stride>>1;
                    b->buf_y_offset = b->stride_line >> 1;
                }
                b->ibuf= s->obmc.spatial_idwt_buffer + (b->buf - s->spatial_dwt_buffer);

                if(level)
                    b->parent= &s->plane[plane_index].band[level-1][orientation];
                //FIXME avoid this realloc
                av_freep(&b->x_coeff);
                b->x_coeff=av_mallocz_array(((b->width+1) * b->height+1), sizeof(x_and_coeff));
                if (!b->x_coeff)
                    goto fail;
            }
            w= (w+1)>>1;
            h= (h+1)>>1;
        }
    }

    return 0;
fail:
    return AVERROR(ENOMEM);
}

av_cold void ff_snow_common_end(SnowContext *s)
{
    int plane_index, level, orientation;

    av_freep(&s->spatial_dwt_buffer);
    av_freep(&s->temp_dwt_buffer);
    av_freep(&s->temp_idwt_buffer);
    av_freep(&s->run_buffer);

    for(plane_index=0; plane_index < MAX_PLANES; plane_index++){
        for(level=MAX_DECOMPOSITIONS-1; level>=0; level--){
            for(orientation=level ? 1 : 0; orientation<4; orientation++){
                SubBand *b= &s->plane[plane_index].band[level][orientation];

                av_freep(&b->x_coeff);
            }
        }
    }
    
    ff_obmc_close(&s->obmc);
}
