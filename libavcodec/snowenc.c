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
#include "libavutil/libm.h"
#include "libavutil/log.h"
#include "libavutil/opt.h"
#include "avcodec.h"
#include "internal.h"
#include "snow_dwt.h"
#include "snow.h"

#include "rangecoder.h"
#include "mathops.h"

#include "mpegvideo.h"
#include "h263.h"

#include "obmcenc.h"

#define FF_ME_ITER 50

static av_cold int encode_init(AVCodecContext *avctx)
{
    SnowContext *s = avctx->priv_data;
    int ret;

#if FF_API_PRIVATE_OPT
FF_DISABLE_DEPRECATION_WARNINGS
    if (avctx->prediction_method)
        s->pred = avctx->prediction_method;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    if(s->pred == DWT_97
       && (avctx->flags & AV_CODEC_FLAG_QSCALE)
       && avctx->global_quality == 0){
        av_log(avctx, AV_LOG_ERROR, "The 9/7 wavelet is incompatible with lossless mode.\n");
        return -1;
    }

    s->spatial_decomposition_type= s->pred; //FIXME add decorrelator type r transform_type

    if ((ret = ff_snow_common_init(avctx)) < 0) {
        return ret;
    }

    s->version=0;

    if(avctx->flags&AV_CODEC_FLAG_PASS1){
        if(!avctx->stats_out)
            avctx->stats_out = av_mallocz(256);

        if (!avctx->stats_out)
            return AVERROR(ENOMEM);
    }
    if((avctx->flags&AV_CODEC_FLAG_PASS2) || !(avctx->flags&CODEC_FLAG_QSCALE)){
        if(ff_rate_control_init(&s->obmc.m) < 0)
            return -1;
    }
    s->pass1_rc= !(avctx->flags & (AV_CODEC_FLAG_QSCALE|CODEC_FLAG_PASS2));

    switch(avctx->pix_fmt){
    case AV_PIX_FMT_YUV444P:
//    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUV420P:
//    case AV_PIX_FMT_YUV411P:
    case AV_PIX_FMT_YUV410P:
        s->nb_planes = 3;
        s->colorspace_type= 0;
        break;
    case AV_PIX_FMT_GRAY8:
        s->nb_planes = 1;
        s->colorspace_type = 1;
        break;
/*    case AV_PIX_FMT_RGB32:
        s->colorspace= 1;
        break;*/
    default:
        av_log(avctx, AV_LOG_ERROR, "pixel format not supported\n");
        return -1;
    }
    avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_h_shift, &s->chroma_v_shift);
    
    obmc_encode_init(&s->obmc, avctx);
    s->obmc.c = &s->c;
    s->obmc.put_symbol = put_symbol;

    return 0;
}

static int encode_subband_c0run(SnowContext *s, SubBand *b, const IDWTELEM *src, const IDWTELEM *parent, int stride, int orientation){
    const int w= b->width;
    const int h= b->height;
    int x, y;

    if(1){
        int run=0;
        int *runs = s->run_buffer;
        int run_index=0;
        int max_index;

        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int v, p=0;
                int /*ll=0, */l=0, lt=0, t=0, rt=0;
                v= src[x + y*stride];

                if(y){
                    t= src[x + (y-1)*stride];
                    if(x){
                        lt= src[x - 1 + (y-1)*stride];
                    }
                    if(x + 1 < w){
                        rt= src[x + 1 + (y-1)*stride];
                    }
                }
                if(x){
                    l= src[x - 1 + y*stride];
                    /*if(x > 1){
                        if(orientation==1) ll= src[y + (x-2)*stride];
                        else               ll= src[x - 2 + y*stride];
                    }*/
                }
                if(parent){
                    int px= x>>1;
                    int py= y>>1;
                    if(px<b->parent->width && py<b->parent->height)
                        p= parent[px + py*2*stride];
                }
                if(!(/*ll|*/l|lt|t|rt|p)){
                    if(v){
                        runs[run_index++]= run;
                        run=0;
                    }else{
                        run++;
                    }
                }
            }
        }
        max_index= run_index;
        runs[run_index++]= run;
        run_index=0;
        run= runs[run_index++];

        put_symbol2(&s->c, b->state[30], max_index, 0);
        if(run_index <= max_index)
            put_symbol2(&s->c, b->state[1], run, 3);

        for(y=0; y<h; y++){
            if(s->c.bytestream_end - s->c.bytestream < w*40){
                av_log(s->avctx, AV_LOG_ERROR, "encoded frame too large\n");
                return -1;
            }
            for(x=0; x<w; x++){
                int v, p=0;
                int /*ll=0, */l=0, lt=0, t=0, rt=0;
                v= src[x + y*stride];

                if(y){
                    t= src[x + (y-1)*stride];
                    if(x){
                        lt= src[x - 1 + (y-1)*stride];
                    }
                    if(x + 1 < w){
                        rt= src[x + 1 + (y-1)*stride];
                    }
                }
                if(x){
                    l= src[x - 1 + y*stride];
                    /*if(x > 1){
                        if(orientation==1) ll= src[y + (x-2)*stride];
                        else               ll= src[x - 2 + y*stride];
                    }*/
                }
                if(parent){
                    int px= x>>1;
                    int py= y>>1;
                    if(px<b->parent->width && py<b->parent->height)
                        p= parent[px + py*2*stride];
                }
                if(/*ll|*/l|lt|t|rt|p){
                    int context= av_log2(/*FFABS(ll) + */3*FFABS(l) + FFABS(lt) + 2*FFABS(t) + FFABS(rt) + FFABS(p));

                    put_rac(&s->c, &b->state[0][context], !!v);
                }else{
                    if(!run){
                        run= runs[run_index++];

                        if(run_index <= max_index)
                            put_symbol2(&s->c, b->state[1], run, 3);
                        av_assert2(v);
                    }else{
                        run--;
                        av_assert2(!v);
                    }
                }
                if(v){
                    int context= av_log2(/*FFABS(ll) + */3*FFABS(l) + FFABS(lt) + 2*FFABS(t) + FFABS(rt) + FFABS(p));
                    int l2= 2*FFABS(l) + (l<0);
                    int t2= 2*FFABS(t) + (t<0);

                    put_symbol2(&s->c, b->state[context + 2], FFABS(v)-1, context-4);
                    put_rac(&s->c, &b->state[0][16 + 1 + 3 + ff_quant3bA[l2&0xFF] + 3*ff_quant3bA[t2&0xFF]], v<0);
                }
            }
        }
    }
    return 0;
}

static int encode_subband(SnowContext *s, SubBand *b, const IDWTELEM *src, const IDWTELEM *parent, int stride, int orientation){
//    encode_subband_qtree(s, b, src, parent, stride, orientation);
//    encode_subband_z0run(s, b, src, parent, stride, orientation);
    return encode_subband_c0run(s, b, src, parent, stride, orientation);
//    encode_subband_dzr(s, b, src, parent, stride, orientation);
}

static void quantize(SnowContext *s, SubBand *b, IDWTELEM *dst, DWTELEM *src, int stride, int bias){
    const int w= b->width;
    const int h= b->height;
    const int qlog= av_clip(s->qlog + b->qlog, 0, QROOT*16);
    const int qmul= ff_qexp[qlog&(QROOT-1)]<<((qlog>>QSHIFT) + ENCODER_EXTRA_BITS);
    int x,y, thres1, thres2;

    if(s->qlog == LOSSLESS_QLOG){
        for(y=0; y<h; y++)
            for(x=0; x<w; x++)
                dst[x + y*stride]= src[x + y*stride];
        return;
    }

    bias= bias ? 0 : (3*qmul)>>3;
    thres1= ((qmul - bias)>>QEXPSHIFT) - 1;
    thres2= 2*thres1;

    if(!bias){
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= src[x + y*stride];

                if((unsigned)(i+thres1) > thres2){
                    if(i>=0){
                        i<<= QEXPSHIFT;
                        i/= qmul; //FIXME optimize
                        dst[x + y*stride]=  i;
                    }else{
                        i= -i;
                        i<<= QEXPSHIFT;
                        i/= qmul; //FIXME optimize
                        dst[x + y*stride]= -i;
                    }
                }else
                    dst[x + y*stride]= 0;
            }
        }
    }else{
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= src[x + y*stride];

                if((unsigned)(i+thres1) > thres2){
                    if(i>=0){
                        i<<= QEXPSHIFT;
                        i= (i + bias) / qmul; //FIXME optimize
                        dst[x + y*stride]=  i;
                    }else{
                        i= -i;
                        i<<= QEXPSHIFT;
                        i= (i + bias) / qmul; //FIXME optimize
                        dst[x + y*stride]= -i;
                    }
                }else
                    dst[x + y*stride]= 0;
            }
        }
    }
}

static void dequantize(SnowContext *s, SubBand *b, IDWTELEM *src, int stride){
    const int w= b->width;
    const int h= b->height;
    const int qlog= av_clip(s->qlog + b->qlog, 0, QROOT*16);
    const int qmul= ff_qexp[qlog&(QROOT-1)]<<(qlog>>QSHIFT);
    const int qadd= (s->qbias*qmul)>>QBIAS_SHIFT;
    int x,y;

    if(s->qlog == LOSSLESS_QLOG) return;

    for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            int i= src[x + y*stride];
            if(i<0){
                src[x + y*stride]= -((-i*qmul + qadd)>>(QEXPSHIFT)); //FIXME try different bias
            }else if(i>0){
                src[x + y*stride]=  (( i*qmul + qadd)>>(QEXPSHIFT));
            }
        }
    }
}

static void decorrelate(SnowContext *s, SubBand *b, IDWTELEM *src, int stride, int inverse, int use_median){
    const int w= b->width;
    const int h= b->height;
    int x,y;

    for(y=h-1; y>=0; y--){
        for(x=w-1; x>=0; x--){
            int i= x + y*stride;

            if(x){
                if(use_median){
                    if(y && x+1<w) src[i] -= mid_pred(src[i - 1], src[i - stride], src[i - stride + 1]);
                    else  src[i] -= src[i - 1];
                }else{
                    if(y) src[i] -= mid_pred(src[i - 1], src[i - stride], src[i - 1] + src[i - stride] - src[i - 1 - stride]);
                    else  src[i] -= src[i - 1];
                }
            }else{
                if(y) src[i] -= src[i - stride];
            }
        }
    }
}

static void correlate(SnowContext *s, SubBand *b, IDWTELEM *src, int stride, int inverse, int use_median){
    const int w= b->width;
    const int h= b->height;
    int x,y;

    for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            int i= x + y*stride;

            if(x){
                if(use_median){
                    if(y && x+1<w) src[i] += mid_pred(src[i - 1], src[i - stride], src[i - stride + 1]);
                    else  src[i] += src[i - 1];
                }else{
                    if(y) src[i] += mid_pred(src[i - 1], src[i - stride], src[i - 1] + src[i - stride] - src[i - 1 - stride]);
                    else  src[i] += src[i - 1];
                }
            }else{
                if(y) src[i] += src[i - stride];
            }
        }
    }
}

static void encode_qlogs(SnowContext *s){
    int plane_index, level, orientation;

    for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
        for(level=0; level<s->spatial_decomposition_count; level++){
            for(orientation=level ? 1:0; orientation<4; orientation++){
                if(orientation==2) continue;
                put_symbol(&s->c, s->header_state, s->plane[plane_index].band[level][orientation].qlog, 1);
            }
        }
    }
}

static void encode_header(SnowContext *s){
    int plane_index, i;
    uint8_t kstate[32];

    memset(kstate, MID_STATE, sizeof(kstate));

    put_rac(&s->c, kstate, s->keyframe);
    if(s->keyframe || s->always_reset){
        ff_snow_reset_contexts(s);
        s->last_spatial_decomposition_type=
        s->last_qlog=
        s->last_qbias=
        s->obmc.last_mv_scale=
        s->obmc.last_block_max_depth= 0;
        for(plane_index=0; plane_index<2; plane_index++){
            PlaneObmc *p= &s->obmc.plane[plane_index];
            p->last_htaps=0;
            p->last_diag_mc=0;
            memset(p->last_hcoeff, 0, sizeof(p->last_hcoeff));
        }
    }
    if(s->keyframe){
        put_symbol(&s->c, s->header_state, s->version, 0);
        put_rac(&s->c, s->header_state, s->always_reset);
        put_symbol(&s->c, s->header_state, s->temporal_decomposition_type, 0);
        put_symbol(&s->c, s->header_state, s->temporal_decomposition_count, 0);
        put_symbol(&s->c, s->header_state, s->spatial_decomposition_count, 0);
        put_symbol(&s->c, s->header_state, s->colorspace_type, 0);
        if (s->nb_planes > 2) {
            put_symbol(&s->c, s->header_state, s->chroma_h_shift, 0);
            put_symbol(&s->c, s->header_state, s->chroma_v_shift, 0);
        }
        put_rac(&s->c, s->header_state, s->spatial_scalability);
//        put_rac(&s->c, s->header_state, s->rate_scalability);
        put_symbol(&s->c, s->header_state, s->obmc.max_ref_frames-1, 0);

        encode_qlogs(s);
    }

    if(!s->keyframe){
        int update_mc=0;
        for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
            PlaneObmc *p= &s->obmc.plane[plane_index];
            update_mc |= p->last_htaps   != p->htaps;
            update_mc |= p->last_diag_mc != p->diag_mc;
            update_mc |= !!memcmp(p->last_hcoeff, p->hcoeff, sizeof(p->hcoeff));
        }
        put_rac(&s->c, s->header_state, update_mc);
        if(update_mc){
            for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
                PlaneObmc *p= &s->obmc.plane[plane_index];
                put_rac(&s->c, s->header_state, p->diag_mc);
                put_symbol(&s->c, s->header_state, p->htaps/2-1, 0);
                for(i= p->htaps/2; i; i--)
                    put_symbol(&s->c, s->header_state, FFABS(p->hcoeff[i]), 0);
            }
        }
        if(s->last_spatial_decomposition_count != s->spatial_decomposition_count){
            put_rac(&s->c, s->header_state, 1);
            put_symbol(&s->c, s->header_state, s->spatial_decomposition_count, 0);
            encode_qlogs(s);
        }else
            put_rac(&s->c, s->header_state, 0);
    }

    put_symbol(&s->c, s->header_state, s->spatial_decomposition_type - s->last_spatial_decomposition_type, 1);
    put_symbol(&s->c, s->header_state, s->qlog                 - s->last_qlog    , 1);
    put_symbol(&s->c, s->header_state, s->obmc.mv_scale        - s->obmc.last_mv_scale, 1);
    put_symbol(&s->c, s->header_state, s->qbias                - s->last_qbias   , 1);
    put_symbol(&s->c, s->header_state, s->obmc.block_max_depth - s->obmc.last_block_max_depth, 1);

}

static void update_last_header_values(SnowContext *s){
    int plane_index;

    if(!s->keyframe){
        for(plane_index=0; plane_index<2; plane_index++){
            PlaneObmc *p= &s->obmc.plane[plane_index];
            p->last_diag_mc= p->diag_mc;
            p->last_htaps  = p->htaps;
            memcpy(p->last_hcoeff, p->hcoeff, sizeof(p->hcoeff));
        }
    }

    s->last_spatial_decomposition_type  = s->spatial_decomposition_type;
    s->last_qlog                        = s->qlog;
    s->last_qbias                       = s->qbias;
    s->obmc.last_mv_scale               = s->obmc.mv_scale;
    s->obmc.last_block_max_depth        = s->obmc.block_max_depth;
    s->last_spatial_decomposition_count = s->spatial_decomposition_count;
}

static int qscale2qlog(int qscale){
    return lrint(QROOT*log2(qscale / (float)FF_QP2LAMBDA))
           + 61*QROOT/8; ///< 64 > 60
}

static int ratecontrol_1pass(SnowContext *s, AVFrame *pict)
{
    /* Estimate the frame's complexity as a sum of weighted dwt coefficients.
     * FIXME we know exact mv bits at this point,
     * but ratecontrol isn't set up to include them. */
    uint32_t coef_sum= 0;
    int level, orientation, delta_qlog;

    for(level=0; level<s->spatial_decomposition_count; level++){
        for(orientation=level ? 1 : 0; orientation<4; orientation++){
            SubBand *b= &s->plane[0].band[level][orientation];
            IDWTELEM *buf= b->ibuf;
            const int w= b->width;
            const int h= b->height;
            const int stride= b->stride;
            const int qlog= av_clip(2*QROOT + b->qlog, 0, QROOT*16);
            const int qmul= ff_qexp[qlog&(QROOT-1)]<<(qlog>>QSHIFT);
            const int qdiv= (1<<16)/qmul;
            int x, y;
            //FIXME this is ugly
            for(y=0; y<h; y++)
                for(x=0; x<w; x++)
                    buf[x+y*stride]= b->buf[x+y*stride];
            if(orientation==0)
                decorrelate(s, b, buf, stride, 1, 0);
            for(y=0; y<h; y++)
                for(x=0; x<w; x++)
                    coef_sum+= abs(buf[x+y*stride]) * qdiv >> 16;
        }
    }

    /* ugly, ratecontrol just takes a sqrt again */
    av_assert0(coef_sum < INT_MAX);
    coef_sum = (uint64_t)coef_sum * coef_sum >> 16;

    if(pict->pict_type == AV_PICTURE_TYPE_I){
        s->obmc.m.current_picture.mb_var_sum= coef_sum;
        s->obmc.m.current_picture.mc_mb_var_sum= 0;
    }else{
        s->obmc.m.current_picture.mc_mb_var_sum= coef_sum;
        s->obmc.m.current_picture.mb_var_sum= 0;
    }

    pict->quality= ff_rate_estimate_qscale(&s->obmc.m, 1);
    if (pict->quality < 0)
        return INT_MIN;
    s->obmc.lambda= pict->quality * 3/2;
    delta_qlog= qscale2qlog(pict->quality) - s->qlog;
    s->qlog+= delta_qlog;
    return delta_qlog;
}

static void calculate_visual_weight(SnowContext *s, Plane *p){
    int width = p->width;
    int height= p->height;
    int level, orientation, x, y;

    for(level=0; level<s->spatial_decomposition_count; level++){
        for(orientation=level ? 1 : 0; orientation<4; orientation++){
            SubBand *b= &p->band[level][orientation];
            IDWTELEM *ibuf= b->ibuf;
            int64_t error=0;

            memset(s->obmc.spatial_idwt_buffer, 0, sizeof(*s->obmc.spatial_idwt_buffer)*width*height);
            ibuf[b->width/2 + b->height/2*b->stride]= 256*16;
            ff_spatial_idwt(s->obmc.spatial_idwt_buffer, s->temp_idwt_buffer, width, height, width, s->spatial_decomposition_type, s->spatial_decomposition_count);
            for(y=0; y<height; y++){
                for(x=0; x<width; x++){
                    int64_t d= s->obmc.spatial_idwt_buffer[x + y*width]*16;
                    error += d*d;
                }
            }

            b->qlog= (int)(QROOT * log2(352256.0/sqrt(error)) + 0.5);
        }
    }
}

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *pict, int *got_packet)
{
    SnowContext *s = avctx->priv_data;
    RangeCoder * const c= &s->c;
    AVFrame *pic;
    const int width= s->avctx->width;
    const int height= s->avctx->height;
    int level, orientation, plane_index, i, y, ret;
    uint8_t rc_header_bak[sizeof(s->header_state)];
    uint8_t rc_block_bak[sizeof(s->obmc.block_state)];

    if ((ret = ff_alloc_packet2(avctx, pkt, s->obmc.b_width*s->obmc.b_height*MB_SIZE*MB_SIZE*3 + AV_INPUT_BUFFER_MIN_SIZE, 0)) < 0)
        return ret;

    ff_init_range_encoder(c, pkt->data, pkt->size);
    ff_build_rac_states(c, (1LL<<32)/20, 256-8);

    for(i=0; i < s->nb_planes; i++){
        int hshift= i ? s->chroma_h_shift : 0;
        int vshift= i ? s->chroma_v_shift : 0;
        for(y=0; y<AV_CEIL_RSHIFT(height, vshift); y++)
            memcpy(&s->obmc.input_picture->data[i][y * s->obmc.input_picture->linesize[i]],
                   &pict->data[i][y * pict->linesize[i]],
                   AV_CEIL_RSHIFT(width, hshift));
        s->obmc.mpvencdsp.draw_edges(s->obmc.input_picture->data[i], s->obmc.input_picture->linesize[i],
                                AV_CEIL_RSHIFT(width, hshift), AV_CEIL_RSHIFT(height, vshift),
                                EDGE_WIDTH >> hshift, EDGE_WIDTH >> vshift,
                                EDGE_TOP | EDGE_BOTTOM);

    }
    emms_c();
    pic = s->obmc.input_picture;
    pic->pict_type = pict->pict_type;
    pic->quality = pict->quality;

    s->obmc.m.picture_number= avctx->frame_number;
    if(avctx->flags&AV_CODEC_FLAG_PASS2){
        s->obmc.m.pict_type = pic->pict_type = s->obmc.m.rc_context.entry[avctx->frame_number].new_pict_type;
        s->keyframe = pic->pict_type == AV_PICTURE_TYPE_I;
        s->obmc.key_frame = s->keyframe;
        if(!(avctx->flags&AV_CODEC_FLAG_QSCALE)) {
            pic->quality = ff_rate_estimate_qscale(&s->obmc.m, 0);
            if (pic->quality < 0)
                return -1;
        }
    }else{
        s->keyframe= avctx->gop_size==0 || avctx->frame_number % avctx->gop_size == 0;
        s->obmc.m.pict_type = pic->pict_type = s->keyframe ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;
        s->obmc.key_frame = s->keyframe;
    }

    if(s->pass1_rc && avctx->frame_number == 0)
        pic->quality = 2*FF_QP2LAMBDA;
    if (pic->quality) {
        s->qlog   = qscale2qlog(pic->quality);
        s->obmc.lambda = pic->quality * 3/2;
    }
    if (s->qlog < 0 || (!pic->quality && (avctx->flags & AV_CODEC_FLAG_QSCALE))) {
        s->qlog= LOSSLESS_QLOG;
        s->obmc.lambda = 0;
    }//else keep previous frame's qlog until after motion estimation

    if(s->pass1_rc){
        memcpy(rc_header_bak, s->header_state, sizeof(s->header_state));
        memcpy(rc_block_bak, s->obmc.block_state, sizeof(s->obmc.block_state));
    }
    
    obmc_pre_encode_frame(&s->obmc, avctx, pict);

redo_frame:

    s->spatial_decomposition_count= 5;

    while(   !(width >>(s->chroma_h_shift + s->spatial_decomposition_count))
          || !(height>>(s->chroma_v_shift + s->spatial_decomposition_count)))
        s->spatial_decomposition_count--;

    if (s->spatial_decomposition_count <= 0) {
        av_log(avctx, AV_LOG_ERROR, "Resolution too low\n");
        return AVERROR(EINVAL);
    }

    s->obmc.m.pict_type = pic->pict_type;
    s->qbias = pic->pict_type == AV_PICTURE_TYPE_P ? 2 : 0;

    ff_snow_common_init_after_header(avctx);

    if(s->last_spatial_decomposition_count != s->spatial_decomposition_count){
        for(plane_index=0; plane_index < s->nb_planes; plane_index++){
            calculate_visual_weight(s, &s->plane[plane_index]);
        }
    }

    encode_header(s);
    s->obmc.m.misc_bits = 8*(s->c.bytestream - s->c.bytestream_start);
    obmc_encode_blocks(&s->obmc, 1);
    s->obmc.m.mv_bits = 8*(s->c.bytestream - s->c.bytestream_start) - s->obmc.m.misc_bits;

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        Plane *p= &s->plane[plane_index];
        int w= p->width;
        int h= p->height;
        int x, y;
//        int bits= put_bits_count(&s->c.pb);

        if (!s->memc_only) {
            //FIXME optimize
            if(pict->data[plane_index]) //FIXME gray hack
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        s->obmc.spatial_idwt_buffer[y*w + x]= pict->data[plane_index][y*pict->linesize[plane_index] + x]<<FRAC_BITS;
                    }
                }
            predict_plane(&s->obmc, s->obmc.spatial_idwt_buffer, plane_index, 0);

#if FF_API_PRIVATE_OPT
FF_DISABLE_DEPRECATION_WARNINGS
            if(s->avctx->scenechange_threshold)
                s->scenechange_threshold = s->avctx->scenechange_threshold;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

            if(   plane_index==0
               && pic->pict_type == AV_PICTURE_TYPE_P
               && !(avctx->flags&AV_CODEC_FLAG_PASS2)
               && s->obmc.m.me.scene_change_score > s->scenechange_threshold){
                ff_init_range_encoder(c, pkt->data, pkt->size);
                ff_build_rac_states(c, (1LL<<32)/20, 256-8);
                pic->pict_type= AV_PICTURE_TYPE_I;
                s->keyframe=1;
                s->obmc.key_frame = 1;
                s->obmc.current_picture->key_frame=1;
                goto redo_frame;
            }

            if(s->qlog == LOSSLESS_QLOG){
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        s->spatial_dwt_buffer[y*w + x]= (s->obmc.spatial_idwt_buffer[y*w + x] + (1<<(FRAC_BITS-1))-1)>>FRAC_BITS;
                    }
                }
            }else{
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        s->spatial_dwt_buffer[y*w + x]=s->obmc.spatial_idwt_buffer[y*w + x]<<ENCODER_EXTRA_BITS;
                    }
                }
            }

            ff_spatial_dwt(s->spatial_dwt_buffer, s->temp_dwt_buffer, w, h, w, s->spatial_decomposition_type, s->spatial_decomposition_count);

            if(s->pass1_rc && plane_index==0){
                int delta_qlog = ratecontrol_1pass(s, pic);
                if (delta_qlog <= INT_MIN)
                    return -1;
                if(delta_qlog){
                    //reordering qlog in the bitstream would eliminate this reset
                    ff_init_range_encoder(c, pkt->data, pkt->size);
                    memcpy(s->header_state, rc_header_bak, sizeof(s->header_state));
                    memcpy(s->obmc.block_state, rc_block_bak, sizeof(s->obmc.block_state));
                    encode_header(s);
                    obmc_encode_blocks(&s->obmc, 0);
                }
            }

            for(level=0; level<s->spatial_decomposition_count; level++){
                for(orientation=level ? 1 : 0; orientation<4; orientation++){
                    SubBand *b= &p->band[level][orientation];

                    quantize(s, b, b->ibuf, b->buf, b->stride, s->qbias);
                    if(orientation==0)
                        decorrelate(s, b, b->ibuf, b->stride, pic->pict_type == AV_PICTURE_TYPE_P, 0);
                    if (!s->no_bitstream)
                    encode_subband(s, b, b->ibuf, b->parent ? b->parent->ibuf : NULL, b->stride, orientation);
                    av_assert0(b->parent==NULL || b->parent->stride == b->stride*2);
                    if(orientation==0)
                        correlate(s, b, b->ibuf, b->stride, 1, 0);
                }
            }

            for(level=0; level<s->spatial_decomposition_count; level++){
                for(orientation=level ? 1 : 0; orientation<4; orientation++){
                    SubBand *b= &p->band[level][orientation];

                    dequantize(s, b, b->ibuf, b->stride);
                }
            }

            ff_spatial_idwt(s->obmc.spatial_idwt_buffer, s->temp_idwt_buffer, w, h, w, s->spatial_decomposition_type, s->spatial_decomposition_count);
            if(s->qlog == LOSSLESS_QLOG){
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        s->obmc.spatial_idwt_buffer[y*w + x]<<=FRAC_BITS;
                    }
                }
            }
            predict_plane(&s->obmc, s->obmc.spatial_idwt_buffer, plane_index, 1);
        }else{
            //ME/MC only
            if(pic->pict_type == AV_PICTURE_TYPE_I){
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        s->obmc.current_picture->data[plane_index][y*s->obmc.current_picture->linesize[plane_index] + x]=
                            pict->data[plane_index][y*pict->linesize[plane_index] + x];
                    }
                }
            }else{
                memset(s->obmc.spatial_idwt_buffer, 0, sizeof(IDWTELEM)*w*h);
                predict_plane(&s->obmc, s->obmc.spatial_idwt_buffer, plane_index, 1);
            }
        }
        if(s->avctx->flags&AV_CODEC_FLAG_PSNR){
            int64_t error= 0;

            if(pict->data[plane_index]) //FIXME gray hack
                for(y=0; y<h; y++){
                    for(x=0; x<w; x++){
                        int d= s->obmc.current_picture->data[plane_index][y*s->obmc.current_picture->linesize[plane_index] + x] - pict->data[plane_index][y*pict->linesize[plane_index] + x];
                        error += d*d;
                    }
                }
            s->avctx->error[plane_index] += error;
            s->encoding_error[plane_index] = error;
        }

    }

    update_last_header_values(s);

    ff_obmc_release_buffer(&s->obmc);

    s->obmc.current_picture->coded_picture_number = avctx->frame_number;
    s->obmc.current_picture->pict_type = pic->pict_type;
    s->obmc.current_picture->quality = pic->quality;
    s->obmc.m.frame_bits = 8*(s->c.bytestream - s->c.bytestream_start);
    s->obmc.m.p_tex_bits = s->obmc.m.frame_bits - s->obmc.m.misc_bits - s->obmc.m.mv_bits;
    s->obmc.m.current_picture.f->display_picture_number =
    s->obmc.m.current_picture.f->coded_picture_number   = avctx->frame_number;
    s->obmc.m.current_picture.f->quality                = pic->quality;
    s->obmc.m.total_bits += 8*(s->c.bytestream - s->c.bytestream_start);
    if(s->pass1_rc)
        if (ff_rate_estimate_qscale(&s->obmc.m, 0) < 0)
            return -1;
    if(avctx->flags&AV_CODEC_FLAG_PASS1)
        ff_write_pass1_stats(&s->obmc.m);
    s->obmc.m.last_pict_type = s->obmc.m.pict_type;
    avctx->frame_bits = s->obmc.m.frame_bits;
    avctx->mv_bits = s->obmc.m.mv_bits;
    avctx->misc_bits = s->obmc.m.misc_bits;
    avctx->p_tex_bits = s->obmc.m.p_tex_bits;

    emms_c();

    ff_side_data_set_encoder_stats(pkt, s->obmc.current_picture->quality,
                                   s->encoding_error,
                                   (s->avctx->flags&AV_CODEC_FLAG_PSNR) ? 4 : 0,
                                   s->obmc.current_picture->pict_type);

#if FF_API_ERROR_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
    memcpy(s->obmc.current_picture->error, s->encoding_error, sizeof(s->encoding_error));
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    pkt->size = ff_rac_terminate(c);
    if (s->obmc.current_picture->key_frame)
        pkt->flags |= AV_PKT_FLAG_KEY;
    *got_packet = 1;

    return 0;
}

static av_cold int encode_end(AVCodecContext *avctx)
{
    SnowContext *s = avctx->priv_data;

    ff_snow_common_end(s);
    ff_rate_control_uninit(&s->obmc.m);
    av_frame_free(&s->obmc.input_picture);
    av_freep(&avctx->stats_out);

    return 0;
}

#define OFFSET(x) offsetof(SnowContext, x)
#define OFFSET_OBMC(x) offsetof(SnowContext, obmc) + offsetof(OBMCContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    FF_MPV_COMMON_OPTS
    { "iter",           NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ITER }, 0, 0, FF_MPV_OPT_FLAGS, "motion_est" },
    { "memc_only",      "Only do ME/MC (I frames -> ref, P frame -> ME+MC).",   OFFSET(memc_only), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    { "no_bitstream",   "Skip final bitstream writeout.",                    OFFSET(no_bitstream), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    { "intra_penalty",  "Penalty for intra blocks in block decission", OFFSET_OBMC(intra_penalty), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, VE },
    { "iterative_dia_size",  "Dia size for the iterative ME",     OFFSET_OBMC(iterative_dia_size), AV_OPT_TYPE_INT, { .i64 = 0 }, 0, INT_MAX, VE },
    { "sc_threshold",   "Scene change threshold",                   OFFSET(scenechange_threshold), AV_OPT_TYPE_INT, { .i64 = 0 }, INT_MIN, INT_MAX, VE },
    { "pred",           "Spatial decomposition type",                                OFFSET(pred), AV_OPT_TYPE_INT, { .i64 = 0 }, DWT_97, DWT_53, VE, "pred" },
        { "dwt97", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 }, INT_MIN, INT_MAX, VE, "pred" },
        { "dwt53", NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 }, INT_MIN, INT_MAX, VE, "pred" },
    { NULL },
};

static const AVClass snowenc_class = {
    .class_name = "snow encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_snow_encoder = {
    .name           = "snow",
    .long_name      = NULL_IF_CONFIG_SMALL("Snow"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_SNOW,
    .priv_data_size = sizeof(SnowContext),
    .init           = encode_init,
    .encode2        = encode_frame,
    .close          = encode_end,
    .pix_fmts       = (const enum AVPixelFormat[]){
        AV_PIX_FMT_YUV420P, AV_PIX_FMT_YUV410P, AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_GRAY8,
        AV_PIX_FMT_NONE
    },
    .priv_class     = &snowenc_class,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
};

