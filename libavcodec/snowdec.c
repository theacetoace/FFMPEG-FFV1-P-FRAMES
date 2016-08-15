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
#include "snow_dwt.h"
#include "internal.h"
#include "snow.h"

#include "rangecoder.h"
#include "mathops.h"

#include "mpegvideo.h"
#include "h263.h"

#include "obmcdec.h"

static int  get_level_break (ObmcCoderContext *c, int ctx)
{
    SnowContext *f = (SnowContext *)c->avctx->priv_data;
    return get_rac(&f->c, &f->block_state[ctx]);
}

static int  get_block_type  (ObmcCoderContext *c, int ctx)
{
    SnowContext *f = (SnowContext *)c->avctx->priv_data;
    return get_rac(&f->c, &f->block_state[ctx]);
}

static void get_block_color (ObmcCoderContext *c, int ctx_l, int ctx_cb, int ctx_cr, int *l, int *cb, int *cr)
{
    SnowContext *f = (SnowContext *)c->avctx->priv_data;
    *l += get_symbol(&f->c, &f->block_state[ctx_l], 1);
    if (f->obmc.nb_planes > 2) {
        *cb += get_symbol(&f->c, &f->block_state[ctx_cb], 1);
        *cr += get_symbol(&f->c, &f->block_state[ctx_cr], 1);
    }
}

static int  get_best_ref    (ObmcCoderContext *c, int ctx)
{
    SnowContext *f = (SnowContext *)c->avctx->priv_data;
    return get_symbol(&f->c, &f->block_state[ctx], 0);
}

static void get_block_mv    (ObmcCoderContext *c, int ctx_mx, int ctx_my, int *mx, int *my)
{
    SnowContext *f = (SnowContext *)c->avctx->priv_data;
    *mx += get_symbol(&f->c, &f->block_state[ctx_mx], 1);
    *my += get_symbol(&f->c, &f->block_state[ctx_my], 1);
}

static void ff_snow_init_decode_callbacks(ObmcCoderContext *c, AVCodecContext *avctx)
{
    c->avctx = avctx;
    c->get_level_break = get_level_break;
    c->get_block_type  = get_block_type;
    c->get_block_color = get_block_color;
    c->get_best_ref    = get_best_ref;
    c->get_block_mv    = get_block_mv;
}


static inline void decode_subband_slice_buffered(SnowContext *s, SubBand *b, slice_buffer * sb, int start_y, int h, int save_state[1]){
    const int w= b->width;
    int y;
    const int qlog= av_clip(s->qlog + b->qlog, 0, QROOT*16);
    int qmul= ff_qexp[qlog&(QROOT-1)]<<(qlog>>QSHIFT);
    int qadd= (s->qbias*qmul)>>QBIAS_SHIFT;
    int new_index = 0;

    if(b->ibuf == s->obmc.spatial_idwt_buffer || s->qlog == LOSSLESS_QLOG){
        qadd= 0;
        qmul= 1<<QEXPSHIFT;
    }

    /* If we are on the second or later slice, restore our index. */
    if (start_y != 0)
        new_index = save_state[0];


    for(y=start_y; y<h; y++){
        int x = 0;
        int v;
        IDWTELEM * line = slice_buffer_get_line(sb, y * b->stride_line + b->buf_y_offset) + b->buf_x_offset;
        memset(line, 0, b->width*sizeof(IDWTELEM));
        v = b->x_coeff[new_index].coeff;
        x = b->x_coeff[new_index++].x;
        while(x < w){
            register int t= ( (v>>1)*qmul + qadd)>>QEXPSHIFT;
            register int u= -(v&1);
            line[x] = (t^u) - u;

            v = b->x_coeff[new_index].coeff;
            x = b->x_coeff[new_index++].x;
        }
    }

    /* Save our variables for the next slice. */
    save_state[0] = new_index;

    return;
}

static void dequantize_slice_buffered(SnowContext *s, slice_buffer * sb, SubBand *b, IDWTELEM *src, int stride, int start_y, int end_y){
    const int w= b->width;
    const int qlog= av_clip(s->qlog + b->qlog, 0, QROOT*16);
    const int qmul= ff_qexp[qlog&(QROOT-1)]<<(qlog>>QSHIFT);
    const int qadd= (s->qbias*qmul)>>QBIAS_SHIFT;
    int x,y;

    if(s->qlog == LOSSLESS_QLOG) return;

    for(y=start_y; y<end_y; y++){
//        DWTELEM * line = slice_buffer_get_line_from_address(sb, src + (y * stride));
        IDWTELEM * line = slice_buffer_get_line(sb, (y * b->stride_line) + b->buf_y_offset) + b->buf_x_offset;
        for(x=0; x<w; x++){
            int i= line[x];
            if(i<0){
                line[x]= -((-i*qmul + qadd)>>(QEXPSHIFT)); //FIXME try different bias
            }else if(i>0){
                line[x]=  (( i*qmul + qadd)>>(QEXPSHIFT));
            }
        }
    }
}

static void correlate_slice_buffered(SnowContext *s, slice_buffer * sb, SubBand *b, IDWTELEM *src, int stride, int inverse, int use_median, int start_y, int end_y){
    const int w= b->width;
    int x,y;

    IDWTELEM * line=0; // silence silly "could be used without having been initialized" warning
    IDWTELEM * prev;

    if (start_y != 0)
        line = slice_buffer_get_line(sb, ((start_y - 1) * b->stride_line) + b->buf_y_offset) + b->buf_x_offset;

    for(y=start_y; y<end_y; y++){
        prev = line;
//        line = slice_buffer_get_line_from_address(sb, src + (y * stride));
        line = slice_buffer_get_line(sb, (y * b->stride_line) + b->buf_y_offset) + b->buf_x_offset;
        for(x=0; x<w; x++){
            if(x){
                if(use_median){
                    if(y && x+1<w) line[x] += mid_pred(line[x - 1], prev[x], prev[x + 1]);
                    else  line[x] += line[x - 1];
                }else{
                    if(y) line[x] += mid_pred(line[x - 1], prev[x], line[x - 1] + prev[x] - prev[x - 1]);
                    else  line[x] += line[x - 1];
                }
            }else{
                if(y) line[x] += prev[x];
            }
        }
    }
}

static void decode_qlogs(SnowContext *s){
    int plane_index, level, orientation;

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        for(level=0; level<s->spatial_decomposition_count; level++){
            for(orientation=level ? 1:0; orientation<4; orientation++){
                int q;
                if     (plane_index==2) q= s->plane[1].band[level][orientation].qlog;
                else if(orientation==2) q= s->plane[plane_index].band[level][1].qlog;
                else                    q= get_symbol(&s->c, s->header_state, 1);
                s->plane[plane_index].band[level][orientation].qlog= q;
            }
        }
    }
}

#define GET_S(dst, check) \
    tmp= get_symbol(&s->c, s->header_state, 0);\
    if(!(check)){\
        av_log(s->avctx, AV_LOG_ERROR, "Error " #dst " is %d\n", tmp);\
        return AVERROR_INVALIDDATA;\
    }\
    dst= tmp;

static int decode_header(SnowContext *s){
    int plane_index, tmp;
    uint8_t kstate[32];

    memset(kstate, MID_STATE, sizeof(kstate));

    s->keyframe= get_rac(&s->c, kstate);
    s->obmc.key_frame = s->keyframe;
    if(s->keyframe || s->always_reset){
        ff_snow_reset_contexts(s);
        s->spatial_decomposition_type=
        s->qlog=
        s->qbias=
        s->obmc.mv_scale=
        s->obmc.block_max_depth= 0;
    }
    if(s->keyframe){
        GET_S(s->version, tmp <= 0U)
        s->always_reset= get_rac(&s->c, s->header_state);
        s->temporal_decomposition_type= get_symbol(&s->c, s->header_state, 0);
        s->temporal_decomposition_count= get_symbol(&s->c, s->header_state, 0);
        GET_S(s->spatial_decomposition_count, 0 < tmp && tmp <= MAX_DECOMPOSITIONS)
        s->colorspace_type= get_symbol(&s->c, s->header_state, 0);
        if (s->colorspace_type == 1) {
            s->avctx->pix_fmt= AV_PIX_FMT_GRAY8;
            s->nb_planes = 1;
        } else if(s->colorspace_type == 0) {
            s->chroma_h_shift= get_symbol(&s->c, s->header_state, 0);
            s->chroma_v_shift= get_symbol(&s->c, s->header_state, 0);

            if(s->chroma_h_shift == 1 && s->chroma_v_shift==1){
                s->avctx->pix_fmt= AV_PIX_FMT_YUV420P;
            }else if(s->chroma_h_shift == 0 && s->chroma_v_shift==0){
                s->avctx->pix_fmt= AV_PIX_FMT_YUV444P;
            }else if(s->chroma_h_shift == 2 && s->chroma_v_shift==2){
                s->avctx->pix_fmt= AV_PIX_FMT_YUV410P;
            } else {
                av_log(s, AV_LOG_ERROR, "unsupported color subsample mode %d %d\n", s->chroma_h_shift, s->chroma_v_shift);
                s->chroma_h_shift = s->chroma_v_shift = 1;
                s->avctx->pix_fmt= AV_PIX_FMT_YUV420P;
                return AVERROR_INVALIDDATA;
            }
            s->nb_planes = 3;
        } else {
            av_log(s, AV_LOG_ERROR, "unsupported color space\n");
            s->chroma_h_shift = s->chroma_v_shift = 1;
            s->avctx->pix_fmt= AV_PIX_FMT_YUV420P;
            return AVERROR_INVALIDDATA;
        }


        s->spatial_scalability= get_rac(&s->c, s->header_state);
//        s->rate_scalability= get_rac(&s->c, s->header_state);
        GET_S(s->obmc.max_ref_frames, tmp < (unsigned)MAX_REF_FRAMES)
        s->obmc.max_ref_frames++;

        decode_qlogs(s);
    }

    if(!s->keyframe){
        if(get_rac(&s->c, s->header_state)){
            for(plane_index=0; plane_index<FFMIN(s->nb_planes, 2); plane_index++){
                int htaps, i, sum=0;
                PlaneObmc *p= &s->obmc.plane[plane_index];
                p->diag_mc= get_rac(&s->c, s->header_state);
                htaps= get_symbol(&s->c, s->header_state, 0)*2 + 2;
                if((unsigned)htaps > HTAPS_MAX || htaps==0)
                    return AVERROR_INVALIDDATA;
                p->htaps= htaps;
                for(i= htaps/2; i; i--){
                    p->hcoeff[i]= get_symbol(&s->c, s->header_state, 0) * (1-2*(i&1));
                    sum += p->hcoeff[i];
                }
                p->hcoeff[0]= 32-sum;
            }
            s->obmc.plane[2].diag_mc= s->obmc.plane[1].diag_mc;
            s->obmc.plane[2].htaps  = s->obmc.plane[1].htaps;
            memcpy(s->obmc.plane[2].hcoeff, s->obmc.plane[1].hcoeff, sizeof(s->obmc.plane[1].hcoeff));
        }
        if(get_rac(&s->c, s->header_state)){
            GET_S(s->spatial_decomposition_count, 0 < tmp && tmp <= MAX_DECOMPOSITIONS)
            decode_qlogs(s);
        }
    }

    s->spatial_decomposition_type+= get_symbol(&s->c, s->header_state, 1);
    if(s->spatial_decomposition_type > 1U){
        av_log(s->avctx, AV_LOG_ERROR, "spatial_decomposition_type %d not supported\n", s->spatial_decomposition_type);
        return AVERROR_INVALIDDATA;
    }
    if(FFMIN(s->avctx-> width>>s->chroma_h_shift,
             s->avctx->height>>s->chroma_v_shift) >> (s->spatial_decomposition_count-1) <= 1){
        av_log(s->avctx, AV_LOG_ERROR, "spatial_decomposition_count %d too large for size\n", s->spatial_decomposition_count);
        return AVERROR_INVALIDDATA;
    }


    s->qlog                 += get_symbol(&s->c, s->header_state, 1);
    s->obmc.mv_scale        += get_symbol(&s->c, s->header_state, 1);
    s->qbias                += get_symbol(&s->c, s->header_state, 1);
    s->obmc.block_max_depth += get_symbol(&s->c, s->header_state, 1);
    if(s->obmc.block_max_depth > 1 || s->obmc.block_max_depth < 0){
        av_log(s->avctx, AV_LOG_ERROR, "block_max_depth= %d is too large\n", s->obmc.block_max_depth);
        s->obmc.block_max_depth= 0;
        return AVERROR_INVALIDDATA;
    }
    
    obmc_decode_init(&s->obmc);
    ff_snow_init_decode_callbacks(&s->obmc.obmc_coder, s->avctx);

    return 0;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    int ret;

    if ((ret = ff_snow_common_init(avctx)) < 0) {
        return ret;
    }

    return 0;
}

static int decode_frame(AVCodecContext *avctx, void *data, int *got_frame,
                        AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    SnowContext *s = avctx->priv_data;
    RangeCoder * const c= &s->c;
    int bytes_read;
    AVFrame *picture = data;
    int level, orientation, plane_index;
    int res;

    ff_init_range_decoder(c, buf, buf_size);
    ff_build_rac_states(c, 0.05*(1LL<<32), 256-8);

    s->obmc.current_picture->pict_type= AV_PICTURE_TYPE_I; //FIXME I vs. P
    if ((res = decode_header(s)) < 0)
        return res;
    if ((res=ff_snow_common_init_after_header(avctx)) < 0)
        return res;

    // realloc slice buffer for the case that spatial_decomposition_count changed
    ff_slice_buffer_destroy(&s->sb);
    if ((res = ff_slice_buffer_init(&s->sb, s->obmc.plane[0].height,
                                    (MB_SIZE >> s->obmc.block_max_depth) +
                                    s->spatial_decomposition_count * 11 + 1,
                                    s->obmc.plane[0].width,
                                    s->obmc.spatial_idwt_buffer)) < 0)
        return res;

    //keyframe flag duplication mess FIXME
    if(avctx->debug&FF_DEBUG_PICT_INFO)
        av_log(avctx, AV_LOG_ERROR,
               "keyframe:%d qlog:%d qbias: %d mvscale: %d "
               "decomposition_type:%d decomposition_count:%d\n",
               s->keyframe, s->qlog, s->qbias, s->obmc.mv_scale,
               s->spatial_decomposition_type,
               s->spatial_decomposition_count
              );

    obmc_decode_frame(&s->obmc);

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        Plane *p= &s->plane[plane_index];
        int w= p->width;
        int h= p->height;
        int x, y;
        int decode_state[MAX_DECOMPOSITIONS][4][1]; /* Stored state info for unpack_coeffs. 1 variable per instance. */

        if(s->avctx->debug&2048){
            memset(s->spatial_dwt_buffer, 0, sizeof(DWTELEM)*w*h);
            predict_plane(&s->obmc, s->obmc.spatial_idwt_buffer, plane_index, 1);

            for(y=0; y<h; y++){
                for(x=0; x<w; x++){
                    int v= s->obmc.current_picture->data[plane_index][y*s->obmc.current_picture->linesize[plane_index] + x];
                    s->obmc.mconly_picture->data[plane_index][y*s->obmc.mconly_picture->linesize[plane_index] + x]= v;
                }
            }
        }

        {
        for(level=0; level<s->spatial_decomposition_count; level++){
            for(orientation=level ? 1 : 0; orientation<4; orientation++){
                SubBand *b= &p->band[level][orientation];
                unpack_coeffs(s, b, b->parent, orientation);
            }
        }
        }

        {
        const int mb_h= s->obmc.b_height << s->obmc.block_max_depth;
        const int block_size = MB_SIZE >> s->obmc.block_max_depth;
        const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
        int mb_y;
        DWTCompose cs[MAX_DECOMPOSITIONS];
        int yd=0, yq=0;
        int y;
        int end_y;

        ff_spatial_idwt_buffered_init(cs, &s->sb, w, h, 1, s->spatial_decomposition_type, s->spatial_decomposition_count);
        for(mb_y=0; mb_y<=mb_h; mb_y++){

            int slice_starty = block_h*mb_y;
            int slice_h = block_h*(mb_y+1);

            if (!(s->keyframe || s->avctx->debug&512)){
                slice_starty = FFMAX(0, slice_starty - (block_h >> 1));
                slice_h -= (block_h >> 1);
            }

            for(level=0; level<s->spatial_decomposition_count; level++){
                for(orientation=level ? 1 : 0; orientation<4; orientation++){
                    SubBand *b= &p->band[level][orientation];
                    int start_y;
                    int end_y;
                    int our_mb_start = mb_y;
                    int our_mb_end = (mb_y + 1);
                    const int extra= 3;
                    start_y = (mb_y ? ((block_h * our_mb_start) >> (s->spatial_decomposition_count - level)) + s->spatial_decomposition_count - level + extra: 0);
                    end_y = (((block_h * our_mb_end) >> (s->spatial_decomposition_count - level)) + s->spatial_decomposition_count - level + extra);
                    if (!(s->keyframe || s->avctx->debug&512)){
                        start_y = FFMAX(0, start_y - (block_h >> (1+s->spatial_decomposition_count - level)));
                        end_y = FFMAX(0, end_y - (block_h >> (1+s->spatial_decomposition_count - level)));
                    }
                    start_y = FFMIN(b->height, start_y);
                    end_y = FFMIN(b->height, end_y);

                    if (start_y != end_y){
                        if (orientation == 0){
                            SubBand * correlate_band = &p->band[0][0];
                            int correlate_end_y = FFMIN(b->height, end_y + 1);
                            int correlate_start_y = FFMIN(b->height, (start_y ? start_y + 1 : 0));
                            decode_subband_slice_buffered(s, correlate_band, &s->sb, correlate_start_y, correlate_end_y, decode_state[0][0]);
                            correlate_slice_buffered(s, &s->sb, correlate_band, correlate_band->ibuf, correlate_band->stride, 1, 0, correlate_start_y, correlate_end_y);
                            dequantize_slice_buffered(s, &s->sb, correlate_band, correlate_band->ibuf, correlate_band->stride, start_y, end_y);
                        }
                        else
                            decode_subband_slice_buffered(s, b, &s->sb, start_y, end_y, decode_state[level][orientation]);
                    }
                }
            }

            for(; yd<slice_h; yd+=4){
                ff_spatial_idwt_buffered_slice(&s->obmc.dwt, cs, &s->sb, s->temp_idwt_buffer, w, h, 1, s->spatial_decomposition_type, s->spatial_decomposition_count, yd);
            }

            if(s->qlog == LOSSLESS_QLOG){
                for(; yq<slice_h && yq<h; yq++){
                    IDWTELEM * line = slice_buffer_get_line(&s->sb, yq);
                    for(x=0; x<w; x++){
                        line[x] <<= FRAC_BITS;
                    }
                }
            }

            predict_slice_buffered(&s->obmc, &s->sb, s->obmc.spatial_idwt_buffer, plane_index, 1, mb_y);

            y = FFMIN(p->height, slice_starty);
            end_y = FFMIN(p->height, slice_h);
            while(y < end_y)
                ff_slice_buffer_release(&s->sb, y++);
        }

        ff_slice_buffer_flush(&s->sb);
        }

    }

    emms_c();

    ff_obmc_release_buffer(&s->obmc);

    if(!(s->avctx->debug&2048))
        res = av_frame_ref(picture, s->obmc.current_picture);
    else
        res = av_frame_ref(picture, s->obmc.mconly_picture);
    if (res >= 0 && s->obmc.avmv_index) {
        AVFrameSideData *sd;

        sd = av_frame_new_side_data(picture, AV_FRAME_DATA_MOTION_VECTORS, s->obmc.avmv_index * sizeof(AVMotionVector));
        if (!sd)
            return AVERROR(ENOMEM);
        memcpy(sd->data, s->obmc.avmv, s->obmc.avmv_index * sizeof(AVMotionVector));
    }

    av_freep(&s->obmc.avmv);

    if (res < 0)
        return res;

    *got_frame = 1;

    bytes_read= c->bytestream - c->bytestream_start;
    if(bytes_read ==0) av_log(s->avctx, AV_LOG_ERROR, "error at end of frame\n"); //FIXME

    return bytes_read;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    SnowContext *s = avctx->priv_data;

    ff_slice_buffer_destroy(&s->sb);

    ff_snow_common_end(s);

    return 0;
}

AVCodec ff_snow_decoder = {
    .name           = "snow",
    .long_name      = NULL_IF_CONFIG_SMALL("Snow"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_SNOW,
    .priv_data_size = sizeof(SnowContext),
    .init           = decode_init,
    .close          = decode_end,
    .decode         = decode_frame,
    .capabilities   = AV_CODEC_CAP_DR1 /*| AV_CODEC_CAP_DRAW_HORIZ_BAND*/,
    .caps_internal  = FF_CODEC_CAP_INIT_THREADSAFE |
                      FF_CODEC_CAP_INIT_CLEANUP,
};
