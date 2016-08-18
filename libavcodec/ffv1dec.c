/*
 * FFV1 decoder
 *
 * Copyright (c) 2003-2013 Michael Niedermayer <michaelni@gmx.at>
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

/**
 * @file
 * FF Video Codec 1 (a lossless codec) decoder
 */

#include "libavutil/avassert.h"
#include "libavutil/crc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timer.h"
#include "avcodec.h"
#include "internal.h"
#include "get_bits.h"
#include "rangecoder.h"
#include "golomb.h"
#include "mathops.h"
#include "ffv1.h"

#include "obmc.h"

static int ff_predict_frame(AVCodecContext *avctx, FFV1Context *f)
{
    int ret, i, x, y;
    AVFrame *curr     = f->picture.f;
    AVFrame *prev     = f->obmc.current_picture;
    AVFrame *residual = f->residual.f;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(prev->format);
    int width  = f->width;
    int height = f->height;
    int has_plane[4] = { 0 };
    const int cw = AV_CEIL_RSHIFT(width, desc->log2_chroma_w);
    const int ch = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);

    if (f->residual.f)
        ff_thread_release_buffer(avctx, &f->residual);
    if ((ret = ff_thread_ref_frame(&f->residual, &f->picture)) < 0)
        return ret;
    if ((ret = av_frame_make_writable(f->residual.f)) < 0) {
        ff_thread_release_buffer(avctx, &f->residual);
        return ret;
    }

    for (i = 0; i < desc->nb_components; i++)
        has_plane[desc->comp[i].plane] = 1;

    for (i = 0; i < desc->nb_components && has_plane[i]; i++)
        memset(residual->buf[i]->data, 0, residual->buf[i]->size * sizeof(*residual->buf[i]->data));

    for (i = 0; i < desc->nb_components; i++) {
        const int w1 = (i == 1 || i == 2) ? cw : width;
        const int h1 = (i == 1 || i == 2) ? ch : height;

        const int depth = desc->comp[i].depth;
        const int max_val = 1 << depth;

        memset(f->p_image_line_buf, 0, 2 * width * sizeof(*f->p_image_line_buf));
        memset(f->c_image_line_buf, 0, 2 * width * sizeof(*f->c_image_line_buf));

        for (y = 0; y < h1; y++) {
            memset(f->p_image_line_buf, 0, width * sizeof(*f->p_image_line_buf));
            memset(f->c_image_line_buf, 0, width * sizeof(*f->c_image_line_buf));
            av_read_image_line(f->c_image_line_buf,
                               (void *)curr->data,
                               curr->linesize,
                               desc,
                               0, y, i, w1, 0);
            av_read_image_line(f->p_image_line_buf,
                              (void *)prev->data,
                              prev->linesize,
                              desc,
                              0, y, i, w1, 0);
            for (x = 0; x < w1; ++x) {
                f->c_image_line_buf[x] = (f->c_image_line_buf[x] + f->p_image_line_buf[x] - (max_val >> 2)) & (max_val - 1);
            }
            av_write_image_line(f->c_image_line_buf,
                                residual->data,
                                residual->linesize,
                                desc,
                                0, y, i, w1);
        }
    }

    av_frame_copy(curr, residual);

    return 0;
}

static inline av_flatten int get_symbol_inline(RangeCoder *c, uint8_t *state,
                                               int is_signed)
{
    if (get_rac(c, state + 0))
        return 0;
    else {
        int i, e, a;
        e = 0;
        while (get_rac(c, state + 1 + FFMIN(e, 9))) { // 1..10
            e++;
            if (e > 31)
                return AVERROR_INVALIDDATA;
        }

        a = 1;
        for (i = e - 1; i >= 0; i--)
            a += a + get_rac(c, state + 22 + FFMIN(i, 9));  // 22..31

        e = -(is_signed && get_rac(c, state + 11 + FFMIN(e, 10))); // 11..21
        return (a ^ e) - e;
    }
}

static av_noinline int get_symbol(RangeCoder *c, uint8_t *state, int is_signed)
{
    return get_symbol_inline(c, state, is_signed);
}

static inline int get_vlc_symbol(GetBitContext *gb, VlcState *const state,
                                 int bits)
{
    int k, i, v, ret;

    i = state->count;
    k = 0;
    while (i < state->error_sum) { // FIXME: optimize
        k++;
        i += i;
    }

    v = get_sr_golomb(gb, k, 12, bits);
    ff_dlog(NULL, "v:%d bias:%d error:%d drift:%d count:%d k:%d",
            v, state->bias, state->error_sum, state->drift, state->count, k);

#if 0 // JPEG LS
    if (k == 0 && 2 * state->drift <= -state->count)
        v ^= (-1);
#else
    v ^= ((2 * state->drift + state->count) >> 31);
#endif

    ret = fold(v + state->bias, bits);

    update_vlc_state(state, v);

    return ret;
}

static int decode_q_branch(FFV1Context *f, int level, int x, int y)
{
    RangeCoder *const c = &f->slice_context[0]->c;
    OBMCContext *s = &f->obmc;
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

    if(s->keyframe){
        set_blocks(s, level, x, y, null_block.color[0], null_block.color[1], null_block.color[2], null_block.mx, null_block.my, null_block.ref, BLOCK_INTRA);
        return 0;
    }

    if(level==s->block_max_depth || get_rac(c, &f->block_state[4 + s_context])){
        int type, mx, my;
        int l = left->color[0];
        int cb= left->color[1];
        int cr= left->color[2];
        unsigned ref = 0;
        int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
        int mx_context= av_log2(2*FFABS(left->mx - top->mx)) + 0*av_log2(2*FFABS(tr->mx - top->mx));
        int my_context= av_log2(2*FFABS(left->my - top->my)) + 0*av_log2(2*FFABS(tr->my - top->my));

        type= get_rac(c, &f->block_state[1 + left->type + top->type]) ? BLOCK_INTRA : 0;

        if(type){
            pred_mv(s, &mx, &my, 0, left, top, tr);
            l += get_symbol(c, &f->block_state[32], 1);
            if (f->obmc.nb_planes > 2) {
                cb += get_symbol(c, &f->block_state[64], 1);
                cr += get_symbol(c, &f->block_state[96], 1);
            }
        }else{
            if(s->ref_frames > 1)
                ref = get_symbol(c, &f->block_state[128 + 1024 + 32*ref_context], 0);
            if (ref >= s->ref_frames) {
                av_log(s->avctx, AV_LOG_ERROR, "Invalid ref\n");
                return AVERROR_INVALIDDATA;
            }
            pred_mv(s, &mx, &my, ref, left, top, tr);
            mx += get_symbol(c, &f->block_state[128 + 32*(mx_context + 16*!!ref)], 1);
            my += get_symbol(c, &f->block_state[128 + 32*(my_context + 16*!!ref)], 1);
        }
        set_blocks(s, level, x, y, l, cb, cr, mx, my, ref, type);
    }else{
        if ((res = decode_q_branch(f, level+1, 2*x+0, 2*y+0)) < 0 ||
            (res = decode_q_branch(f, level+1, 2*x+1, 2*y+0)) < 0 ||
            (res = decode_q_branch(f, level+1, 2*x+0, 2*y+1)) < 0 ||
            (res = decode_q_branch(f, level+1, 2*x+1, 2*y+1)) < 0)
            return res;
    }
    return 0;
}

static int decode_blocks(FFV1Context *s){
    int x, y;
    int w= s->obmc.b_width;
    int h= s->obmc.b_height;
    int res;

    for(y=0; y<h; y++){
        for(x=0; x<w; x++){
            if ((res = decode_q_branch(s, 0, x, y)) < 0)
                return res;
        }
    }

    return 0;
}

#define TYPE int16_t
#define RENAME(name) name
#include "ffv1dec_template.c"
#undef TYPE
#undef RENAME

#define TYPE int32_t
#define RENAME(name) name ## 32
#include "ffv1dec_template.c"

static void decode_plane(FFV1Context *s, uint8_t *src,
                         int w, int h, int stride, int plane_index,
                         int pixel_stride)
{
    int x, y;
    int16_t *sample[2];
    sample[0] = s->sample_buffer + 3;
    sample[1] = s->sample_buffer + w + 6 + 3;

    s->run_index = 0;

    memset(s->sample_buffer, 0, 2 * (w + 6) * sizeof(*s->sample_buffer));

    for (y = 0; y < h; y++) {
        int16_t *temp = sample[0]; // FIXME: try a normal buffer

        sample[0] = sample[1];
        sample[1] = temp;

        sample[1][-1] = sample[0][0];
        sample[0][w]  = sample[0][w - 1];

// { START_TIMER
        if (s->avctx->bits_per_raw_sample <= 8) {
            decode_line(s, w, sample, plane_index, 8);
            for (x = 0; x < w; x++)
                src[x*pixel_stride + stride * y] = sample[1][x];
        } else {
            decode_line(s, w, sample, plane_index, s->avctx->bits_per_raw_sample);
            if (s->packed_at_lsb) {
                for (x = 0; x < w; x++) {
                    ((uint16_t*)(src + stride*y))[x*pixel_stride] = sample[1][x];
                }
            } else {
                for (x = 0; x < w; x++) {
                    ((uint16_t*)(src + stride*y))[x*pixel_stride] = sample[1][x] << (16 - s->avctx->bits_per_raw_sample);
                }
            }
        }
// STOP_TIMER("decode-line") }
    }
}

static int decode_slice_header(FFV1Context *f, FFV1Context *fs)
{
    RangeCoder *c = &fs->c;
    uint8_t state[CONTEXT_SIZE];
    unsigned ps, i, context_count;
    memset(state, 128, sizeof(state));

    av_assert0(f->version > 2);

    fs->slice_x      =  get_symbol(c, state, 0)      * f->width ;
    fs->slice_y      =  get_symbol(c, state, 0)      * f->height;
    fs->slice_width  = (get_symbol(c, state, 0) + 1) * f->width  + fs->slice_x;
    fs->slice_height = (get_symbol(c, state, 0) + 1) * f->height + fs->slice_y;

    fs->slice_x /= f->num_h_slices;
    fs->slice_y /= f->num_v_slices;
    fs->slice_width  = fs->slice_width /f->num_h_slices - fs->slice_x;
    fs->slice_height = fs->slice_height/f->num_v_slices - fs->slice_y;
    if ((unsigned)fs->slice_width > f->width || (unsigned)fs->slice_height > f->height)
        return -1;
    if (    (unsigned)fs->slice_x + (uint64_t)fs->slice_width  > f->width
         || (unsigned)fs->slice_y + (uint64_t)fs->slice_height > f->height)
        return -1;

    for (i = 0; i < f->plane_count; i++) {
        PlaneContext * const p = &fs->plane[i];
        int idx = get_symbol(c, state, 0);
        if (idx >= (unsigned)f->quant_table_count) {
            av_log(f->avctx, AV_LOG_ERROR, "quant_table_index out of range\n");
            return -1;
        }
        p->quant_table_index = idx;
        memcpy(p->quant_table, f->quant_tables[idx], sizeof(p->quant_table));
        context_count = f->context_count[idx];

        if (p->context_count < context_count) {
            av_freep(&p->state);
            av_freep(&p->vlc_state);
        }
        p->context_count = context_count;
    }

    ps = get_symbol(c, state, 0);
    if (ps == 1) {
        f->cur->interlaced_frame = 1;
        f->cur->top_field_first  = 1;
    } else if (ps == 2) {
        f->cur->interlaced_frame = 1;
        f->cur->top_field_first  = 0;
    } else if (ps == 3) {
        f->cur->interlaced_frame = 0;
    }
    f->cur->sample_aspect_ratio.num = get_symbol(c, state, 0);
    f->cur->sample_aspect_ratio.den = get_symbol(c, state, 0);

    if (av_image_check_sar(f->width, f->height,
                           f->cur->sample_aspect_ratio) < 0) {
        av_log(f->avctx, AV_LOG_WARNING, "ignoring invalid SAR: %u/%u\n",
               f->cur->sample_aspect_ratio.num,
               f->cur->sample_aspect_ratio.den);
        f->cur->sample_aspect_ratio = (AVRational){ 0, 1 };
    }

    if (fs->version > 3) {
        fs->slice_reset_contexts = get_rac(c, state);
        fs->slice_coding_mode = get_symbol(c, state, 0);
        if (fs->slice_coding_mode != 1) {
            fs->slice_rct_by_coef = get_symbol(c, state, 0);
            fs->slice_rct_ry_coef = get_symbol(c, state, 0);
            if ((uint64_t)fs->slice_rct_by_coef + (uint64_t)fs->slice_rct_ry_coef > 4) {
                av_log(f->avctx, AV_LOG_ERROR, "slice_rct_y_coef out of range\n");
                return AVERROR_INVALIDDATA;
            }
        }
    }

    return 0;
}

static int decode_slice(AVCodecContext *c, void *arg)
{
    FFV1Context *fs   = *(void **)arg;
    FFV1Context *f    = fs->avctx->priv_data;
    int width, height, x, y, ret;
    const int ps      = av_pix_fmt_desc_get(c->pix_fmt)->comp[0].step;
    AVFrame * const p = f->cur;
    int i, si;

    for( si=0; fs != f->slice_context[si]; si ++)
        ;

    if(f->fsrc && !p->key_frame)
        ff_thread_await_progress(&f->last_picture, si, 0);

    if(f->fsrc && !p->key_frame) {
        FFV1Context *fssrc = f->fsrc->slice_context[si];
        FFV1Context *fsdst = f->slice_context[si];
        av_assert1(fsdst->plane_count == fssrc->plane_count);
        av_assert1(fsdst == fs);

        if (!p->key_frame)
            fsdst->slice_damaged |= fssrc->slice_damaged;

        for (i = 0; i < f->plane_count; i++) {
            PlaneContext *psrc = &fssrc->plane[i];
            PlaneContext *pdst = &fsdst->plane[i];

            av_free(pdst->state);
            av_free(pdst->vlc_state);
            memcpy(pdst, psrc, sizeof(*pdst));
            pdst->state = NULL;
            pdst->vlc_state = NULL;

            if (fssrc->ac) {
                pdst->state = av_malloc_array(CONTEXT_SIZE,  psrc->context_count);
                memcpy(pdst->state, psrc->state, CONTEXT_SIZE * psrc->context_count);
            } else {
                pdst->vlc_state = av_malloc_array(sizeof(*pdst->vlc_state), psrc->context_count);
                memcpy(pdst->vlc_state, psrc->vlc_state, sizeof(*pdst->vlc_state) * psrc->context_count);
            }
        }
    }

    fs->slice_rct_by_coef = 1;
    fs->slice_rct_ry_coef = 1;

    if (f->version > 2) {
        if (ff_ffv1_init_slice_state(f, fs) < 0)
            return AVERROR(ENOMEM);
        if (decode_slice_header(f, fs) < 0) {
            fs->slice_x = fs->slice_y = fs->slice_height = fs->slice_width = 0;
            fs->slice_damaged = 1;
            return AVERROR_INVALIDDATA;
        }
    }
    if ((ret = ff_ffv1_init_slice_state(f, fs)) < 0)
        return ret;
    if (f->cur->key_frame || fs->slice_reset_contexts)
        ff_ffv1_clear_slice_state(f, fs);

    width  = fs->slice_width;
    height = fs->slice_height;
    x      = fs->slice_x;
    y      = fs->slice_y;

    if (fs->ac == AC_GOLOMB_RICE) {
        if (f->version == 3 && f->micro_version > 1 || f->version > 3)
            get_rac(&fs->c, (uint8_t[]) { 129 });
        fs->ac_byte_count = f->version > 2 || (!x && !y) ? fs->c.bytestream - fs->c.bytestream_start - 1 : 0;
        init_get_bits(&fs->gb,
                      fs->c.bytestream_start + fs->ac_byte_count,
                      (fs->c.bytestream_end - fs->c.bytestream_start - fs->ac_byte_count) * 8);
    }

    av_assert1(width && height);
    if (f->colorspace == 0 && (f->chroma_planes || !fs->transparency)) {
        const int chroma_width  = AV_CEIL_RSHIFT(width,  f->chroma_h_shift);
        const int chroma_height = AV_CEIL_RSHIFT(height, f->chroma_v_shift);
        const int cx            = x >> f->chroma_h_shift;
        const int cy            = y >> f->chroma_v_shift;
        decode_plane(fs, p->data[0] + ps*x + y*p->linesize[0], width, height, p->linesize[0], 0, 1);

        if (f->chroma_planes) {
            decode_plane(fs, p->data[1] + ps*cx+cy*p->linesize[1], chroma_width, chroma_height, p->linesize[1], 1, 1);
            decode_plane(fs, p->data[2] + ps*cx+cy*p->linesize[2], chroma_width, chroma_height, p->linesize[2], 1, 1);
        }
        if (fs->transparency)
            decode_plane(fs, p->data[3] + ps*x + y*p->linesize[3], width, height, p->linesize[3], (f->version >= 4 && !f->chroma_planes) ? 1 : 2, 1);
    } else if (f->colorspace == 0) {
         decode_plane(fs, p->data[0] + ps*x + y*p->linesize[0]    , width, height, p->linesize[0], 0, 2);
         decode_plane(fs, p->data[0] + ps*x + y*p->linesize[0] + 1, width, height, p->linesize[0], 1, 2);
    } else if (f->use32bit) {
        uint8_t *planes[3] = { p->data[0] + ps * x + y * p->linesize[0],
                               p->data[1] + ps * x + y * p->linesize[1],
                               p->data[2] + ps * x + y * p->linesize[2] };
        decode_rgb_frame32(fs, planes, width, height, p->linesize);
    } else {
        uint8_t *planes[3] = { p->data[0] + ps * x + y * p->linesize[0],
                               p->data[1] + ps * x + y * p->linesize[1],
                               p->data[2] + ps * x + y * p->linesize[2] };
        decode_rgb_frame(fs, planes, width, height, p->linesize);
    }
    if (fs->ac != AC_GOLOMB_RICE && f->version > 2) {
        int v;
        get_rac(&fs->c, (uint8_t[]) { 129 });
        v = fs->c.bytestream_end - fs->c.bytestream - 2 - 5*f->ec;
        if (v) {
            av_log(f->avctx, AV_LOG_ERROR, "bytestream end mismatching by %d\n", v);
            fs->slice_damaged = 1;
        }
    }

    emms_c();

    ff_thread_report_progress(&f->picture, si, 0);

    return 0;
}

static int read_quant_table(RangeCoder *c, int16_t *quant_table, int scale)
{
    int v;
    int i = 0;
    uint8_t state[CONTEXT_SIZE];

    memset(state, 128, sizeof(state));

    for (v = 0; i < 128; v++) {
        unsigned len = get_symbol(c, state, 0) + 1;

        if (len > 128 - i || !len)
            return AVERROR_INVALIDDATA;

        while (len--) {
            quant_table[i] = scale * v;
            i++;
        }
    }

    for (i = 1; i < 128; i++)
        quant_table[256 - i] = -quant_table[i];
    quant_table[128] = -quant_table[127];

    return 2 * v - 1;
}

static int read_quant_tables(RangeCoder *c,
                             int16_t quant_table[MAX_CONTEXT_INPUTS][256])
{
    int i;
    int context_count = 1;

    for (i = 0; i < 5; i++) {
        int ret = read_quant_table(c, quant_table[i], context_count);
        if (ret < 0)
            return ret;
        context_count *= ret;
        if (context_count > 32768U) {
            return AVERROR_INVALIDDATA;
        }
    }
    return (context_count + 1) / 2;
}

static int read_extra_header(FFV1Context *f)
{
    RangeCoder *const c = &f->c;
    uint8_t state[CONTEXT_SIZE];
    int i, j, k, ret;
    uint8_t state2[32][CONTEXT_SIZE];
    unsigned crc = 0;

    memset(state2, 128, sizeof(state2));
    memset(state, 128, sizeof(state));

    ff_init_range_decoder(c, f->avctx->extradata, f->avctx->extradata_size);
    ff_build_rac_states(c, 0.05 * (1LL << 32), 256 - 8);

    f->version = get_symbol(c, state, 0);
    if (f->version < 2) {
        av_log(f->avctx, AV_LOG_ERROR, "Invalid version in global header\n");
        return AVERROR_INVALIDDATA;
    }
    if (f->version > 2) {
        c->bytestream_end -= 4;
        f->micro_version = get_symbol(c, state, 0);
        if (f->micro_version < 0)
            return AVERROR_INVALIDDATA;
    }

    if (f->version == 3 && f->micro_version > 4 || f->version == 4 && f->micro_version > 2) {
        f->p_frame = 1;
        f->micro_version--;
    } else {
        f->p_frame = 0;
    }
    f->ac = get_symbol(c, state, 0);

    if (f->ac == AC_RANGE_CUSTOM_TAB) {
        for (i = 1; i < 256; i++)
            f->state_transition[i] = get_symbol(c, state, 1) + c->one_state[i];
    }

    f->colorspace                 = get_symbol(c, state, 0); //YUV cs type
    f->avctx->bits_per_raw_sample = get_symbol(c, state, 0);
    f->chroma_planes              = get_rac(c, state);
    f->chroma_h_shift             = get_symbol(c, state, 0);
    f->chroma_v_shift             = get_symbol(c, state, 0);
    f->transparency               = get_rac(c, state);
    f->plane_count                = 1 + (f->chroma_planes || f->version<4) + f->transparency;
    f->num_h_slices               = 1 + get_symbol(c, state, 0);
    f->num_v_slices               = 1 + get_symbol(c, state, 0);

    if (f->chroma_h_shift > 4U || f->chroma_v_shift > 4U) {
        av_log(f->avctx, AV_LOG_ERROR, "chroma shift parameters %d %d are invalid\n",
               f->chroma_h_shift, f->chroma_v_shift);
        return AVERROR_INVALIDDATA;
    }

    if (f->num_h_slices > (unsigned)f->width  || !f->num_h_slices ||
        f->num_v_slices > (unsigned)f->height || !f->num_v_slices
       ) {
        av_log(f->avctx, AV_LOG_ERROR, "slice count invalid\n");
        return AVERROR_INVALIDDATA;
    }

    f->quant_table_count = get_symbol(c, state, 0);
    if (f->quant_table_count > (unsigned)MAX_QUANT_TABLES || !f->quant_table_count) {
        av_log(f->avctx, AV_LOG_ERROR, "quant table count %d is invalid\n", f->quant_table_count);
        f->quant_table_count = 0;
        return AVERROR_INVALIDDATA;
    }

    for (i = 0; i < f->quant_table_count; i++) {
        f->context_count[i] = read_quant_tables(c, f->quant_tables[i]);
        if (f->context_count[i] < 0) {
            av_log(f->avctx, AV_LOG_ERROR, "read_quant_table error\n");
            return AVERROR_INVALIDDATA;
        }
    }
    if ((ret = ff_ffv1_allocate_initial_states(f)) < 0)
        return ret;

    for (i = 0; i < f->quant_table_count; i++)
        if (get_rac(c, state)) {
            for (j = 0; j < f->context_count[i]; j++)
                for (k = 0; k < CONTEXT_SIZE; k++) {
                    int pred = j ? f->initial_states[i][j - 1][k] : 128;
                    f->initial_states[i][j][k] =
                        (pred + get_symbol(c, state2[k], 1)) & 0xFF;
                }
        }

    if (f->version > 2) {
        f->ec = get_symbol(c, state, 0);
        if (f->micro_version > 2)
            f->intra = get_symbol(c, state, 0);
    }

    if (f->version > 2) {
        unsigned v;
        v = av_crc(av_crc_get_table(AV_CRC_32_IEEE), 0,
                   f->avctx->extradata, f->avctx->extradata_size);
        if (v || f->avctx->extradata_size < 4) {
            av_log(f->avctx, AV_LOG_ERROR, "CRC mismatch %X!\n", v);
            return AVERROR_INVALIDDATA;
        }
        crc = AV_RB32(f->avctx->extradata + f->avctx->extradata_size - 4);
    }

    if (f->avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(f->avctx, AV_LOG_DEBUG,
               "global: ver:%d.%d, coder:%d, colorspace: %d bpr:%d chroma:%d(%d:%d), alpha:%d slices:%dx%d qtabs:%d ec:%d intra:%d CRC:0x%08X\n",
               f->version, f->micro_version,
               f->ac,
               f->colorspace,
               f->avctx->bits_per_raw_sample,
               f->chroma_planes, f->chroma_h_shift, f->chroma_v_shift,
               f->transparency,
               f->num_h_slices, f->num_v_slices,
               f->quant_table_count,
               f->ec,
               f->intra,
               crc
              );
    return 0;
}

static int read_header(FFV1Context *f)
{
    uint8_t state[CONTEXT_SIZE];
    int i, j, ret, context_count = -1; //-1 to avoid warning
    RangeCoder *const c = &f->slice_context[0]->c;

    memset(state, 128, sizeof(state));

    if (f->version < 2) {
        int chroma_planes, chroma_h_shift, chroma_v_shift, transparency, colorspace, bits_per_raw_sample;
        unsigned v= get_symbol(c, state, 0);
        if (v >= 2) {
            av_log(f->avctx, AV_LOG_ERROR, "invalid version %d in ver01 header\n", v);
            return AVERROR_INVALIDDATA;
        }
        f->version = v;
        f->ac = get_symbol(c, state, 0);

        if (f->ac == AC_RANGE_CUSTOM_TAB) {
            for (i = 1; i < 256; i++)
                f->state_transition[i] = get_symbol(c, state, 1) + c->one_state[i];
        }

        colorspace          = get_symbol(c, state, 0); //YUV cs type
        bits_per_raw_sample = f->version > 0 ? get_symbol(c, state, 0) : f->avctx->bits_per_raw_sample;
        chroma_planes       = get_rac(c, state);
        chroma_h_shift      = get_symbol(c, state, 0);
        chroma_v_shift      = get_symbol(c, state, 0);
        transparency        = get_rac(c, state);
        if (colorspace == 0 && f->avctx->skip_alpha)
            transparency = 0;

        if (f->plane_count) {
            if (colorspace          != f->colorspace                 ||
                bits_per_raw_sample != f->avctx->bits_per_raw_sample ||
                chroma_planes       != f->chroma_planes              ||
                chroma_h_shift      != f->chroma_h_shift             ||
                chroma_v_shift      != f->chroma_v_shift             ||
                transparency        != f->transparency) {
                av_log(f->avctx, AV_LOG_ERROR, "Invalid change of global parameters\n");
                return AVERROR_INVALIDDATA;
            }
        }

        if (chroma_h_shift > 4U || chroma_v_shift > 4U) {
            av_log(f->avctx, AV_LOG_ERROR, "chroma shift parameters %d %d are invalid\n",
                   chroma_h_shift, chroma_v_shift);
            return AVERROR_INVALIDDATA;
        }

        f->colorspace                 = colorspace;
        f->avctx->bits_per_raw_sample = bits_per_raw_sample;
        f->chroma_planes              = chroma_planes;
        f->chroma_h_shift             = chroma_h_shift;
        f->chroma_v_shift             = chroma_v_shift;
        f->transparency               = transparency;

        f->plane_count    = 2 + f->transparency;
    }

    if (f->colorspace == 0) {
        if (!f->transparency && !f->chroma_planes) {
            if (f->avctx->bits_per_raw_sample <= 8)
                f->avctx->pix_fmt = AV_PIX_FMT_GRAY8;
            else
                f->avctx->pix_fmt = AV_PIX_FMT_GRAY16;
        } else if (f->transparency && !f->chroma_planes) {
            if (f->avctx->bits_per_raw_sample <= 8)
                f->avctx->pix_fmt = AV_PIX_FMT_YA8;
            else
                return AVERROR(ENOSYS);
        } else if (f->avctx->bits_per_raw_sample<=8 && !f->transparency) {
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUV444P; break;
            case 0x01: f->avctx->pix_fmt = AV_PIX_FMT_YUV440P; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUV422P; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUV420P; break;
            case 0x20: f->avctx->pix_fmt = AV_PIX_FMT_YUV411P; break;
            case 0x22: f->avctx->pix_fmt = AV_PIX_FMT_YUV410P; break;
            }
        } else if (f->avctx->bits_per_raw_sample <= 8 && f->transparency) {
            switch(16*f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUVA444P; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUVA422P; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUVA420P; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 9 && !f->transparency) {
            f->packed_at_lsb = 1;
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUV444P9; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUV422P9; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUV420P9; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 9 && f->transparency) {
            f->packed_at_lsb = 1;
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUVA444P9; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUVA422P9; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUVA420P9; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 10 && !f->transparency) {
            f->packed_at_lsb = 1;
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUV444P10; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUV422P10; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUV420P10; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 10 && f->transparency) {
            f->packed_at_lsb = 1;
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUVA444P10; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUVA422P10; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUVA420P10; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 16 && !f->transparency){
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUV444P16; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUV422P16; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUV420P16; break;
            }
        } else if (f->avctx->bits_per_raw_sample == 16 && f->transparency){
            switch(16 * f->chroma_h_shift + f->chroma_v_shift) {
            case 0x00: f->avctx->pix_fmt = AV_PIX_FMT_YUVA444P16; break;
            case 0x10: f->avctx->pix_fmt = AV_PIX_FMT_YUVA422P16; break;
            case 0x11: f->avctx->pix_fmt = AV_PIX_FMT_YUVA420P16; break;
            }
        }
    } else if (f->colorspace == 1) {
        if (f->chroma_h_shift || f->chroma_v_shift) {
            av_log(f->avctx, AV_LOG_ERROR,
                   "chroma subsampling not supported in this colorspace\n");
            return AVERROR(ENOSYS);
        }
        if (     f->avctx->bits_per_raw_sample <=  8 && !f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_0RGB32;
        else if (f->avctx->bits_per_raw_sample <=  8 && f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_RGB32;
        else if (f->avctx->bits_per_raw_sample ==  9 && !f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_GBRP9;
        else if (f->avctx->bits_per_raw_sample == 10 && !f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_GBRP10;
        else if (f->avctx->bits_per_raw_sample == 12 && !f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_GBRP12;
        else if (f->avctx->bits_per_raw_sample == 14 && !f->transparency)
            f->avctx->pix_fmt = AV_PIX_FMT_GBRP14;
        else if (f->avctx->bits_per_raw_sample == 16 && !f->transparency) {
            f->avctx->pix_fmt = AV_PIX_FMT_GBRP16;
            f->use32bit = 1;
        }
    } else {
        av_log(f->avctx, AV_LOG_ERROR, "colorspace not supported\n");
        return AVERROR(ENOSYS);
    }
    if (f->avctx->pix_fmt == AV_PIX_FMT_NONE) {
        av_log(f->avctx, AV_LOG_ERROR, "format not supported\n");
        return AVERROR(ENOSYS);
    }

    if ((ret = ff_obmc_decode_init(&f->obmc)) < 0)
        return ret;

    ff_dlog(f->avctx, "%d %d %d\n",
            f->chroma_h_shift, f->chroma_v_shift, f->avctx->pix_fmt);
    if (f->version < 2) {
        context_count = read_quant_tables(c, f->quant_table);
        if (context_count < 0) {
            av_log(f->avctx, AV_LOG_ERROR, "read_quant_table error\n");
            return AVERROR_INVALIDDATA;
        }
        f->slice_count = f->max_slice_count;
    } else if (f->version < 3) {
        f->slice_count = get_symbol(c, state, 0);
    } else {
        const uint8_t *p = c->bytestream_end;
        for (f->slice_count = 0;
             f->slice_count < MAX_SLICES && 3 < p - c->bytestream_start;
             f->slice_count++) {
            int trailer = 3 + 5*!!f->ec;
            int size = AV_RB24(p-trailer);
            if (size + trailer > p - c->bytestream_start)
                break;
            p -= size + trailer;
        }
    }
    if (f->slice_count > (unsigned)MAX_SLICES || f->slice_count <= 0 || f->slice_count > f->max_slice_count) {
        av_log(f->avctx, AV_LOG_ERROR, "slice count %d is invalid (max=%d)\n", f->slice_count, f->max_slice_count);
        return AVERROR_INVALIDDATA;
    }

    for (j = 0; j < f->slice_count; j++) {
        FFV1Context *fs = f->slice_context[j];
        fs->ac            = f->ac;
        fs->packed_at_lsb = f->packed_at_lsb;

        fs->slice_damaged = 0;

        if (f->version == 2) {
            fs->slice_x      =  get_symbol(c, state, 0)      * f->width ;
            fs->slice_y      =  get_symbol(c, state, 0)      * f->height;
            fs->slice_width  = (get_symbol(c, state, 0) + 1) * f->width  + fs->slice_x;
            fs->slice_height = (get_symbol(c, state, 0) + 1) * f->height + fs->slice_y;

            fs->slice_x     /= f->num_h_slices;
            fs->slice_y     /= f->num_v_slices;
            fs->slice_width  = fs->slice_width  / f->num_h_slices - fs->slice_x;
            fs->slice_height = fs->slice_height / f->num_v_slices - fs->slice_y;
            if ((unsigned)fs->slice_width  > f->width ||
                (unsigned)fs->slice_height > f->height)
                return AVERROR_INVALIDDATA;
            if (   (unsigned)fs->slice_x + (uint64_t)fs->slice_width  > f->width
                || (unsigned)fs->slice_y + (uint64_t)fs->slice_height > f->height)
                return AVERROR_INVALIDDATA;
        }

        for (i = 0; i < f->plane_count; i++) {
            PlaneContext *const p = &fs->plane[i];

            if (f->version == 2) {
                int idx = get_symbol(c, state, 0);
                if (idx > (unsigned)f->quant_table_count) {
                    av_log(f->avctx, AV_LOG_ERROR,
                           "quant_table_index out of range\n");
                    return AVERROR_INVALIDDATA;
                }
                p->quant_table_index = idx;
                memcpy(p->quant_table, f->quant_tables[idx],
                       sizeof(p->quant_table));
                context_count = f->context_count[idx];
            } else {
                memcpy(p->quant_table, f->quant_table, sizeof(p->quant_table));
            }

            if (f->version <= 2) {
                av_assert0(context_count >= 0);
                if (p->context_count < context_count) {
                    av_freep(&p->state);
                    av_freep(&p->vlc_state);
                }
                p->context_count = context_count;
            }
        }
    }

    return 0;
}

static int decode_p_header(FFV1Context *f)
{
    uint8_t state[CONTEXT_SIZE];
    int plane_index;
    RangeCoder *const c = &f->slice_context[0]->c;

    memset(state, 128, sizeof(state));

    if (f->key_frame) {
        memset(f->block_state, MID_STATE, sizeof(f->block_state));
        f->obmc.max_ref_frames = get_symbol(c, state, 0) + 1;
    }
    if (!f->key_frame) {
        for(plane_index=0; plane_index<FFMIN(f->obmc.nb_planes, 2); plane_index++){
            int htaps, i, sum=0;
            PlaneObmc *p= &f->obmc.plane[plane_index];
            p->diag_mc = get_rac(c, state);
            htaps = get_symbol(c, state, 0)*2 + 2;
            if((unsigned)htaps > HTAPS_MAX || htaps==0)
                return AVERROR_INVALIDDATA;
            p->htaps= htaps;
            for(i= p->htaps/2; i; i--) {
                p->hcoeff[i]= get_symbol(c, state, 0) * (1-2*(i&1));
                sum += p->hcoeff[i];
            }
            p->hcoeff[0]= 32-sum;
        }
        f->obmc.plane[2].diag_mc= f->obmc.plane[1].diag_mc;
        f->obmc.plane[2].htaps  = f->obmc.plane[1].htaps;
        memcpy(f->obmc.plane[2].hcoeff, f->obmc.plane[1].hcoeff, sizeof(f->obmc.plane[1].hcoeff));
    }

    f->obmc.mv_scale       = get_symbol(c, state, 0);
    f->obmc.block_max_depth= get_symbol(c, state, 0);
    if(f->obmc.block_max_depth > 1 || f->obmc.block_max_depth < 0){
        av_log(f->avctx, AV_LOG_ERROR, "block_max_depth= %d is too large\n", f->obmc.block_max_depth);
        f->obmc.block_max_depth= 0;
        return AVERROR_INVALIDDATA;
    }
    return 0;
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    FFV1Context *f = avctx->priv_data;
    int ret;

    if ((ret = ff_ffv1_common_init(avctx)) < 0)
        return ret;

    if (avctx->extradata_size > 0 && (ret = read_extra_header(f)) < 0)
        return ret;

    if ((ret = ff_ffv1_init_slice_contexts(f)) < 0)
        return ret;

    avctx->internal->allocate_progress = 1;

    return 0;
}

static int decode_frame(AVCodecContext *avctx, void *data, int *got_frame, AVPacket *avpkt)
{
    uint8_t *buf        = avpkt->data;
    int buf_size        = avpkt->size;
    FFV1Context *f      = avctx->priv_data;
    RangeCoder *const c = &f->slice_context[0]->c;
    int i, ret, plane_index;
    uint8_t keystate = 128;
    uint8_t *buf_p;
    AVFrame *p;

    if (f->last_picture.f)
        ff_thread_release_buffer(avctx, &f->last_picture);
    FFSWAP(ThreadFrame, f->picture, f->last_picture);

    f->cur = p = f->picture.f;

    if (f->version < 3 && avctx->field_order > AV_FIELD_PROGRESSIVE) {
        /* we have interlaced material flagged in container */
        p->interlaced_frame = 1;
        if (avctx->field_order == AV_FIELD_TT || avctx->field_order == AV_FIELD_TB)
            p->top_field_first = 1;
    }

    f->avctx = avctx;
    ff_init_range_decoder(c, buf, buf_size);
    ff_build_rac_states(c, 0.05 * (1LL << 32), 256 - 8);

    p->pict_type = AV_PICTURE_TYPE_I; //FIXME I vs. P
    f->obmc.current_picture->pict_type = AV_PICTURE_TYPE_I;
    if (get_rac(c, &keystate)) {
        p->key_frame    = 1;
        f->obmc.keyframe = f->key_frame = 1;
        f->key_frame_ok = 0;
        if ((ret = read_header(f)) < 0)
            return ret;
        f->key_frame_ok = 1;
    } else {
        if (!f->key_frame_ok) {
            av_log(avctx, AV_LOG_ERROR,
                   "Cannot decode non-keyframe without valid keyframe\n");
            return AVERROR_INVALIDDATA;
        }
        p->key_frame = 0;
        f->obmc.keyframe = f->key_frame = 0;
    }

    if (f->p_frame) {
        if ((ret = decode_p_header(f)) < 0)
            return ret;

        p->pict_type = p->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

        if ((ret=ff_obmc_common_init_after_header(&f->obmc)) < 0)
            return ret;
    }

    if ((ret = ff_thread_get_buffer(avctx, &f->picture, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;

    if (avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(avctx, AV_LOG_DEBUG, "ver:%d keyframe:%d coder:%d ec:%d slices:%d bps:%d\n",
               f->version, p->key_frame, f->ac, f->ec, f->slice_count, f->avctx->bits_per_raw_sample);

    if (f->p_frame) {
        if ((ret = ff_obmc_predecode_frame(&f->obmc)) < 0)
            return ret;

        if ((ret = decode_blocks(f)) < 0)
            return ret;
    }

    ff_thread_finish_setup(avctx);

    buf_p = buf + buf_size;
    for (i = f->slice_count - 1; i >= 0; i--) {
        FFV1Context *fs = f->slice_context[i];
        int trailer = 3 + 5*!!f->ec;
        int v;

        if (i || f->version > 2) v = AV_RB24(buf_p-trailer) + trailer;
        else                     v = buf_p - c->bytestream_start;
        if (buf_p - c->bytestream_start < v) {
            av_log(avctx, AV_LOG_ERROR, "Slice pointer chain broken\n");
            ff_thread_report_progress(&f->picture, INT_MAX, 0);
            return AVERROR_INVALIDDATA;
        }
        buf_p -= v;

        if (f->ec) {
            unsigned crc = av_crc(av_crc_get_table(AV_CRC_32_IEEE), 0, buf_p, v);
            if (crc) {
                int64_t ts = avpkt->pts != AV_NOPTS_VALUE ? avpkt->pts : avpkt->dts;
                av_log(f->avctx, AV_LOG_ERROR, "CRC mismatch %X!", crc);
                if (ts != AV_NOPTS_VALUE && avctx->pkt_timebase.num) {
                    av_log(f->avctx, AV_LOG_ERROR, "at %f seconds\n", ts*av_q2d(avctx->pkt_timebase));
                } else if (ts != AV_NOPTS_VALUE) {
                    av_log(f->avctx, AV_LOG_ERROR, "at %"PRId64"\n", ts);
                } else {
                    av_log(f->avctx, AV_LOG_ERROR, "\n");
                }
                fs->slice_damaged = 1;
            }
            if (avctx->debug & FF_DEBUG_PICT_INFO) {
                av_log(avctx, AV_LOG_DEBUG, "slice %d, CRC: 0x%08X\n", i, AV_RB32(buf_p + v - 4));
            }
        }

        if (i) {
            ff_init_range_decoder(&fs->c, buf_p, v);
        } else
            fs->c.bytestream_end = buf_p + v;

        fs->avctx = avctx;
        fs->cur = p;
    }

    avctx->execute(avctx,
                   decode_slice,
                   &f->slice_context[0],
                   NULL,
                   f->slice_count,
                   sizeof(void*));

    for (i = f->slice_count - 1; i >= 0; i--) {
        FFV1Context *fs = f->slice_context[i];
        int j;
        if (fs->slice_damaged && f->last_picture.f->data[0]) {
            const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(avctx->pix_fmt);
            const uint8_t *src[4];
            uint8_t *dst[4];
            ff_thread_await_progress(&f->last_picture, INT_MAX, 0);
            for (j = 0; j < 4; j++) {
                int pixshift = desc->comp[j].depth > 8;
                int sh = (j == 1 || j == 2) ? f->chroma_h_shift : 0;
                int sv = (j == 1 || j == 2) ? f->chroma_v_shift : 0;
                dst[j] = p->data[j] + p->linesize[j] *
                         (fs->slice_y >> sv) + ((fs->slice_x >> sh) << pixshift);
                src[j] = f->last_picture.f->data[j] + f->last_picture.f->linesize[j] *
                         (fs->slice_y >> sv) + ((fs->slice_x >> sh) << pixshift);
            }
            av_image_copy(dst, p->linesize, src,
                          f->last_picture.f->linesize,
                          avctx->pix_fmt,
                          fs->slice_width,
                          fs->slice_height);
        }
    }

    if (f->p_frame) {
        ff_thread_await_progress(&f->last_picture, INT_MAX, 0);

        av_frame_copy(f->obmc.last_pictures[1], f->last_picture.f);

        for (plane_index=0; plane_index < f->obmc.nb_planes; plane_index++) {
            PlaneObmc *pc = &f->obmc.plane[plane_index];
            int w = pc->width;
            int h = pc->height;

            if(!p->key_frame){
                memset(f->obmc.spatial_idwt_buffer, 0, sizeof(IDWTELEM)*w*h);
                predict_plane(&f->obmc, f->obmc.spatial_idwt_buffer, plane_index, 1);
            }
        }

        if (!p->key_frame) {
            if ((ret = ff_predict_frame(avctx, f)) < 0) {
                ff_thread_report_progress(&f->picture, INT_MAX, 0);
                return ret;
            }
        }
        av_frame_copy(f->obmc.current_picture, f->picture.f);
    }
    ff_obmc_release_buffer(&f->obmc);

    ff_thread_report_progress(&f->picture, INT_MAX, 0);

    f->picture_number++;

    if (f->last_picture.f)
        ff_thread_release_buffer(avctx, &f->last_picture);
    f->cur = NULL;
    if ((ret = av_frame_ref(data, f->picture.f)) < 0)
        return ret;

    *got_frame = 1;

    return buf_size;
}

#if HAVE_THREADS
static int init_thread_copy(AVCodecContext *avctx)
{
    FFV1Context *f = avctx->priv_data;
    int i, ret;

    f->picture.f      = NULL;
    f->last_picture.f = NULL;
    f->residual.f     = NULL;
    f->sample_buffer  = NULL;
    f->max_slice_count = 0;
    f->slice_count = 0;

    f->obmc.current_picture = NULL;
    for (i = 0; i < MAX_REF_FRAMES; i++)
        f->obmc.last_pictures[i] = NULL;

    f->p_image_line_buf = NULL;
    f->c_image_line_buf = NULL;

    for (i = 0; i < f->quant_table_count; i++) {
        av_assert0(f->version > 1);
        f->initial_states[i] = av_memdup(f->initial_states[i],
                                         f->context_count[i] * sizeof(*f->initial_states[i]));
    }

    f->picture.f      = av_frame_alloc();
    f->last_picture.f = av_frame_alloc();
    f->residual.f     = av_frame_alloc();

    if (!f->picture.f || !f->last_picture.f || !f->residual.f)
        goto fail;

    f->obmc.current_picture = av_frame_alloc();
    f->obmc.mconly_picture = av_frame_alloc();

    f->width  = avctx->width;
    f->height = avctx->height;

    FF_ALLOCZ_ARRAY_OR_GOTO(avctx, f->obmc.spatial_idwt_buffer, f->width, f->height * sizeof(IDWTELEM), fail);

    for (i = 0; i < MAX_REF_FRAMES; i++)
        f->obmc.last_pictures[i] = av_frame_alloc();

    int w= AV_CEIL_RSHIFT(avctx->width,  LOG2_MB_SIZE);
    int h= AV_CEIL_RSHIFT(avctx->height, LOG2_MB_SIZE);

    f->obmc.b_width = w;
    f->obmc.b_height= h;

    f->obmc.block = av_mallocz_array(w * h,  sizeof(BlockNode) << 2); // FIXME Maybe large

    f->obmc.avctx = avctx;

    f->obmc.chroma_h_shift = f->chroma_h_shift;
    f->obmc.chroma_v_shift = f->chroma_v_shift;

    f->p_image_line_buf = av_mallocz_array(sizeof(*f->p_image_line_buf), 2 * f->width);
    f->c_image_line_buf = av_mallocz_array(sizeof(*f->c_image_line_buf), 2 * f->width);

    if (!f->p_image_line_buf || !f->c_image_line_buf)
        goto fail;

    if ((ret = ff_ffv1_init_slice_contexts(f)) < 0)
        return ret;

    return 0;
fail:
    return AVERROR(ENOMEM);
}
#endif

static void copy_fields(FFV1Context *fsdst, FFV1Context *fssrc, FFV1Context *fsrc)
{
    fsdst->version             = fsrc->version;
    fsdst->micro_version       = fsrc->micro_version;
    fsdst->chroma_planes       = fsrc->chroma_planes;
    fsdst->chroma_h_shift      = fsrc->chroma_h_shift;
    fsdst->chroma_v_shift      = fsrc->chroma_v_shift;
    fsdst->transparency        = fsrc->transparency;
    fsdst->plane_count         = fsrc->plane_count;
    fsdst->ac                  = fsrc->ac;
    fsdst->colorspace          = fsrc->colorspace;

    fsdst->ec                  = fsrc->ec;
    fsdst->p_frame             = fsrc->p_frame;
    fsdst->intra               = fsrc->intra;
    fsdst->slice_damaged       = fssrc->slice_damaged;
    fsdst->key_frame_ok        = fsrc->key_frame_ok;

    fsdst->bits_per_raw_sample = fsrc->bits_per_raw_sample;
    fsdst->packed_at_lsb       = fsrc->packed_at_lsb;
    fsdst->slice_count         = fsrc->slice_count;
    if (fsrc->version<3){
        fsdst->slice_x             = fssrc->slice_x;
        fsdst->slice_y             = fssrc->slice_y;
        fsdst->slice_width         = fssrc->slice_width;
        fsdst->slice_height        = fssrc->slice_height;
    }
}

#if HAVE_THREADS
static int update_thread_context(AVCodecContext *dst, const AVCodecContext *src)
{
    FFV1Context *fsrc = src->priv_data;
    FFV1Context *fdst = dst->priv_data;
    int i, j, ret;

    if (dst == src)
        return 0;

    {
        ThreadFrame picture = fdst->picture, last_picture = fdst->last_picture, residual = fdst->residual;
        uint16_t *c_image_line_buf = fdst->c_image_line_buf, *p_image_line_buf = fdst->p_image_line_buf;
        uint8_t (*initial_states[MAX_QUANT_TABLES])[32];
        struct FFV1Context *slice_context[MAX_SLICES];
        memcpy(initial_states, fdst->initial_states, sizeof(fdst->initial_states));
        memcpy(slice_context,  fdst->slice_context , sizeof(fdst->slice_context));
        AVFrame *current_picture = fdst->obmc.current_picture, *mconly_picture = fdst->obmc.mconly_picture;
        AVFrame *last_pictures[MAX_REF_FRAMES];
        BlockNode *block = fdst->obmc.block;
        uint8_t *scratchbuf = fdst->obmc.scratchbuf;
        uint8_t *emu_edge_buffer = fdst->obmc.emu_edge_buffer;
        IDWTELEM *spatial_idwt_buffer = fdst->obmc.spatial_idwt_buffer;
        for (i = 0; i < MAX_REF_FRAMES; i++)
            last_pictures[i] = fdst->obmc.last_pictures[i];

        memcpy(fdst, fsrc, sizeof(*fdst));
        memcpy(fdst->initial_states, initial_states, sizeof(fdst->initial_states));
        memcpy(fdst->slice_context,  slice_context , sizeof(fdst->slice_context));
        fdst->picture      = picture;
        fdst->last_picture = last_picture;
        fdst->residual     = residual;

        fdst->p_image_line_buf = p_image_line_buf;
        fdst->c_image_line_buf = c_image_line_buf;

        fdst->obmc.current_picture   = current_picture;
        fdst->obmc.mconly_picture    = mconly_picture;
        for (i = 0; i < MAX_REF_FRAMES; i++)
            fdst->obmc.last_pictures[i] = last_pictures[i];
        fdst->obmc.block = block;
        fdst->obmc.scratchbuf = scratchbuf;
        fdst->obmc.emu_edge_buffer = emu_edge_buffer;
        fdst->obmc.spatial_idwt_buffer = spatial_idwt_buffer;

        fdst->obmc.avctx = dst;

        for (i = 0; i<fdst->num_h_slices * fdst->num_v_slices; i++) {
            FFV1Context *fssrc = fsrc->slice_context[i];
            FFV1Context *fsdst = fdst->slice_context[i];
            copy_fields(fsdst, fssrc, fsrc);
        }
        av_assert0(!fdst->plane[0].state);
        av_assert0(!fdst->sample_buffer);
    }

    av_assert1(fdst->max_slice_count == fsrc->max_slice_count);


    ff_thread_release_buffer(dst, &fdst->picture);
    if (fsrc->picture.f->data[0]) {
        if ((ret = ff_thread_ref_frame(&fdst->picture, &fsrc->picture)) < 0)
            return ret;
    }

    for (i = 0; i < MAX_REF_FRAMES; i++)
        av_frame_ref(fdst->obmc.last_pictures[i], fsrc->obmc.last_pictures[i]);

    av_frame_ref(fdst->obmc.current_picture, fsrc->obmc.current_picture);

    for (i = 0; i < MAX_REF_FRAMES; i++) {
        for (j=0; j<9; j++) {
            int is_chroma = !!(j%3);
            int h = is_chroma ? AV_CEIL_RSHIFT(fsrc->avctx->height, fsrc->chroma_v_shift) : fsrc->avctx->height;
            int ls = fdst->obmc.last_pictures[i]->linesize[j%3];
            if (fsrc->obmc.halfpel_plane[i][1+j/3][j%3]) {
                fdst->obmc.halfpel_plane[i][1+j/3][j%3] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
                memcpy(
                    fdst->obmc.halfpel_plane[i][1+j/3][j%3],
                    fsrc->obmc.halfpel_plane[i][1+j/3][j%3] - EDGE_WIDTH*(1+fsrc->obmc.last_pictures[i]->linesize[j%3]),
                    ls * (h + 2 * EDGE_WIDTH) * sizeof(*fdst->obmc.halfpel_plane[i][1+j/3][j%3])
                );
                fdst->obmc.halfpel_plane[i][1+j/3][j%3] += EDGE_WIDTH * (1 + ls);
            }
            fdst->obmc.halfpel_plane[i][0][j%3] = fdst->obmc.last_pictures[i]->data[j%3];
        }
    }

    memcpy(
        fdst->obmc.block,
        fsrc->obmc.block,
        (fsrc->obmc.b_width * fsrc->obmc.b_height * (sizeof(BlockNode) << (fsrc->obmc.block_max_depth*2)))
    );

    fdst->fsrc = fsrc;

    return 0;
}
#endif

AVCodec ff_ffv1_decoder = {
    .name           = "ffv1",
    .long_name      = NULL_IF_CONFIG_SMALL("FFmpeg video codec #1"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_FFV1,
    .priv_data_size = sizeof(FFV1Context),
    .init           = decode_init,
    .close          = ff_ffv1_close,
    .decode         = decode_frame,
    .init_thread_copy = ONLY_IF_THREADS_ENABLED(init_thread_copy),
    .update_thread_context = ONLY_IF_THREADS_ENABLED(update_thread_context),
    .capabilities   = AV_CODEC_CAP_DR1 /*| AV_CODEC_CAP_DRAW_HORIZ_BAND*/ |
                      AV_CODEC_CAP_FRAME_THREADS | AV_CODEC_CAP_SLICE_THREADS,
    .caps_internal  = FF_CODEC_CAP_INIT_CLEANUP
};
