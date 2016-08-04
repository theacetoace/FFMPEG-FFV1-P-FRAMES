/*
 * FFV1 encoder
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
 * FF Video Codec 1 (a lossless codec) encoder
 */

#include "libavutil/attributes.h"
#include "libavutil/avassert.h"
#include "libavutil/crc.h"
#include "libavutil/opt.h"
#include "libavutil/imgutils.h"
#include "libavutil/pixdesc.h"
#include "libavutil/timer.h"
#include "avcodec.h"
#include "internal.h"
#include "put_bits.h"
#include "rangecoder.h"
#include "golomb.h"
#include "mathops.h"
#include "ffv1.h"

#include "mpegvideo.h"
#include "h263.h"

#define FF_ME_ITER 50

static const int8_t quant5_10bit[256] = {
     0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
     1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
     1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -0, -0, -0, -0, -0, -0, -0, -0, -0, -0,
};

static const int8_t quant5[256] = {
     0,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -1, -1, -1,
};

static const int8_t quant9_10bit[256] = {
     0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,
     2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,  3,
     3,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3,
    -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3, -3,
    -3, -3, -3, -3, -3, -3, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2,
    -2, -2, -2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -0, -0, -0, -0,
};

static const int8_t quant11[256] = {
     0,  1,  2,  2,  2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,
     4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,  4,
     4,  4,  4,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
     5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
     5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
     5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
     5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
     5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5,
    -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -5, -4, -4,
    -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4,
    -4, -4, -4, -4, -4, -3, -3, -3, -3, -3, -3, -3, -2, -2, -2, -1,
};

static const uint8_t ver2_state[256] = {
      0,  10,  10,  10,  10,  16,  16,  16, 28,   16,  16,  29,  42,  49,  20,  49,
     59,  25,  26,  26,  27,  31,  33,  33, 33,   34,  34,  37,  67,  38,  39,  39,
     40,  40,  41,  79,  43,  44,  45,  45, 48,   48,  64,  50,  51,  52,  88,  52,
     53,  74,  55,  57,  58,  58,  74,  60, 101,  61,  62,  84,  66,  66,  68,  69,
     87,  82,  71,  97,  73,  73,  82,  75, 111,  77,  94,  78,  87,  81,  83,  97,
     85,  83,  94,  86,  99,  89,  90,  99, 111,  92,  93,  134, 95,  98,  105, 98,
    105, 110, 102, 108, 102, 118, 103, 106, 106, 113, 109, 112, 114, 112, 116, 125,
    115, 116, 117, 117, 126, 119, 125, 121, 121, 123, 145, 124, 126, 131, 127, 129,
    165, 130, 132, 138, 133, 135, 145, 136, 137, 139, 146, 141, 143, 142, 144, 148,
    147, 155, 151, 149, 151, 150, 152, 157, 153, 154, 156, 168, 158, 162, 161, 160,
    172, 163, 169, 164, 166, 184, 167, 170, 177, 174, 171, 173, 182, 176, 180, 178,
    175, 189, 179, 181, 186, 183, 192, 185, 200, 187, 191, 188, 190, 197, 193, 196,
    197, 194, 195, 196, 198, 202, 199, 201, 210, 203, 207, 204, 205, 206, 208, 214,
    209, 211, 221, 212, 213, 215, 224, 216, 217, 218, 219, 220, 222, 228, 223, 225,
    226, 224, 227, 229, 240, 230, 231, 232, 233, 234, 235, 236, 238, 239, 237, 242,
    241, 243, 242, 244, 245, 246, 247, 248, 249, 250, 251, 252, 252, 253, 254, 255,
};

static int ff_frame_diff(FFV1Context *f, const AVFrame *pict)
{
    int ret, i, x, y;
    //AVFrame *curr     = f->picture.f;
    AVFrame *prev     = f->current_picture;//f->last_picture.f;
    AVFrame *residual = f->residual.f;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(prev->format);
    int width  = f->width;
    int height = f->height;
    int has_plane[4] = { 0 };
    const int cw = AV_CEIL_RSHIFT(width, desc->log2_chroma_w);
    const int ch = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);

    if (f->picture.f)
        av_frame_unref(f->picture.f);
    if (f->residual.f)
        av_frame_unref(f->residual.f);
    if ((ret = av_frame_ref(f->residual.f, pict)) < 0)
        return ret;
    if ((ret = av_frame_make_writable(f->residual.f)) < 0) {
        av_frame_unref(f->residual.f);
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
            /*memcpy(
                f->p_image_line_buf + width,
                f->c_image_line_buf + width,
                width * sizeof(*f->p_image_line_buf)
            );*/
            memset(f->p_image_line_buf, 0, width * sizeof(*f->p_image_line_buf));
            memset(f->c_image_line_buf, 0, width * sizeof(*f->c_image_line_buf));
            av_read_image_line(f->c_image_line_buf,
                               (void *)pict->data,
                               pict->linesize,
                               desc,
                               0, y, i, w1, 0);
            av_read_image_line(f->p_image_line_buf,
                              (void *)prev->data,
                              prev->linesize,
                              desc,
                              0, y, i, w1, 0);
            for (x = 0; x < w1; ++x) {
                /*uint16_t mid = mid_pred(
                    x ? f->c_image_line_buf[width + x - 1] : 0,
                    x ? f->p_image_line_buf[width + x - 1] : 0,
                    f->p_image_line_buf[width + x]
                );
                f->c_image_line_buf[width + x] = f->c_image_line_buf[x] - f->p_image_line_buf[x];
                f->c_image_line_buf[x] = (f->c_image_line_buf[width + x] - mid + (max_val >> 2)) & (max_val - 1);*/
                f->c_image_line_buf[x] = (f->c_image_line_buf[x] - f->p_image_line_buf[x] + (max_val >> 2)) & (max_val - 1);
            }
            av_write_image_line(f->c_image_line_buf, //CHANGED
                                residual->data,
                                residual->linesize,
                                desc,
                                0, y, i, w1);
        }
    }

    if ((ret = av_frame_ref(f->picture.f, f->residual.f)) < 0)
        return ret;

    return 0;
}

static void find_best_state(uint8_t best_state[256][256],
                            const uint8_t one_state[256])
{
    int i, j, k, m;
    double l2tab[256];

    for (i = 1; i < 256; i++)
        l2tab[i] = log2(i / 256.0);

    for (i = 0; i < 256; i++) {
        double best_len[256];
        double p = i / 256.0;

        for (j = 0; j < 256; j++)
            best_len[j] = 1 << 30;

        for (j = FFMAX(i - 10, 1); j < FFMIN(i + 11, 256); j++) {
            double occ[256] = { 0 };
            double len      = 0;
            occ[j] = 1.0;

            if (!one_state[j])
                continue;

            for (k = 0; k < 256; k++) {
                double newocc[256] = { 0 };
                for (m = 1; m < 256; m++)
                    if (occ[m]) {
                        len -=occ[m]*(     p *l2tab[    m]
                                      + (1-p)*l2tab[256-m]);
                    }
                if (len < best_len[k]) {
                    best_len[k]      = len;
                    best_state[i][k] = j;
                }
                for (m = 1; m < 256; m++)
                    if (occ[m]) {
                        newocc[      one_state[      m]] += occ[m] * p;
                        newocc[256 - one_state[256 - m]] += occ[m] * (1 - p);
                    }
                memcpy(occ, newocc, sizeof(occ));
            }
        }
    }
}

static av_always_inline av_flatten void put_symbol_inline(RangeCoder *c,
                                                          uint8_t *state, int v,
                                                          int is_signed,
                                                          uint64_t rc_stat[256][2],
                                                          uint64_t rc_stat2[32][2])
{
    int i;

#define put_rac(C, S, B)                        \
    do {                                        \
        if (rc_stat) {                          \
            rc_stat[*(S)][B]++;                 \
            rc_stat2[(S) - state][B]++;         \
        }                                       \
        put_rac(C, S, B);                       \
    } while (0)

    if (v) {
        const int a = FFABS(v);
        const int e = av_log2(a);
        put_rac(c, state + 0, 0);
        if (e <= 9) {
            for (i = 0; i < e; i++)
                put_rac(c, state + 1 + i, 1);  // 1..10
            put_rac(c, state + 1 + i, 0);

            for (i = e - 1; i >= 0; i--)
                put_rac(c, state + 22 + i, (a >> i) & 1);  // 22..31

            if (is_signed)
                put_rac(c, state + 11 + e, v < 0);  // 11..21
        } else {
            for (i = 0; i < e; i++)
                put_rac(c, state + 1 + FFMIN(i, 9), 1);  // 1..10
            put_rac(c, state + 1 + 9, 0);

            for (i = e - 1; i >= 0; i--)
                put_rac(c, state + 22 + FFMIN(i, 9), (a >> i) & 1);  // 22..31

            if (is_signed)
                put_rac(c, state + 11 + 10, v < 0);  // 11..21
        }
    } else {
        put_rac(c, state + 0, 1);
    }
#undef put_rac
}

static av_noinline void put_symbol(RangeCoder *c, uint8_t *state,
                                   int v, int is_signed)
{
    put_symbol_inline(c, state, v, is_signed, NULL, NULL);
}


static inline void put_vlc_symbol(PutBitContext *pb, VlcState *const state,
                                  int v, int bits)
{
    int i, k, code;
    v = fold(v - state->bias, bits);

    i = state->count;
    k = 0;
    while (i < state->error_sum) { // FIXME: optimize
        k++;
        i += i;
    }

    av_assert2(k <= 13);

#if 0 // JPEG LS
    if (k == 0 && 2 * state->drift <= -state->count)
        code = v ^ (-1);
    else
        code = v;
#else
    code = v ^ ((2 * state->drift + state->count) >> 31);
#endif

    ff_dlog(NULL, "v:%d/%d bias:%d error:%d drift:%d count:%d k:%d\n", v, code,
            state->bias, state->error_sum, state->drift, state->count, k);
    set_sr_golomb(pb, code, k, 12, bits);

    update_vlc_state(state, v);
}

static av_always_inline int encode_line(FFV1Context *s, int w,
                                        int16_t *sample[3],
                                        int plane_index, int bits)
{
    PlaneContext *const p = &s->plane[plane_index];
    RangeCoder *const c   = &s->c;
    int x;
    int run_index = s->run_index;
    int run_count = 0;
    int run_mode  = 0;

    if (s->ac != AC_GOLOMB_RICE) {
        if (c->bytestream_end - c->bytestream < w * 35) {
            av_log(s->avctx, AV_LOG_ERROR, "encoded frame too large\n");
            return AVERROR_INVALIDDATA;
        }
    } else {
        if (s->pb.buf_end - s->pb.buf - (put_bits_count(&s->pb) >> 3) < w * 4) {
            av_log(s->avctx, AV_LOG_ERROR, "encoded frame too large\n");
            return AVERROR_INVALIDDATA;
        }
    }

    if (s->slice_coding_mode == 1) {
        for (x = 0; x < w; x++) {
            int i;
            int v = sample[0][x];
            for (i = bits-1; i>=0; i--) {
                uint8_t state = 128;
                put_rac(c, &state, (v>>i) & 1);
            }
        }
        return 0;
    }

    for (x = 0; x < w; x++) {
        int diff, context;

        context = get_context(p, sample[0] + x, sample[1] + x, sample[2] + x);
        diff    = sample[0][x] - predict(sample[0] + x, sample[1] + x);

        if (context < 0) {
            context = -context;
            diff    = -diff;
        }

        diff = fold(diff, bits);

        if (s->ac != AC_GOLOMB_RICE) {
            if (s->flags & AV_CODEC_FLAG_PASS1) {
                put_symbol_inline(c, p->state[context], diff, 1, s->rc_stat,
                                  s->rc_stat2[p->quant_table_index][context]);
            } else {
                put_symbol_inline(c, p->state[context], diff, 1, NULL, NULL);
            }
        } else {
            if (context == 0)
                run_mode = 1;

            if (run_mode) {
                if (diff) {
                    while (run_count >= 1 << ff_log2_run[run_index]) {
                        run_count -= 1 << ff_log2_run[run_index];
                        run_index++;
                        put_bits(&s->pb, 1, 1);
                    }

                    put_bits(&s->pb, 1 + ff_log2_run[run_index], run_count);
                    if (run_index)
                        run_index--;
                    run_count = 0;
                    run_mode  = 0;
                    if (diff > 0)
                        diff--;
                } else {
                    run_count++;
                }
            }

            ff_dlog(s->avctx, "count:%d index:%d, mode:%d, x:%d pos:%d\n",
                    run_count, run_index, run_mode, x,
                    (int)put_bits_count(&s->pb));

            if (run_mode == 0)
                put_vlc_symbol(&s->pb, &p->vlc_state[context], diff, bits);
        }
    }
    if (run_mode) {
        while (run_count >= 1 << ff_log2_run[run_index]) {
            run_count -= 1 << ff_log2_run[run_index];
            run_index++;
            put_bits(&s->pb, 1, 1);
        }

        if (run_count)
            put_bits(&s->pb, 1, 1);
    }
    s->run_index = run_index;

    return 0;
}

static int encode_plane(FFV1Context *s, uint8_t *src, int w, int h,
                         int stride, int plane_index, int pixel_stride)
{
    int x, y, i, ret;
    const int ring_size = s->context_model ? 3 : 2;
    int16_t *sample[3];
    s->run_index = 0;

    memset(s->sample_buffer, 0, ring_size * (w + 6) * sizeof(*s->sample_buffer));

    for (y = 0; y < h; y++) {
        for (i = 0; i < ring_size; i++)
            sample[i] = s->sample_buffer + (w + 6) * ((h + i - y) % ring_size) + 3;

        sample[0][-1]= sample[1][0  ];
        sample[1][ w]= sample[1][w-1];
// { START_TIMER
        if (s->bits_per_raw_sample <= 8) {
            for (x = 0; x < w; x++)
                sample[0][x] = src[x * pixel_stride + stride * y];
            if((ret = encode_line(s, w, sample, plane_index, 8)) < 0)
                return ret;
        } else {
            if (s->packed_at_lsb) {
                for (x = 0; x < w; x++) {
                    sample[0][x] = ((uint16_t*)(src + stride*y))[x];
                }
            } else {
                for (x = 0; x < w; x++) {
                    sample[0][x] = ((uint16_t*)(src + stride*y))[x] >> (16 - s->bits_per_raw_sample);
                }
            }
            if((ret = encode_line(s, w, sample, plane_index, s->bits_per_raw_sample)) < 0)
                return ret;
        }
// STOP_TIMER("encode line") }
    }
    return 0;
}

static int encode_rgb_frame(FFV1Context *s, const uint8_t *src[3],
                             int w, int h, const int stride[3])
{
    int x, y, p, i;
    const int ring_size = s->context_model ? 3 : 2;
    int16_t *sample[4][3];
    int lbd    = s->bits_per_raw_sample <= 8;
    int bits   = s->bits_per_raw_sample > 0 ? s->bits_per_raw_sample : 8;
    int offset = 1 << bits;

    s->run_index = 0;

    memset(s->sample_buffer, 0, ring_size * MAX_PLANES *
                                (w + 6) * sizeof(*s->sample_buffer));

    for (y = 0; y < h; y++) {
        for (i = 0; i < ring_size; i++)
            for (p = 0; p < MAX_PLANES; p++)
                sample[p][i]= s->sample_buffer + p*ring_size*(w+6) + ((h+i-y)%ring_size)*(w+6) + 3;

        for (x = 0; x < w; x++) {
            int b, g, r, av_uninit(a);
            if (lbd) {
                unsigned v = *((const uint32_t*)(src[0] + x*4 + stride[0]*y));
                b =  v        & 0xFF;
                g = (v >>  8) & 0xFF;
                r = (v >> 16) & 0xFF;
                a =  v >> 24;
            } else {
                b = *((const uint16_t *)(src[0] + x*2 + stride[0]*y));
                g = *((const uint16_t *)(src[1] + x*2 + stride[1]*y));
                r = *((const uint16_t *)(src[2] + x*2 + stride[2]*y));
            }

            if (s->slice_coding_mode != 1) {
                b -= g;
                r -= g;
                g += (b * s->slice_rct_by_coef + r * s->slice_rct_ry_coef) >> 2;
                b += offset;
                r += offset;
            }

            sample[0][0][x] = g;
            sample[1][0][x] = b;
            sample[2][0][x] = r;
            sample[3][0][x] = a;
        }
        for (p = 0; p < 3 + s->transparency; p++) {
            int ret;
            sample[p][0][-1] = sample[p][1][0  ];
            sample[p][1][ w] = sample[p][1][w-1];
            if (lbd && s->slice_coding_mode == 0)
                ret = encode_line(s, w, sample[p], (p + 1) / 2, 9);
            else
                ret = encode_line(s, w, sample[p], (p + 1) / 2, bits + (s->slice_coding_mode != 1));
            if (ret < 0)
                return ret;
        }
    }
    return 0;
}

static void write_quant_table(RangeCoder *c, int16_t *quant_table)
{
    int last = 0;
    int i;
    uint8_t state[CONTEXT_SIZE];
    memset(state, 128, sizeof(state));

    for (i = 1; i < 128; i++)
        if (quant_table[i] != quant_table[i - 1]) {
            put_symbol(c, state, i - last - 1, 0);
            last = i;
        }
    put_symbol(c, state, i - last - 1, 0);
}

static void write_quant_tables(RangeCoder *c,
                               int16_t quant_table[MAX_CONTEXT_INPUTS][256])
{
    int i;
    for (i = 0; i < 5; i++)
        write_quant_table(c, quant_table[i]);
}

static void write_header(FFV1Context *f)
{
    uint8_t state[CONTEXT_SIZE];
    int i, j;
    RangeCoder *const c = &f->slice_context[0]->c;

    memset(state, 128, sizeof(state));

    if (f->version < 2) {
        put_symbol(c, state, f->version, 0);
        put_symbol(c, state, f->ac, 0);
        if (f->ac == AC_RANGE_CUSTOM_TAB) {
            for (i = 1; i < 256; i++)
                put_symbol(c, state,
                           f->state_transition[i] - c->one_state[i], 1);
        }
        put_symbol(c, state, f->colorspace, 0); //YUV cs type
        if (f->version > 0)
            put_symbol(c, state, f->bits_per_raw_sample, 0);
        put_rac(c, state, f->chroma_planes);
        put_symbol(c, state, f->chroma_h_shift, 0);
        put_symbol(c, state, f->chroma_v_shift, 0);
        put_rac(c, state, f->transparency);

        write_quant_tables(c, f->quant_table);
    } else if (f->version < 3) {
        put_symbol(c, state, f->slice_count, 0);
        for (i = 0; i < f->slice_count; i++) {
            FFV1Context *fs = f->slice_context[i];
            put_symbol(c, state,
                       (fs->slice_x      + 1) * f->num_h_slices / f->width, 0);
            put_symbol(c, state,
                       (fs->slice_y      + 1) * f->num_v_slices / f->height, 0);
            put_symbol(c, state,
                       (fs->slice_width  + 1) * f->num_h_slices / f->width - 1,
                       0);
            put_symbol(c, state,
                       (fs->slice_height + 1) * f->num_v_slices / f->height - 1,
                       0);
            for (j = 0; j < f->plane_count; j++) {
                put_symbol(c, state, f->plane[j].quant_table_index, 0);
                av_assert0(f->plane[j].quant_table_index == f->context_model);
            }
        }
    }
}

static void write_p_header(FFV1Context *f)
{
    uint8_t state[CONTEXT_SIZE];
    int i, plane_index;
    RangeCoder *const c = &f->slice_context[0]->c;

    memset(state, 128, sizeof(state));

    if (f->key_frame) {
        ff_ffv1_reset_contexts(f);
        put_symbol(c, state, f->max_ref_frames-1, 0);
    }
    if (!f->key_frame) { //FIXME update_mc
        for(plane_index=0; plane_index<FFMIN(f->nb_planes, 2); plane_index++){
            PlaneContext *p= &f->plane[plane_index];
            put_rac(c, state, p->diag_mc);
            put_symbol(c, state, p->htaps/2-1, 0);
            for(i= p->htaps/2; i; i--)
                put_symbol(c, state, FFABS(p->hcoeff[i]), 0);
        }
    }

    put_symbol(c, state, f->mv_scale, 0);
    put_symbol(c, state, f->block_max_depth, 0);
}

static int write_extradata(FFV1Context *f)
{
    RangeCoder *const c = &f->c;
    uint8_t state[CONTEXT_SIZE];
    int i, j, k;
    uint8_t state2[32][CONTEXT_SIZE];
    unsigned v;

    memset(state2, 128, sizeof(state2));
    memset(state, 128, sizeof(state));

    f->avctx->extradata_size = 10000 + 4 +
                                    (11 * 11 * 5 * 5 * 5 + 11 * 11 * 11) * 32;
    f->avctx->extradata = av_malloc(f->avctx->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!f->avctx->extradata)
        return AVERROR(ENOMEM);
    ff_init_range_encoder(c, f->avctx->extradata, f->avctx->extradata_size);
    ff_build_rac_states(c, 0.05 * (1LL << 32), 256 - 8);

    put_symbol(c, state, f->version, 0);
    if (f->version > 2) {
        if (f->version == 3) {
            f->micro_version = 4 + f->p_frame;
        } else if (f->version == 4)
            f->micro_version = 2 + f->p_frame;
        put_symbol(c, state, f->micro_version, 0);
    }

    put_symbol(c, state, f->ac, 0);
    if (f->ac == AC_RANGE_CUSTOM_TAB)
        for (i = 1; i < 256; i++)
            put_symbol(c, state, f->state_transition[i] - c->one_state[i], 1);

    put_symbol(c, state, f->colorspace, 0); // YUV cs type
    put_symbol(c, state, f->bits_per_raw_sample, 0);
    put_rac(c, state, f->chroma_planes);
    put_symbol(c, state, f->chroma_h_shift, 0);
    put_symbol(c, state, f->chroma_v_shift, 0);
    put_rac(c, state, f->transparency);
    put_symbol(c, state, f->num_h_slices - 1, 0);
    put_symbol(c, state, f->num_v_slices - 1, 0);

    put_symbol(c, state, f->quant_table_count, 0);
    for (i = 0; i < f->quant_table_count; i++)
        write_quant_tables(c, f->quant_tables[i]);

    for (i = 0; i < f->quant_table_count; i++) {
        for (j = 0; j < f->context_count[i] * CONTEXT_SIZE; j++)
            if (f->initial_states[i] && f->initial_states[i][0][j] != 128)
                break;
        if (j < f->context_count[i] * CONTEXT_SIZE) {
            put_rac(c, state, 1);
            for (j = 0; j < f->context_count[i]; j++)
                for (k = 0; k < CONTEXT_SIZE; k++) {
                    int pred = j ? f->initial_states[i][j - 1][k] : 128;
                    put_symbol(c, state2[k],
                               (int8_t)(f->initial_states[i][j][k] - pred), 1);
                }
        } else {
            put_rac(c, state, 0);
        }
    }

    if (f->version > 2) {
        put_symbol(c, state, f->ec, 0);
        put_symbol(c, state, f->intra = (f->avctx->gop_size < 2), 0);
    }

    f->avctx->extradata_size = ff_rac_terminate(c);
    v = av_crc(av_crc_get_table(AV_CRC_32_IEEE), 0, f->avctx->extradata, f->avctx->extradata_size);
    AV_WL32(f->avctx->extradata + f->avctx->extradata_size, v);
    f->avctx->extradata_size += 4;

    return 0;
}

static int sort_stt(FFV1Context *s, uint8_t stt[256])
{
    int i, i2, changed, print = 0;

    do {
        changed = 0;
        for (i = 12; i < 244; i++) {
            for (i2 = i + 1; i2 < 245 && i2 < i + 4; i2++) {

#define COST(old, new)                                      \
    s->rc_stat[old][0] * -log2((256 - (new)) / 256.0) +     \
    s->rc_stat[old][1] * -log2((new)         / 256.0)

#define COST2(old, new)                         \
    COST(old, new) + COST(256 - (old), 256 - (new))

                double size0 = COST2(i,  i) + COST2(i2, i2);
                double sizeX = COST2(i, i2) + COST2(i2, i);
                if (size0 - sizeX > size0*(1e-14) && i != 128 && i2 != 128) {
                    int j;
                    FFSWAP(int, stt[i], stt[i2]);
                    FFSWAP(int, s->rc_stat[i][0], s->rc_stat[i2][0]);
                    FFSWAP(int, s->rc_stat[i][1], s->rc_stat[i2][1]);
                    if (i != 256 - i2) {
                        FFSWAP(int, stt[256 - i], stt[256 - i2]);
                        FFSWAP(int, s->rc_stat[256 - i][0], s->rc_stat[256 - i2][0]);
                        FFSWAP(int, s->rc_stat[256 - i][1], s->rc_stat[256 - i2][1]);
                    }
                    for (j = 1; j < 256; j++) {
                        if (stt[j] == i)
                            stt[j] = i2;
                        else if (stt[j] == i2)
                            stt[j] = i;
                        if (i != 256 - i2) {
                            if (stt[256 - j] == 256 - i)
                                stt[256 - j] = 256 - i2;
                            else if (stt[256 - j] == 256 - i2)
                                stt[256 - j] = 256 - i;
                        }
                    }
                    print = changed = 1;
                }
            }
        }
    } while (changed);
    return print;
}

static av_cold int encode_init(AVCodecContext *avctx)
{
    FFV1Context *s = avctx->priv_data;
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(avctx->pix_fmt);
    int i, j, k, m, ret, plane_index;

    #if FF_API_MOTION_EST
    FF_DISABLE_DEPRECATION_WARNINGS
        if (avctx->me_method == ME_ITER)
            s->motion_est = FF_ME_ITER;
    FF_ENABLE_DEPRECATION_WARNINGS
    #endif

    s->mv_scale       = (avctx->flags & AV_CODEC_FLAG_QPEL) ? 2 : 4;
    s->block_max_depth= (avctx->flags & AV_CODEC_FLAG_4MV ) ? 1 : 0;
    
    //s->p_frame = 0;

    for(plane_index=0; plane_index<3; plane_index++){
        s->plane[plane_index].diag_mc= 1;
        s->plane[plane_index].htaps= 6;
        s->plane[plane_index].hcoeff[0]=  40;
        s->plane[plane_index].hcoeff[1]= -10;
        s->plane[plane_index].hcoeff[2]=   2;
        s->plane[plane_index].fast_mc= 1;
    }

    if ((ret = ff_ffv1_common_init(avctx)) < 0)
        return ret;

    ff_mpegvideoencdsp_init(&s->mpvencdsp, avctx);

    ff_ffv1_alloc_blocks(s);

    s->version = 0;

    s->m.avctx   = avctx;
    s->m.bit_rate= avctx->bit_rate;

    s->m.me.temp      =
    s->m.me.scratchpad= av_mallocz_array((avctx->width+64), 2*16*2*sizeof(uint8_t));
    s->m.me.map       = av_mallocz(ME_MAP_SIZE*sizeof(uint32_t));
    s->m.me.score_map = av_mallocz(ME_MAP_SIZE*sizeof(uint32_t));
    s->m.sc.obmc_scratchpad= av_mallocz(MB_SIZE*MB_SIZE*12*sizeof(uint32_t));
    if (!s->m.me.scratchpad || !s->m.me.map || !s->m.me.score_map || !s->m.sc.obmc_scratchpad)
        return AVERROR(ENOMEM);

    ff_h263_encode_init(&s->m); //mv_penalty

    s->max_ref_frames = av_clip(avctx->refs, 1, MAX_REF_FRAMES);

    if ((avctx->flags & (AV_CODEC_FLAG_PASS1 | AV_CODEC_FLAG_PASS2)) ||
        avctx->slices > 1)
        s->version = FFMAX(s->version, 2);

    // Unspecified level & slices, we choose version 1.2+ to ensure multithreaded decodability
    if (avctx->slices == 0 && avctx->level < 0 && avctx->width * avctx->height > 720*576)
        s->version = FFMAX(s->version, 2);

    if (avctx->level <= 0 && s->version == 2) {
        s->version = 3;
    }
    if (avctx->level >= 0 && avctx->level <= 4) {
        if (avctx->level < s->version) {
            av_log(avctx, AV_LOG_ERROR, "Version %d needed for requested features but %d requested\n", s->version, avctx->level);
            return AVERROR(EINVAL);
        }
        s->version = avctx->level;
    }

    if (s->ec < 0) {
        s->ec = (s->version >= 3);
    }

    if ((s->version == 2 || s->version>3) && avctx->strict_std_compliance > FF_COMPLIANCE_EXPERIMENTAL) {
        av_log(avctx, AV_LOG_ERROR, "Version 2 needed for requested features but version 2 is experimental and not enabled\n");
        return AVERROR_INVALIDDATA;
    }

#if FF_API_CODER_TYPE
FF_DISABLE_DEPRECATION_WARNINGS
    if (avctx->coder_type != -1)
        s->ac = avctx->coder_type > 0 ? AC_RANGE_CUSTOM_TAB : AC_GOLOMB_RICE;
    else
FF_ENABLE_DEPRECATION_WARNINGS
#endif
    if (s->ac == 1) // Compatbility with common command line usage
        s->ac = AC_RANGE_CUSTOM_TAB;
    else if (s->ac == AC_RANGE_DEFAULT_TAB_FORCE)
        s->ac = AC_RANGE_DEFAULT_TAB;

    s->plane_count = 3;
    switch(avctx->pix_fmt) {
    case AV_PIX_FMT_YUV444P9:
    case AV_PIX_FMT_YUV422P9:
    case AV_PIX_FMT_YUV420P9:
    case AV_PIX_FMT_YUVA444P9:
    case AV_PIX_FMT_YUVA422P9:
    case AV_PIX_FMT_YUVA420P9:
        if (!avctx->bits_per_raw_sample)
            s->bits_per_raw_sample = 9;
    case AV_PIX_FMT_YUV444P10:
    case AV_PIX_FMT_YUV420P10:
    case AV_PIX_FMT_YUV422P10:
    case AV_PIX_FMT_YUVA444P10:
    case AV_PIX_FMT_YUVA422P10:
    case AV_PIX_FMT_YUVA420P10:
        s->packed_at_lsb = 1;
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample)
            s->bits_per_raw_sample = 10;
    case AV_PIX_FMT_GRAY16:
    case AV_PIX_FMT_YUV444P16:
    case AV_PIX_FMT_YUV422P16:
    case AV_PIX_FMT_YUV420P16:
    case AV_PIX_FMT_YUVA444P16:
    case AV_PIX_FMT_YUVA422P16:
    case AV_PIX_FMT_YUVA420P16:
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample) {
            s->bits_per_raw_sample = 16;
        } else if (!s->bits_per_raw_sample) {
            s->bits_per_raw_sample = avctx->bits_per_raw_sample;
        }
        if (s->bits_per_raw_sample <= 8) {
            av_log(avctx, AV_LOG_ERROR, "bits_per_raw_sample invalid\n");
            return AVERROR_INVALIDDATA;
        }
        if (s->ac == AC_GOLOMB_RICE) {
            av_log(avctx, AV_LOG_INFO,
                   "bits_per_raw_sample > 8, forcing range coder\n");
            s->ac = AC_RANGE_CUSTOM_TAB;
        }
        s->version = FFMAX(s->version, 1);
        s->p_frame = 0;
    case AV_PIX_FMT_GRAY8:
    case AV_PIX_FMT_YA8:
    case AV_PIX_FMT_YUV444P:
    case AV_PIX_FMT_YUV440P:
    case AV_PIX_FMT_YUV422P:
    case AV_PIX_FMT_YUV420P:
    case AV_PIX_FMT_YUV411P:
    case AV_PIX_FMT_YUV410P:
    case AV_PIX_FMT_YUVA444P:
    case AV_PIX_FMT_YUVA422P:
    case AV_PIX_FMT_YUVA420P:
        //s->p_frame = 1;
        s->chroma_planes = desc->nb_components < 3 ? 0 : 1;
        s->colorspace = 0;
        s->transparency = desc->nb_components == 4 || desc->nb_components == 2;
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample)
            s->bits_per_raw_sample = 8;
        else if (!s->bits_per_raw_sample)
            s->bits_per_raw_sample = 8;
        break;
    case AV_PIX_FMT_RGB32:
        s->p_frame = 0;
        s->colorspace = 1;
        s->transparency = 1;
        s->chroma_planes = 1;
        if (!avctx->bits_per_raw_sample)
            s->bits_per_raw_sample = 8;
        break;
    case AV_PIX_FMT_0RGB32:
        s->p_frame = 0;
        s->colorspace = 1;
        s->chroma_planes = 1;
        if (!avctx->bits_per_raw_sample)
            s->bits_per_raw_sample = 8;
        break;
    case AV_PIX_FMT_GBRP9:
        if (!avctx->bits_per_raw_sample)
            s->bits_per_raw_sample = 9;
    case AV_PIX_FMT_GBRP10:
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample)
            s->bits_per_raw_sample = 10;
    case AV_PIX_FMT_GBRP12:
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample)
            s->bits_per_raw_sample = 12;
    case AV_PIX_FMT_GBRP14:
        if (!avctx->bits_per_raw_sample && !s->bits_per_raw_sample)
            s->bits_per_raw_sample = 14;
        else if (!s->bits_per_raw_sample)
            s->bits_per_raw_sample = avctx->bits_per_raw_sample;
        s->colorspace = 1;
        s->chroma_planes = 1;
        s->version = FFMAX(s->version, 1);
        if (s->ac == AC_GOLOMB_RICE) {
            av_log(avctx, AV_LOG_INFO,
                   "bits_per_raw_sample > 8, forcing coder 1\n");
            s->ac = AC_RANGE_CUSTOM_TAB;
        }
        s->p_frame = 0;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "format not supported\n");
        return AVERROR(ENOSYS);
    }
    av_assert0(s->bits_per_raw_sample >= 8);
    avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_h_shift, &s->chroma_v_shift);

    s->nb_planes = 0;
    for (i = 0; i < desc->nb_components; i++)
        s->nb_planes = FFMAX(s->nb_planes, desc->comp[i].plane + 1);

    if (s->transparency) {
        av_log(avctx, AV_LOG_WARNING, "Storing alpha plane, this will require a recent FFV1 decoder to playback!\n");
    }
#if FF_API_PRIVATE_OPT
FF_DISABLE_DEPRECATION_WARNINGS
    if (avctx->context_model)
        s->context_model = avctx->context_model;
    if (avctx->context_model > 1U) {
        av_log(avctx, AV_LOG_ERROR, "Invalid context model %d, valid values are 0 and 1\n", avctx->context_model);
        return AVERROR(EINVAL);
    }
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    if (s->ac == AC_RANGE_CUSTOM_TAB) {
        for (i = 1; i < 256; i++)
            s->state_transition[i] = ver2_state[i];
    } else {
        RangeCoder c;
        ff_build_rac_states(&c, 0.05 * (1LL << 32), 256 - 8);
        for (i = 1; i < 256; i++)
            s->state_transition[i] = c.one_state[i];
    }

    for (i = 0; i < 256; i++) {
        s->quant_table_count = 2;
        if (s->bits_per_raw_sample <= 8) {
            s->quant_tables[0][0][i]=           quant11[i];
            s->quant_tables[0][1][i]=        11*quant11[i];
            s->quant_tables[0][2][i]=     11*11*quant11[i];
            s->quant_tables[1][0][i]=           quant11[i];
            s->quant_tables[1][1][i]=        11*quant11[i];
            s->quant_tables[1][2][i]=     11*11*quant5 [i];
            s->quant_tables[1][3][i]=   5*11*11*quant5 [i];
            s->quant_tables[1][4][i]= 5*5*11*11*quant5 [i];
        } else {
            s->quant_tables[0][0][i]=           quant9_10bit[i];
            s->quant_tables[0][1][i]=        11*quant9_10bit[i];
            s->quant_tables[0][2][i]=     11*11*quant9_10bit[i];
            s->quant_tables[1][0][i]=           quant9_10bit[i];
            s->quant_tables[1][1][i]=        11*quant9_10bit[i];
            s->quant_tables[1][2][i]=     11*11*quant5_10bit[i];
            s->quant_tables[1][3][i]=   5*11*11*quant5_10bit[i];
            s->quant_tables[1][4][i]= 5*5*11*11*quant5_10bit[i];
        }
    }
    s->context_count[0] = (11 * 11 * 11        + 1) / 2;
    s->context_count[1] = (11 * 11 * 5 * 5 * 5 + 1) / 2;
    memcpy(s->quant_table, s->quant_tables[s->context_model],
           sizeof(s->quant_table));

    for (i = 0; i < s->plane_count; i++) {
        PlaneContext *const p = &s->plane[i];

        memcpy(p->quant_table, s->quant_table, sizeof(p->quant_table));
        p->quant_table_index = s->context_model;
        p->context_count     = s->context_count[p->quant_table_index];
    }

    if ((ret = ff_ffv1_allocate_initial_states(s)) < 0)
        return ret;

#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
    avctx->coded_frame->pict_type = AV_PICTURE_TYPE_I;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    if (!s->transparency)
        s->plane_count = 2;
    if (!s->chroma_planes && s->version > 3)
        s->plane_count--;

    avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_h_shift, &s->chroma_v_shift);
    s->picture_number = 0;

    if (avctx->flags & (AV_CODEC_FLAG_PASS1 | AV_CODEC_FLAG_PASS2)) {
        for (i = 0; i < s->quant_table_count; i++) {
            s->rc_stat2[i] = av_mallocz(s->context_count[i] *
                                        sizeof(*s->rc_stat2[i]));
            if (!s->rc_stat2[i])
                return AVERROR(ENOMEM);
        }
    }
    if (avctx->stats_in) {
        char *p = avctx->stats_in;
        uint8_t (*best_state)[256] = av_malloc_array(256, 256);
        int gob_count = 0;
        char *next;
        if (!best_state)
            return AVERROR(ENOMEM);

        av_assert0(s->version >= 2);

        for (;;) {
            for (j = 0; j < 256; j++)
                for (i = 0; i < 2; i++) {
                    s->rc_stat[j][i] = strtol(p, &next, 0);
                    if (next == p) {
                        av_log(avctx, AV_LOG_ERROR,
                               "2Pass file invalid at %d %d [%s]\n", j, i, p);
                        av_freep(&best_state);
                        return AVERROR_INVALIDDATA;
                    }
                    p = next;
                }
            for (i = 0; i < s->quant_table_count; i++)
                for (j = 0; j < s->context_count[i]; j++) {
                    for (k = 0; k < 32; k++)
                        for (m = 0; m < 2; m++) {
                            s->rc_stat2[i][j][k][m] = strtol(p, &next, 0);
                            if (next == p) {
                                av_log(avctx, AV_LOG_ERROR,
                                       "2Pass file invalid at %d %d %d %d [%s]\n",
                                       i, j, k, m, p);
                                av_freep(&best_state);
                                return AVERROR_INVALIDDATA;
                            }
                            p = next;
                        }
                }
            gob_count = strtol(p, &next, 0);
            if (next == p || gob_count <= 0) {
                av_log(avctx, AV_LOG_ERROR, "2Pass file invalid\n");
                av_freep(&best_state);
                return AVERROR_INVALIDDATA;
            }
            p = next;
            while (*p == '\n' || *p == ' ')
                p++;
            if (p[0] == 0)
                break;
        }
        if (s->ac == AC_RANGE_CUSTOM_TAB)
            sort_stt(s, s->state_transition);

        find_best_state(best_state, s->state_transition);

        for (i = 0; i < s->quant_table_count; i++) {
            for (k = 0; k < 32; k++) {
                double a=0, b=0;
                int jp = 0;
                for (j = 0; j < s->context_count[i]; j++) {
                    double p = 128;
                    if (s->rc_stat2[i][j][k][0] + s->rc_stat2[i][j][k][1] > 200 && j || a+b > 200) {
                        if (a+b)
                            p = 256.0 * b / (a + b);
                        s->initial_states[i][jp][k] =
                            best_state[av_clip(round(p), 1, 255)][av_clip_uint8((a + b) / gob_count)];
                        for(jp++; jp<j; jp++)
                            s->initial_states[i][jp][k] = s->initial_states[i][jp-1][k];
                        a=b=0;
                    }
                    a += s->rc_stat2[i][j][k][0];
                    b += s->rc_stat2[i][j][k][1];
                    if (a+b) {
                        p = 256.0 * b / (a + b);
                    }
                    s->initial_states[i][j][k] =
                        best_state[av_clip(round(p), 1, 255)][av_clip_uint8((a + b) / gob_count)];
                }
            }
        }
        av_freep(&best_state);
    }

    if (s->version > 1) {
        s->num_v_slices = (avctx->width > 352 || avctx->height > 288 || !avctx->slices) ? 2 : 1;
        for (; s->num_v_slices < 9; s->num_v_slices++) {
            for (s->num_h_slices = s->num_v_slices; s->num_h_slices < 2*s->num_v_slices; s->num_h_slices++) {
                if (avctx->slices == s->num_h_slices * s->num_v_slices && avctx->slices <= 64 || !avctx->slices)
                    goto slices_ok;
            }
        }
        av_log(avctx, AV_LOG_ERROR,
               "Unsupported number %d of slices requested, please specify a "
               "supported number with -slices (ex:4,6,9,12,16, ...)\n",
               avctx->slices);
        return AVERROR(ENOSYS);
slices_ok:
        if ((ret = write_extradata(s)) < 0)
            return ret;
    }

    if ((ret = ff_ffv1_init_slice_contexts(s)) < 0)
        return ret;
    s->slice_count = s->max_slice_count;
    if ((ret = ff_ffv1_init_slices_state(s)) < 0)
        return ret;

#define STATS_OUT_SIZE 1024 * 1024 * 6
    if (avctx->flags & AV_CODEC_FLAG_PASS1) {
        avctx->stats_out = av_mallocz(STATS_OUT_SIZE);
        if (!avctx->stats_out)
            return AVERROR(ENOMEM);
        for (i = 0; i < s->quant_table_count; i++)
            for (j = 0; j < s->max_slice_count; j++) {
                FFV1Context *sf = s->slice_context[j];
                av_assert0(!sf->rc_stat2[i]);
                sf->rc_stat2[i] = av_mallocz(s->context_count[i] *
                                             sizeof(*sf->rc_stat2[i]));
                if (!sf->rc_stat2[i])
                    return AVERROR(ENOMEM);
            }
    }

    ff_set_cmp(&s->mecc, s->mecc.me_cmp, s->avctx->me_cmp);
    ff_set_cmp(&s->mecc, s->mecc.me_sub_cmp, s->avctx->me_sub_cmp);

    s->input_picture = av_frame_alloc();
    if (!s->input_picture)
        return AVERROR(ENOMEM);

    if ((ret = ff_ffv1_get_buffer(s, s->input_picture)) < 0)
        return ret;

    if(s->motion_est == FF_ME_ITER){
        int size= s->b_width * s->b_height << 2*s->block_max_depth;
        for(i=0; i<s->max_ref_frames; i++){
            s->ref_mvs[i]= av_mallocz_array(size, sizeof(int16_t[2]));
            s->ref_scores[i]= av_mallocz_array(size, sizeof(uint32_t));
            if (!s->ref_mvs[i] || !s->ref_scores[i])
                return AVERROR(ENOMEM);
        }
    }

    return 0;
}

static void encode_slice_header(FFV1Context *f, FFV1Context *fs)
{
    RangeCoder *c = &fs->c;
    uint8_t state[CONTEXT_SIZE];
    int j;
    memset(state, 128, sizeof(state));

    put_symbol(c, state, (fs->slice_x     +1)*f->num_h_slices / f->width   , 0);
    put_symbol(c, state, (fs->slice_y     +1)*f->num_v_slices / f->height  , 0);
    put_symbol(c, state, (fs->slice_width +1)*f->num_h_slices / f->width -1, 0);
    put_symbol(c, state, (fs->slice_height+1)*f->num_v_slices / f->height-1, 0);
    for (j=0; j<f->plane_count; j++) {
        put_symbol(c, state, f->plane[j].quant_table_index, 0);
        av_assert0(f->plane[j].quant_table_index == f->context_model);
    }
    if (!f->picture.f->interlaced_frame)
        put_symbol(c, state, 3, 0);
    else
        put_symbol(c, state, 1 + !f->picture.f->top_field_first, 0);
    put_symbol(c, state, f->picture.f->sample_aspect_ratio.num, 0);
    put_symbol(c, state, f->picture.f->sample_aspect_ratio.den, 0);
    if (f->version > 3) {
        put_rac(c, state, fs->slice_coding_mode == 1);
        if (fs->slice_coding_mode == 1)
            ff_ffv1_clear_slice_state(f, fs);
        put_symbol(c, state, fs->slice_coding_mode, 0);
        if (fs->slice_coding_mode != 1) {
            put_symbol(c, state, fs->slice_rct_by_coef, 0);
            put_symbol(c, state, fs->slice_rct_ry_coef, 0);
        }
    }
}

static void choose_rct_params(FFV1Context *fs, const uint8_t *src[3], const int stride[3], int w, int h)
{
#define NB_Y_COEFF 15
    static const int rct_y_coeff[15][2] = {
        {0, 0}, //      4G
        {1, 1}, //  R + 2G + B
        {2, 2}, // 2R      + 2B
        {0, 2}, //      2G + 2B
        {2, 0}, // 2R + 2G
        {4, 0}, // 4R
        {0, 4}, //           4B

        {0, 3}, //      1G + 3B
        {3, 0}, // 3R + 1G
        {3, 1}, // 3R      +  B
        {1, 3}, //  R      + 3B
        {1, 2}, //  R +  G + 2B
        {2, 1}, // 2R +  G +  B
        {0, 1}, //      3G +  B
        {1, 0}, //  R + 3G
    };

    int stat[NB_Y_COEFF] = {0};
    int x, y, i, p, best;
    int16_t *sample[3];
    int lbd = fs->bits_per_raw_sample <= 8;

    for (y = 0; y < h; y++) {
        int lastr=0, lastg=0, lastb=0;
        for (p = 0; p < 3; p++)
            sample[p] = fs->sample_buffer + p*w;

        for (x = 0; x < w; x++) {
            int b, g, r;
            int ab, ag, ar;
            if (lbd) {
                unsigned v = *((const uint32_t*)(src[0] + x*4 + stride[0]*y));
                b =  v        & 0xFF;
                g = (v >>  8) & 0xFF;
                r = (v >> 16) & 0xFF;
            } else {
                b = *((const uint16_t*)(src[0] + x*2 + stride[0]*y));
                g = *((const uint16_t*)(src[1] + x*2 + stride[1]*y));
                r = *((const uint16_t*)(src[2] + x*2 + stride[2]*y));
            }

            ar = r - lastr;
            ag = g - lastg;
            ab = b - lastb;
            if (x && y) {
                int bg = ag - sample[0][x];
                int bb = ab - sample[1][x];
                int br = ar - sample[2][x];

                br -= bg;
                bb -= bg;

                for (i = 0; i<NB_Y_COEFF; i++) {
                    stat[i] += FFABS(bg + ((br*rct_y_coeff[i][0] + bb*rct_y_coeff[i][1])>>2));
                }

            }
            sample[0][x] = ag;
            sample[1][x] = ab;
            sample[2][x] = ar;

            lastr = r;
            lastg = g;
            lastb = b;
        }
    }

    best = 0;
    for (i=1; i<NB_Y_COEFF; i++) {
        if (stat[i] < stat[best])
            best = i;
    }

    fs->slice_rct_by_coef = rct_y_coeff[best][1];
    fs->slice_rct_ry_coef = rct_y_coeff[best][0];
}

//near copy & paste from dsputil, FIXME
static int pix_sum(uint8_t * pix, int line_size, int w, int h)
{
    int s, i, j;

    s = 0;
    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            s += pix[0];
            pix ++;
        }
        pix += line_size - w;
    }
    return s;
}

//near copy & paste from dsputil, FIXME
static int pix_norm1(uint8_t * pix, int line_size, int w)
{
    int s, i, j;
    uint32_t *sq = ff_square_tab + 256;

    s = 0;
    for (i = 0; i < w; i++) {
        for (j = 0; j < w; j ++) {
            s += sq[pix[0]];
            pix ++;
        }
        pix += line_size - w;
    }
    return s;
}

static inline int get_penalty_factor(int lambda, int lambda2, int type){
    switch(type&0xFF){
    default:
    case FF_CMP_SAD:
        return lambda>>FF_LAMBDA_SHIFT;
    case FF_CMP_DCT:
        return (3*lambda)>>(FF_LAMBDA_SHIFT+1);
    case FF_CMP_W53:
        return (4*lambda)>>(FF_LAMBDA_SHIFT);
    case FF_CMP_W97:
        return (2*lambda)>>(FF_LAMBDA_SHIFT);
    case FF_CMP_SATD:
    case FF_CMP_DCT264:
        return (2*lambda)>>FF_LAMBDA_SHIFT;
    case FF_CMP_RD:
    case FF_CMP_PSNR:
    case FF_CMP_SSE:
    case FF_CMP_NSSE:
        return lambda2>>FF_LAMBDA_SHIFT;
    case FF_CMP_BIT:
    case FF_CMP_MEDIAN_SAD:
        return 1;
    }
}

#define P_LEFT P[1]
#define P_TOP P[2]
#define P_TOPRIGHT P[3]
#define P_MEDIAN P[4]
#define P_MV1 P[9]
#define FLAG_QPEL   1 //must be 1

static int encode_q_branch(FFV1Context *s, int level, int x, int y)
{
    RangeCoder *const rc_s = &s->slice_context[0]->c;
    uint8_t p_buffer[1024];
    uint8_t i_buffer[1024];
    uint8_t p_state[sizeof(s->block_state)];
    uint8_t i_state[sizeof(s->block_state)];
    RangeCoder pc, ic;
    uint8_t *pbbak= rc_s->bytestream;
    uint8_t *pbbak_start= rc_s->bytestream_start;
    int score, score2, iscore, i_len, p_len, block_s, sum, base_bits;
    const int w= s->b_width  << s->block_max_depth;
    const int h= s->b_height << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    const int block_w= 1<<(LOG2_MB_SIZE - level);
    int trx= (x+1)<<rem_depth;
    int try= (y+1)<<rem_depth;
    const BlockNode *left  = x ? &s->block[index-1] : &null_block;
    const BlockNode *top   = y ? &s->block[index-w] : &null_block;
    const BlockNode *right = trx<w ? &s->block[index+1] : &null_block;
    const BlockNode *bottom= try<h ? &s->block[index+w] : &null_block;
    const BlockNode *tl    = y && x ? &s->block[index-w-1] : left;
    const BlockNode *tr    = y && trx<w && ((x&1)==0 || level==0) ? &s->block[index-w+(1<<rem_depth)] : tl; //FIXME use lt
    int pl = left->color[0];
    int pcb= left->color[1];
    int pcr= left->color[2];
    int pmx, pmy;
    int mx=0, my=0;
    int l,cr,cb;
    const int stride= s->current_picture->linesize[0];
    const int uvstride= s->current_picture->linesize[1];
    uint8_t *current_data[3]= { s->input_picture->data[0] + (x + y*  stride)*block_w,
                                s->input_picture->data[1] + ((x*block_w)>>s->chroma_h_shift) + ((y*uvstride*block_w)>>s->chroma_v_shift),
                                s->input_picture->data[2] + ((x*block_w)>>s->chroma_h_shift) + ((y*uvstride*block_w)>>s->chroma_v_shift)};
    int P[10][2];
    int16_t last_mv[3][2];
    int qpel= !!(s->avctx->flags & AV_CODEC_FLAG_QPEL); //unused
    const int shift= 1+qpel;
    MotionEstContext *c= &s->m.me;
    int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
    int mx_context= av_log2(2*FFABS(left->mx - top->mx));
    int my_context= av_log2(2*FFABS(left->my - top->my));
    int s_context= 2*left->level + 2*top->level + tl->level + tr->level;
    int ref, best_ref, ref_score, ref_mx, ref_my;

    av_assert0(sizeof(s->block_state) >= 256);
    if(s->key_frame){
        set_blocks(s, level, x, y, pl, pcb, pcr, 0, 0, 0, BLOCK_INTRA);
        return 0;
    }

//    clip predictors / edge ?

    P_LEFT[0]= left->mx;
    P_LEFT[1]= left->my;
    P_TOP [0]= top->mx;
    P_TOP [1]= top->my;
    P_TOPRIGHT[0]= tr->mx;
    P_TOPRIGHT[1]= tr->my;

    last_mv[0][0]= s->block[index].mx;
    last_mv[0][1]= s->block[index].my;
    last_mv[1][0]= right->mx;
    last_mv[1][1]= right->my;
    last_mv[2][0]= bottom->mx;
    last_mv[2][1]= bottom->my;

    s->m.mb_stride=2;
    s->m.mb_x=
    s->m.mb_y= 0;
    c->skip= 0;

    av_assert1(c->  stride ==   stride);
    av_assert1(c->uvstride == uvstride);

    c->penalty_factor    = get_penalty_factor(s->lambda, s->lambda2, c->avctx->me_cmp);
    c->sub_penalty_factor= get_penalty_factor(s->lambda, s->lambda2, c->avctx->me_sub_cmp);
    c->mb_penalty_factor = get_penalty_factor(s->lambda, s->lambda2, c->avctx->mb_cmp);
    c->current_mv_penalty= c->mv_penalty[s->m.f_code=1] + MAX_DMV;

    c->xmin = - x*block_w - 16+3;
    c->ymin = - y*block_w - 16+3;
    c->xmax = - (x+1)*block_w + (w<<(LOG2_MB_SIZE - s->block_max_depth)) + 16-3;
    c->ymax = - (y+1)*block_w + (h<<(LOG2_MB_SIZE - s->block_max_depth)) + 16-3;

    if(P_LEFT[0]     > (c->xmax<<shift)) P_LEFT[0]    = (c->xmax<<shift);
    if(P_LEFT[1]     > (c->ymax<<shift)) P_LEFT[1]    = (c->ymax<<shift);
    if(P_TOP[0]      > (c->xmax<<shift)) P_TOP[0]     = (c->xmax<<shift);
    if(P_TOP[1]      > (c->ymax<<shift)) P_TOP[1]     = (c->ymax<<shift);
    if(P_TOPRIGHT[0] < (c->xmin<<shift)) P_TOPRIGHT[0]= (c->xmin<<shift);
    if(P_TOPRIGHT[0] > (c->xmax<<shift)) P_TOPRIGHT[0]= (c->xmax<<shift); //due to pmx no clip
    if(P_TOPRIGHT[1] > (c->ymax<<shift)) P_TOPRIGHT[1]= (c->ymax<<shift);

    P_MEDIAN[0]= mid_pred(P_LEFT[0], P_TOP[0], P_TOPRIGHT[0]);
    P_MEDIAN[1]= mid_pred(P_LEFT[1], P_TOP[1], P_TOPRIGHT[1]);

    if (!y) {
        c->pred_x= P_LEFT[0];
        c->pred_y= P_LEFT[1];
    } else {
        c->pred_x = P_MEDIAN[0];
        c->pred_y = P_MEDIAN[1];
    }

    score= INT_MAX;
    best_ref= 0;
    for(ref=0; ref<s->ref_frames; ref++){
        init_ref(c, current_data, s->last_pictures[ref]->data, NULL, block_w*x, block_w*y, 0);

        ref_score= ff_epzs_motion_search(&s->m, &ref_mx, &ref_my, P, 0, /*ref_index*/ 0, last_mv,
                                         (1<<16)>>shift, level-LOG2_MB_SIZE+4, block_w);

        av_assert2(ref_mx >= c->xmin);
        av_assert2(ref_mx <= c->xmax);
        av_assert2(ref_my >= c->ymin);
        av_assert2(ref_my <= c->ymax);

        ref_score= c->sub_motion_search(&s->m, &ref_mx, &ref_my, ref_score, 0, 0, level-LOG2_MB_SIZE+4, block_w);
        ref_score= ff_get_mb_score(&s->m, ref_mx, ref_my, 0, 0, level-LOG2_MB_SIZE+4, block_w, 0);
        ref_score+= 2*av_log2(2*ref)*c->penalty_factor;
        if(s->ref_mvs[ref]){
            s->ref_mvs[ref][index][0]= ref_mx;
            s->ref_mvs[ref][index][1]= ref_my;
            s->ref_scores[ref][index]= ref_score;
        }
        if(score > ref_score){
            score= ref_score;
            best_ref= ref;
            mx= ref_mx;
            my= ref_my;
        }
    }
    //FIXME if mb_cmp != SSE then intra cannot be compared currently and mb_penalty vs. lambda2

    // subpel search
    base_bits= get_rac_count(rc_s) - 8*(rc_s->bytestream - rc_s->bytestream_start);
    pc= *rc_s;
    pc.bytestream_start=
    pc.bytestream= p_buffer; //FIXME end/start? and at the other stoo
    memcpy(p_state, s->block_state, sizeof(s->block_state));

    if(level!=s->block_max_depth)
        put_rac(&pc, &p_state[4 + s_context], 1);
    put_rac(&pc, &p_state[1 + left->type + top->type], 0);
    if(s->ref_frames > 1)
        put_symbol(&pc, &p_state[128 + 1024 + 32*ref_context], best_ref, 0);
    pred_mv(s, &pmx, &pmy, best_ref, left, top, tr);
    put_symbol(&pc, &p_state[128 + 32*(mx_context + 16*!!best_ref)], mx - pmx, 1);
    put_symbol(&pc, &p_state[128 + 32*(my_context + 16*!!best_ref)], my - pmy, 1);
    p_len= pc.bytestream - pc.bytestream_start;
    score += (s->lambda2*(get_rac_count(&pc)-base_bits))>>FF_LAMBDA_SHIFT;

    block_s= block_w*block_w;
    sum = pix_sum(current_data[0], stride, block_w, block_w);
    l= (sum + block_s/2)/block_s;
    iscore = pix_norm1(current_data[0], stride, block_w) - 2*l*sum + l*l*block_s;

    if (s->nb_planes > 2) {
        block_s= block_w*block_w>>(s->chroma_h_shift + s->chroma_v_shift);
        sum = pix_sum(current_data[1], uvstride, block_w>>s->chroma_h_shift, block_w>>s->chroma_v_shift);
        cb= (sum + block_s/2)/block_s;
    //    iscore += pix_norm1(&current_mb[1][0], uvstride, block_w>>1) - 2*cb*sum + cb*cb*block_s;
        sum = pix_sum(current_data[2], uvstride, block_w>>s->chroma_h_shift, block_w>>s->chroma_v_shift);
        cr= (sum + block_s/2)/block_s;
    //    iscore += pix_norm1(&current_mb[2][0], uvstride, block_w>>1) - 2*cr*sum + cr*cr*block_s;
    }else
        cb = cr = 0;

    ic= *rc_s;
    ic.bytestream_start=
    ic.bytestream= i_buffer; //FIXME end/start? and at the other stoo
    memcpy(i_state, s->block_state, sizeof(s->block_state));
    if(level!=s->block_max_depth)
        put_rac(&ic, &i_state[4 + s_context], 1);
    put_rac(&ic, &i_state[1 + left->type + top->type], 1);
    put_symbol(&ic, &i_state[32],  l-pl , 1);
    if (s->nb_planes > 2) {
        put_symbol(&ic, &i_state[64], cb-pcb, 1);
        put_symbol(&ic, &i_state[96], cr-pcr, 1);
    }
    i_len= ic.bytestream - ic.bytestream_start;
    iscore += (s->lambda2*(get_rac_count(&ic)-base_bits))>>FF_LAMBDA_SHIFT;

    av_assert1(iscore < 255*255*256 + s->lambda2*10);
    av_assert1(iscore >= 0);
    av_assert1(l>=0 && l<=255);
    av_assert1(pl>=0 && pl<=255);

    if(level==0){
        int varc= iscore >> 8;
        int vard= score >> 8;
        if (vard <= 64 || vard < varc)
            c->scene_change_score+= ff_sqrt(vard) - ff_sqrt(varc);
        else
            c->scene_change_score+= s->m.qscale;
    }

    if(level!=s->block_max_depth){
        put_rac(rc_s, &s->block_state[4 + s_context], 0);
        score2 = encode_q_branch(s, level+1, 2*x+0, 2*y+0);
        score2+= encode_q_branch(s, level+1, 2*x+1, 2*y+0);
        score2+= encode_q_branch(s, level+1, 2*x+0, 2*y+1);
        score2+= encode_q_branch(s, level+1, 2*x+1, 2*y+1);
        score2+= s->lambda2>>FF_LAMBDA_SHIFT; //FIXME exact split overhead

        if(score2 < score && score2 < iscore)
            return score2;
    }

    if(iscore < score){
        pred_mv(s, &pmx, &pmy, 0, left, top, tr);
        memcpy(pbbak, i_buffer, i_len);
        *rc_s= ic;
        rc_s->bytestream_start= pbbak_start;
        rc_s->bytestream= pbbak + i_len;
        set_blocks(s, level, x, y, l, cb, cr, pmx, pmy, 0, BLOCK_INTRA);
        memcpy(s->block_state, i_state, sizeof(s->block_state));
        return iscore;
    }else{
        memcpy(pbbak, p_buffer, p_len);
        *rc_s= pc;
        rc_s->bytestream_start= pbbak_start;
        rc_s->bytestream= pbbak + p_len;
        set_blocks(s, level, x, y, pl, pcb, pcr, mx, my, best_ref, 0);
        memcpy(s->block_state, p_state, sizeof(s->block_state));
        return score;
    }
}

static void encode_q_branch2(FFV1Context *s, int level, int x, int y)
{
    RangeCoder *const rc_s = &s->slice_context[0]->c;
    const int w= s->b_width  << s->block_max_depth;
    const int rem_depth= s->block_max_depth - level;
    const int index= (x + y*w) << rem_depth;
    int trx= (x+1)<<rem_depth;
    BlockNode *b= &s->block[index];
    const BlockNode *left  = x ? &s->block[index-1] : &null_block;
    const BlockNode *top   = y ? &s->block[index-w] : &null_block;
    const BlockNode *tl    = y && x ? &s->block[index-w-1] : left;
    const BlockNode *tr    = y && trx<w && ((x&1)==0 || level==0) ? &s->block[index-w+(1<<rem_depth)] : tl; //FIXME use lt
    int pl = left->color[0];
    int pcb= left->color[1];
    int pcr= left->color[2];
    int pmx, pmy;
    int ref_context= av_log2(2*left->ref) + av_log2(2*top->ref);
    int mx_context= av_log2(2*FFABS(left->mx - top->mx)) + 16*!!b->ref;
    int my_context= av_log2(2*FFABS(left->my - top->my)) + 16*!!b->ref;
    int s_context= 2*left->level + 2*top->level + tl->level + tr->level;

    if(s->key_frame){
        set_blocks(s, level, x, y, pl, pcb, pcr, 0, 0, 0, BLOCK_INTRA);
        return;
    }

    if(level!=s->block_max_depth){
        if(same_block(b,b+1) && same_block(b,b+w) && same_block(b,b+w+1)){
            put_rac(rc_s, &s->block_state[4 + s_context], 1);
        }else{
            put_rac(rc_s, &s->block_state[4 + s_context], 0);
            encode_q_branch2(s, level+1, 2*x+0, 2*y+0);
            encode_q_branch2(s, level+1, 2*x+1, 2*y+0);
            encode_q_branch2(s, level+1, 2*x+0, 2*y+1);
            encode_q_branch2(s, level+1, 2*x+1, 2*y+1);
            return;
        }
    }
    if(b->type & BLOCK_INTRA){
        pred_mv(s, &pmx, &pmy, 0, left, top, tr);
        put_rac(rc_s, &s->block_state[1 + (left->type&1) + (top->type&1)], 1);
        put_symbol(rc_s, &s->block_state[32], b->color[0]-pl , 1);
        if (s->nb_planes > 2) {
            put_symbol(rc_s, &s->block_state[64], b->color[1]-pcb, 1);
            put_symbol(rc_s, &s->block_state[96], b->color[2]-pcr, 1);
        }
        set_blocks(s, level, x, y, b->color[0], b->color[1], b->color[2], pmx, pmy, 0, BLOCK_INTRA);
    }else{
        pred_mv(s, &pmx, &pmy, b->ref, left, top, tr);
        put_rac(rc_s, &s->block_state[1 + (left->type&1) + (top->type&1)], 0);
        if(s->ref_frames > 1)
            put_symbol(rc_s, &s->block_state[128 + 1024 + 32*ref_context], b->ref, 0);
        put_symbol(rc_s, &s->block_state[128 + 32*mx_context], b->mx - pmx, 1);
        put_symbol(rc_s, &s->block_state[128 + 32*my_context], b->my - pmy, 1);
        set_blocks(s, level, x, y, pl, pcb, pcr, b->mx, b->my, b->ref, 0);
    }
}

static int get_dc(FFV1Context *s, int mb_x, int mb_y, int plane_index){
    int i, x2, y2;
    PlaneContext *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? ff_ffv1_obmc_tab[s->block_max_depth+s->chroma_h_shift] : ff_ffv1_obmc_tab[s->block_max_depth];
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_picture->linesize[plane_index];
    uint8_t *src= s-> input_picture->data[plane_index];
    IDWTELEM *dst= (IDWTELEM*)s->m.sc.obmc_scratchpad + plane_index*block_size*block_size*4; //FIXME change to unsigned
    const int b_stride = s->b_width << s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int index= mb_x + mb_y*b_stride;
    BlockNode *b= &s->block[index];
    BlockNode backup= *b;
    int ab=0;
    int aa=0;

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc stuff above

    b->type|= BLOCK_INTRA;
    b->color[plane_index]= 0;
    memset(dst, 0, obmc_stride*obmc_stride*sizeof(IDWTELEM));

    for(i=0; i<4; i++){
        int mb_x2= mb_x + (i &1) - 1;
        int mb_y2= mb_y + (i>>1) - 1;
        int x= block_w*mb_x2 + block_w/2;
        int y= block_h*mb_y2 + block_h/2;

        add_yblock(s, 0, NULL, dst + (i&1)*block_w + (i>>1)*obmc_stride*block_h, NULL, obmc,
                    x, y, block_w, block_h, w, h, obmc_stride, ref_stride, obmc_stride, mb_x2, mb_y2, 0, 0, plane_index);

        for(y2= FFMAX(y, 0); y2<FFMIN(h, y+block_h); y2++){
            for(x2= FFMAX(x, 0); x2<FFMIN(w, x+block_w); x2++){
                int index= x2-(block_w*mb_x - block_w/2) + (y2-(block_h*mb_y - block_h/2))*obmc_stride;
                int obmc_v= obmc[index];
                int d;
                if(y<0) obmc_v += obmc[index + block_h*obmc_stride];
                if(x<0) obmc_v += obmc[index + block_w];
                if(y+block_h>h) obmc_v += obmc[index - block_h*obmc_stride];
                if(x+block_w>w) obmc_v += obmc[index - block_w];
                //FIXME precalculate this or simplify it somehow else

                d = -dst[index] + (1<<(FRAC_BITS-1));
                dst[index] = d;
                ab += (src[x2 + y2*ref_stride] - (d>>FRAC_BITS)) * obmc_v;
                aa += obmc_v * obmc_v; //FIXME precalculate this
            }
        }
    }
    *b= backup;

    return av_clip_uint8( ROUNDED_DIV(ab<<LOG2_OBMC_MAX, aa) ); //FIXME we should not need clipping
}

static inline int get_block_bits(FFV1Context *s, int x, int y, int w){
    const int b_stride = s->b_width << s->block_max_depth;
    const int b_height = s->b_height<< s->block_max_depth;
    int index= x + y*b_stride;
    const BlockNode *b     = &s->block[index];
    const BlockNode *left  = x ? &s->block[index-1] : &null_block;
    const BlockNode *top   = y ? &s->block[index-b_stride] : &null_block;
    const BlockNode *tl    = y && x ? &s->block[index-b_stride-1] : left;
    const BlockNode *tr    = y && x+w<b_stride ? &s->block[index-b_stride+w] : tl;
    int dmx, dmy;
//  int mx_context= av_log2(2*FFABS(left->mx - top->mx));
//  int my_context= av_log2(2*FFABS(left->my - top->my));

    if(x<0 || x>=b_stride || y>=b_height)
        return 0;
/*
1            0      0
01X          1-2    1
001XX        3-6    2-3
0001XXX      7-14   4-7
00001XXXX   15-30   8-15
*/
//FIXME try accurate rate
//FIXME intra and inter predictors if surrounding blocks are not the same type
    if(b->type & BLOCK_INTRA){
        return 3+2*( av_log2(2*FFABS(left->color[0] - b->color[0]))
                   + av_log2(2*FFABS(left->color[1] - b->color[1]))
                   + av_log2(2*FFABS(left->color[2] - b->color[2])));
    }else{
        pred_mv(s, &dmx, &dmy, b->ref, left, top, tr);
        dmx-= b->mx;
        dmy-= b->my;
        return 2*(1 + av_log2(2*FFABS(dmx)) //FIXME kill the 2* can be merged in lambda
                    + av_log2(2*FFABS(dmy))
                    + av_log2(2*b->ref));
    }
}

static int get_block_rd(FFV1Context *s, int mb_x, int mb_y, int plane_index, uint8_t (*obmc_edged)[MB_SIZE * 2]){
    PlaneContext *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_picture->linesize[plane_index];
    uint8_t *dst= s->current_picture->data[plane_index];
    uint8_t *src= s->  input_picture->data[plane_index];
    IDWTELEM *pred= (IDWTELEM*)s->m.sc.obmc_scratchpad + plane_index*block_size*block_size*4;
    uint8_t *cur = s->scratchbuf;
    uint8_t *tmp = s->emu_edge_buffer;
    const int b_stride = s->b_width << s->block_max_depth;
    const int b_height = s->b_height<< s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int distortion;
    int rate= 0;
    const int penalty_factor= get_penalty_factor(s->lambda, s->lambda2, s->avctx->me_cmp);
    int sx= block_w*mb_x - block_w/2;
    int sy= block_h*mb_y - block_h/2;
    int x0= FFMAX(0,-sx);
    int y0= FFMAX(0,-sy);
    int x1= FFMIN(block_w*2, w-sx);
    int y1= FFMIN(block_h*2, h-sy);
    int i,x,y;

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc and square assumtions below chckinhg only block_w

    ff_ffv1_pred_block(s, cur, tmp, ref_stride, sx, sy, block_w*2, block_h*2, &s->block[mb_x + mb_y*b_stride], plane_index, w, h);

    for(y=y0; y<y1; y++){
        const uint8_t *obmc1= obmc_edged[y];
        const IDWTELEM *pred1 = pred + y*obmc_stride;
        uint8_t *cur1 = cur + y*ref_stride;
        uint8_t *dst1 = dst + sx + (sy+y)*ref_stride;
        for(x=x0; x<x1; x++){
#if FRAC_BITS >= LOG2_OBMC_MAX
            int v = (cur1[x] * obmc1[x]) << (FRAC_BITS - LOG2_OBMC_MAX);
#else
            int v = (cur1[x] * obmc1[x] + (1<<(LOG2_OBMC_MAX - FRAC_BITS-1))) >> (LOG2_OBMC_MAX - FRAC_BITS);
#endif
            v = (v + pred1[x]) >> FRAC_BITS;
            if(v&(~255)) v= ~(v>>31);
            dst1[x] = v;
        }
    }

    /* copy the regions where obmc[] = (uint8_t)256 */
    if(LOG2_OBMC_MAX == 8
        && (mb_x == 0 || mb_x == b_stride-1)
        && (mb_y == 0 || mb_y == b_height-1)){
        if(mb_x == 0)
            x1 = block_w;
        else
            x0 = block_w;
        if(mb_y == 0)
            y1 = block_h;
        else
            y0 = block_h;
        for(y=y0; y<y1; y++)
            memcpy(dst + sx+x0 + (sy+y)*ref_stride, cur + x0 + y*ref_stride, x1-x0);
    }

    if(block_w==16){
        /* FIXME rearrange dsputil to fit 32x32 cmp functions */
        /* FIXME check alignment of the cmp wavelet vs the encoding wavelet */
        /* FIXME cmps overlap but do not cover the wavelet's whole support.
         * So improving the score of one block is not strictly guaranteed
         * to improve the score of the whole frame, thus iterative motion
         * estimation does not always converge. */
        if(s->avctx->me_cmp == FF_CMP_W97)
            distortion = ff_w97_32_c(&s->m, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, 32);
        else if(s->avctx->me_cmp == FF_CMP_W53)
            distortion = ff_w53_32_c(&s->m, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, 32);
        else{
            distortion = 0;
            for(i=0; i<4; i++){
                int off = sx+16*(i&1) + (sy+16*(i>>1))*ref_stride;
                distortion += s->mecc.me_cmp[0](&s->m, src + off, dst + off, ref_stride, 16);
            }
        }
    }else{
        av_assert2(block_w==8);
        distortion = s->mecc.me_cmp[0](&s->m, src + sx + sy*ref_stride, dst + sx + sy*ref_stride, ref_stride, block_w*2);
    }

    if(plane_index==0){
        for(i=0; i<4; i++){
/* ..RRr
 * .RXx.
 * rxx..
 */
            rate += get_block_bits(s, mb_x + (i&1) - (i>>1), mb_y + (i>>1), 1);
        }
        if(mb_x == b_stride-2)
            rate += get_block_bits(s, mb_x + 1, mb_y + 1, 1);
    }
    return distortion + rate*penalty_factor;
}

static int get_4block_rd(FFV1Context *s, int mb_x, int mb_y, int plane_index){
    int i, y2;
    PlaneContext *p= &s->plane[plane_index];
    const int block_size = MB_SIZE >> s->block_max_depth;
    const int block_w    = plane_index ? block_size>>s->chroma_h_shift : block_size;
    const int block_h    = plane_index ? block_size>>s->chroma_v_shift : block_size;
    const uint8_t *obmc  = plane_index ? ff_ffv1_obmc_tab[s->block_max_depth+s->chroma_h_shift] : ff_ffv1_obmc_tab[s->block_max_depth];
    const int obmc_stride= plane_index ? (2*block_size)>>s->chroma_h_shift : 2*block_size;
    const int ref_stride= s->current_picture->linesize[plane_index];
    uint8_t *dst= s->current_picture->data[plane_index];
    uint8_t *src= s-> input_picture->data[plane_index];
    //FIXME zero_dst is const but add_yblock changes dst if add is 0 (this is never the case for dst=zero_dst
    // const has only been removed from zero_dst to suppress a warning
    static IDWTELEM zero_dst[4096]; //FIXME
    const int b_stride = s->b_width << s->block_max_depth;
    const int w= p->width;
    const int h= p->height;
    int distortion= 0;
    int rate= 0;
    const int penalty_factor= get_penalty_factor(s->lambda, s->lambda2, s->avctx->me_cmp);

    av_assert2(s->chroma_h_shift == s->chroma_v_shift); //obmc and square assumtions below

    for(i=0; i<9; i++){
        int mb_x2= mb_x + (i%3) - 1;
        int mb_y2= mb_y + (i/3) - 1;
        int x= block_w*mb_x2 + block_w/2;
        int y= block_h*mb_y2 + block_h/2;

        add_yblock(s, 0, NULL, zero_dst, dst, obmc,
                   x, y, block_w, block_h, w, h, /*dst_stride*/0, ref_stride, obmc_stride, mb_x2, mb_y2, 1, 1, plane_index);

        //FIXME find a cleaner/simpler way to skip the outside stuff
        for(y2= y; y2<0; y2++)
            memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, block_w);
        for(y2= h; y2<y+block_h; y2++)
            memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, block_w);
        if(x<0){
            for(y2= y; y2<y+block_h; y2++)
                memcpy(dst + x + y2*ref_stride, src + x + y2*ref_stride, -x);
        }
        if(x+block_w > w){
            for(y2= y; y2<y+block_h; y2++)
                memcpy(dst + w + y2*ref_stride, src + w + y2*ref_stride, x+block_w - w);
        }

        av_assert1(block_w== 8 || block_w==16);
        distortion += s->mecc.me_cmp[block_w==8](&s->m, src + x + y*ref_stride, dst + x + y*ref_stride, ref_stride, block_h);
    }

    if(plane_index==0){
        BlockNode *b= &s->block[mb_x+mb_y*b_stride];
        int merged= same_block(b,b+1) && same_block(b,b+b_stride) && same_block(b,b+b_stride+1);

/* ..RRRr
 * .RXXx.
 * .RXXx.
 * rxxx.
 */
        if(merged)
            rate = get_block_bits(s, mb_x, mb_y, 2);
        for(i=merged?4:0; i<9; i++){
            static const int dxy[9][2] = {{0,0},{1,0},{0,1},{1,1},{2,0},{2,1},{-1,2},{0,2},{1,2}};
            rate += get_block_bits(s, mb_x + dxy[i][0], mb_y + dxy[i][1], 1);
        }
    }
    return distortion + rate*penalty_factor;
}

static av_always_inline int check_block(FFV1Context *s, int mb_x, int mb_y, int p[3], int intra, uint8_t (*obmc_edged)[MB_SIZE * 2], int *best_rd){
    const int b_stride= s->b_width << s->block_max_depth;
    BlockNode *block= &s->block[mb_x + mb_y * b_stride];
    BlockNode backup= *block;
    unsigned value;
    int rd, index;

    av_assert2(mb_x>=0 && mb_y>=0);
    av_assert2(mb_x<b_stride);

    if(intra){
        block->color[0] = p[0];
        block->color[1] = p[1];
        block->color[2] = p[2];
        block->type |= BLOCK_INTRA;
    }else{
        index= (p[0] + 31*p[1]) & (ME_CACHE_SIZE-1);
        value= s->me_cache_generation + (p[0]>>10) + (p[1]<<6) + (block->ref<<12);
        if(s->me_cache[index] == value)
            return 0;
        s->me_cache[index]= value;

        block->mx= p[0];
        block->my= p[1];
        block->type &= ~BLOCK_INTRA;
    }

    rd= get_block_rd(s, mb_x, mb_y, 0, obmc_edged) + s->intra_penalty * !!intra;

//FIXME chroma
    if(rd < *best_rd){
        *best_rd= rd;
        return 1;
    }else{
        *block= backup;
        return 0;
    }
}

/* special case for int[2] args we discard afterwards,
 * fixes compilation problem with gcc 2.95 */
static av_always_inline int check_block_inter(FFV1Context *s, int mb_x, int mb_y, int p0, int p1, uint8_t (*obmc_edged)[MB_SIZE * 2], int *best_rd){
    int p[2] = {p0, p1};
    return check_block(s, mb_x, mb_y, p, 0, obmc_edged, best_rd);
}

static av_always_inline int check_4block_inter(FFV1Context *s, int mb_x, int mb_y, int p0, int p1, int ref, int *best_rd){
    const int b_stride= s->b_width << s->block_max_depth;
    BlockNode *block= &s->block[mb_x + mb_y * b_stride];
    BlockNode backup[4];
    unsigned value;
    int rd, index;

    /* We don't initialize backup[] during variable declaration, because
     * that fails to compile on MSVC: "cannot convert from 'BlockNode' to
     * 'int16_t'". */
    backup[0] = block[0];
    backup[1] = block[1];
    backup[2] = block[b_stride];
    backup[3] = block[b_stride + 1];

    av_assert2(mb_x>=0 && mb_y>=0);
    av_assert2(mb_x<b_stride);
    av_assert2(((mb_x|mb_y)&1) == 0);

    index= (p0 + 31*p1) & (ME_CACHE_SIZE-1);
    value= s->me_cache_generation + (p0>>10) + (p1<<6) + (block->ref<<12);
    if(s->me_cache[index] == value)
        return 0;
    s->me_cache[index]= value;

    block->mx= p0;
    block->my= p1;
    block->ref= ref;
    block->type &= ~BLOCK_INTRA;
    block[1]= block[b_stride]= block[b_stride+1]= *block;

    rd= get_4block_rd(s, mb_x, mb_y, 0);

//FIXME chroma
    if(rd < *best_rd){
        *best_rd= rd;
        return 1;
    }else{
        block[0]= backup[0];
        block[1]= backup[1];
        block[b_stride]= backup[2];
        block[b_stride+1]= backup[3];
        return 0;
    }
}

static void iterative_me(FFV1Context *s){
    int pass, mb_x, mb_y;
    const int b_width = s->b_width  << s->block_max_depth;
    const int b_height= s->b_height << s->block_max_depth;
    const int b_stride= b_width;
    int color[3];

    {
        RangeCoder r = s->slice_context[0]->c;
        uint8_t state[sizeof(s->block_state)];
        memcpy(state, s->block_state, sizeof(s->block_state));
        for(mb_y= 0; mb_y<s->b_height; mb_y++)
            for(mb_x= 0; mb_x<s->b_width; mb_x++)
                encode_q_branch(s, 0, mb_x, mb_y);
        s->slice_context[0]->c = r;
        memcpy(s->block_state, state, sizeof(s->block_state));
    }

    for(pass=0; pass<25; pass++){
        int change= 0;

        for(mb_y= 0; mb_y<b_height; mb_y++){
            for(mb_x= 0; mb_x<b_width; mb_x++){
                int dia_change, i, j, ref;
                int best_rd= INT_MAX, ref_rd;
                BlockNode backup, ref_b;
                const int index= mb_x + mb_y * b_stride;
                BlockNode *block= &s->block[index];
                BlockNode *tb =                   mb_y            ? &s->block[index-b_stride  ] : NULL;
                BlockNode *lb = mb_x                              ? &s->block[index         -1] : NULL;
                BlockNode *rb = mb_x+1<b_width                    ? &s->block[index         +1] : NULL;
                BlockNode *bb =                   mb_y+1<b_height ? &s->block[index+b_stride  ] : NULL;
                BlockNode *tlb= mb_x           && mb_y            ? &s->block[index-b_stride-1] : NULL;
                BlockNode *trb= mb_x+1<b_width && mb_y            ? &s->block[index-b_stride+1] : NULL;
                BlockNode *blb= mb_x           && mb_y+1<b_height ? &s->block[index+b_stride-1] : NULL;
                BlockNode *brb= mb_x+1<b_width && mb_y+1<b_height ? &s->block[index+b_stride+1] : NULL;
                const int b_w= (MB_SIZE >> s->block_max_depth);
                uint8_t obmc_edged[MB_SIZE * 2][MB_SIZE * 2];

                if(pass && (block->type & BLOCK_OPT))
                    continue;
                block->type |= BLOCK_OPT;

                backup= *block;

                if(!s->me_cache_generation)
                    memset(s->me_cache, 0, sizeof(s->me_cache));
                s->me_cache_generation += 1<<22;

                //FIXME precalculate
                {
                    int x, y;
                    for (y = 0; y < b_w * 2; y++)
                        memcpy(obmc_edged[y], ff_ffv1_obmc_tab[s->block_max_depth] + y * b_w * 2, b_w * 2);
                    if(mb_x==0)
                        for(y=0; y<b_w*2; y++)
                            memset(obmc_edged[y], obmc_edged[y][0] + obmc_edged[y][b_w-1], b_w);
                    if(mb_x==b_stride-1)
                        for(y=0; y<b_w*2; y++)
                            memset(obmc_edged[y]+b_w, obmc_edged[y][b_w] + obmc_edged[y][b_w*2-1], b_w);
                    if(mb_y==0){
                        for(x=0; x<b_w*2; x++)
                            obmc_edged[0][x] += obmc_edged[b_w-1][x];
                        for(y=1; y<b_w; y++)
                            memcpy(obmc_edged[y], obmc_edged[0], b_w*2);
                    }
                    if(mb_y==b_height-1){
                        for(x=0; x<b_w*2; x++)
                            obmc_edged[b_w*2-1][x] += obmc_edged[b_w][x];
                        for(y=b_w; y<b_w*2-1; y++)
                            memcpy(obmc_edged[y], obmc_edged[b_w*2-1], b_w*2);
                    }
                }

                //skip stuff outside the picture
                if(mb_x==0 || mb_y==0 || mb_x==b_width-1 || mb_y==b_height-1){
                    uint8_t *src= s->  input_picture->data[0];
                    uint8_t *dst= s->current_picture->data[0];
                    const int stride= s->current_picture->linesize[0];
                    const int block_w= MB_SIZE >> s->block_max_depth;
                    const int block_h= MB_SIZE >> s->block_max_depth;
                    const int sx= block_w*mb_x - block_w/2;
                    const int sy= block_h*mb_y - block_h/2;
                    const int w= s->plane[0].width;
                    const int h= s->plane[0].height;
                    int y;

                    for(y=sy; y<0; y++)
                        memcpy(dst + sx + y*stride, src + sx + y*stride, block_w*2);
                    for(y=h; y<sy+block_h*2; y++)
                        memcpy(dst + sx + y*stride, src + sx + y*stride, block_w*2);
                    if(sx<0){
                        for(y=sy; y<sy+block_h*2; y++)
                            memcpy(dst + sx + y*stride, src + sx + y*stride, -sx);
                    }
                    if(sx+block_w*2 > w){
                        for(y=sy; y<sy+block_h*2; y++)
                            memcpy(dst + w + y*stride, src + w + y*stride, sx+block_w*2 - w);
                    }
                }

                // intra(black) = neighbors' contribution to the current block
                for(i=0; i < s->nb_planes; i++)
                    color[i]= get_dc(s, mb_x, mb_y, i);

                // get previous score (cannot be cached due to OBMC)
                if(pass > 0 && (block->type&BLOCK_INTRA)){
                    int color0[3]= {block->color[0], block->color[1], block->color[2]};
                    check_block(s, mb_x, mb_y, color0, 1, obmc_edged, &best_rd);
                }else
                    check_block_inter(s, mb_x, mb_y, block->mx, block->my, obmc_edged, &best_rd);

                ref_b= *block;
                ref_rd= best_rd;
                for(ref=0; ref < s->ref_frames; ref++){
                    int16_t (*mvr)[2]= &s->ref_mvs[ref][index];
                    if(s->ref_scores[ref][index] > s->ref_scores[ref_b.ref][index]*3/2) //FIXME tune threshold
                        continue;
                    block->ref= ref;
                    best_rd= INT_MAX;

                    check_block_inter(s, mb_x, mb_y, mvr[0][0], mvr[0][1], obmc_edged, &best_rd);
                    check_block_inter(s, mb_x, mb_y, 0, 0, obmc_edged, &best_rd);
                    if(tb)
                        check_block_inter(s, mb_x, mb_y, mvr[-b_stride][0], mvr[-b_stride][1], obmc_edged, &best_rd);
                    if(lb)
                        check_block_inter(s, mb_x, mb_y, mvr[-1][0], mvr[-1][1], obmc_edged, &best_rd);
                    if(rb)
                        check_block_inter(s, mb_x, mb_y, mvr[1][0], mvr[1][1], obmc_edged, &best_rd);
                    if(bb)
                        check_block_inter(s, mb_x, mb_y, mvr[b_stride][0], mvr[b_stride][1], obmc_edged, &best_rd);

                    /* fullpel ME */
                    //FIXME avoid subpel interpolation / round to nearest integer
                    do{
                        int newx = block->mx;
                        int newy = block->my;
                        int dia_size = s->iterative_dia_size ? s->iterative_dia_size : FFMAX(s->avctx->dia_size, 1);
                        dia_change=0;
                        for(i=0; i < dia_size; i++){
                            for(j=0; j<i; j++){
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx+4*(i-j), newy+(4*j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx-4*(i-j), newy-(4*j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx-(4*j), newy+4*(i-j), obmc_edged, &best_rd);
                                dia_change |= check_block_inter(s, mb_x, mb_y, newx+(4*j), newy-4*(i-j), obmc_edged, &best_rd);
                            }
                        }
                    }while(dia_change);
                    /* subpel ME */
                    do{
                        static const int square[8][2]= {{+1, 0},{-1, 0},{ 0,+1},{ 0,-1},{+1,+1},{-1,-1},{+1,-1},{-1,+1},};
                        dia_change=0;
                        for(i=0; i<8; i++)
                            dia_change |= check_block_inter(s, mb_x, mb_y, block->mx+square[i][0], block->my+square[i][1], obmc_edged, &best_rd);
                    }while(dia_change);
                    //FIXME or try the standard 2 pass qpel or similar

                    mvr[0][0]= block->mx;
                    mvr[0][1]= block->my;
                    if(ref_rd > best_rd){
                        ref_rd= best_rd;
                        ref_b= *block;
                    }
                }
                best_rd= ref_rd;
                *block= ref_b;
                check_block(s, mb_x, mb_y, color, 1, obmc_edged, &best_rd);
                //FIXME RD style color selection
                if(!same_block(block, &backup)){
                    if(tb ) tb ->type &= ~BLOCK_OPT;
                    if(lb ) lb ->type &= ~BLOCK_OPT;
                    if(rb ) rb ->type &= ~BLOCK_OPT;
                    if(bb ) bb ->type &= ~BLOCK_OPT;
                    if(tlb) tlb->type &= ~BLOCK_OPT;
                    if(trb) trb->type &= ~BLOCK_OPT;
                    if(blb) blb->type &= ~BLOCK_OPT;
                    if(brb) brb->type &= ~BLOCK_OPT;
                    change ++;
                }
            }
        }
        av_log(s->avctx, AV_LOG_DEBUG, "pass:%d changed:%d\n", pass, change);
        if(!change)
            break;
    }

    if(s->block_max_depth == 1){
        int change= 0;
        for(mb_y= 0; mb_y<b_height; mb_y+=2){
            for(mb_x= 0; mb_x<b_width; mb_x+=2){
                int i;
                int best_rd, init_rd;
                const int index= mb_x + mb_y * b_stride;
                BlockNode *b[4];

                b[0]= &s->block[index];
                b[1]= b[0]+1;
                b[2]= b[0]+b_stride;
                b[3]= b[2]+1;
                if(same_block(b[0], b[1]) &&
                   same_block(b[0], b[2]) &&
                   same_block(b[0], b[3]))
                    continue;

                if(!s->me_cache_generation)
                    memset(s->me_cache, 0, sizeof(s->me_cache));
                s->me_cache_generation += 1<<22;

                init_rd= best_rd= get_4block_rd(s, mb_x, mb_y, 0);

                //FIXME more multiref search?
                check_4block_inter(s, mb_x, mb_y,
                                   (b[0]->mx + b[1]->mx + b[2]->mx + b[3]->mx + 2) >> 2,
                                   (b[0]->my + b[1]->my + b[2]->my + b[3]->my + 2) >> 2, 0, &best_rd);

                for(i=0; i<4; i++)
                    if(!(b[i]->type&BLOCK_INTRA))
                        check_4block_inter(s, mb_x, mb_y, b[i]->mx, b[i]->my, b[i]->ref, &best_rd);

                if(init_rd != best_rd)
                    change++;
            }
        }
        av_log(s->avctx, AV_LOG_DEBUG, "pass:4mv changed:%d\n", change*4);
    }
}

static void encode_blocks(FFV1Context *s, int search){
    int x, y;
    int w= s->b_width;
    int h= s->b_height;
    RangeCoder *const c = &s->slice_context[0]->c;

    if(s->motion_est == FF_ME_ITER && !s->key_frame && search)
        iterative_me(s);

    for(y=0; y<h; y++){
        if(c->bytestream_end - c->bytestream < w*MB_SIZE*MB_SIZE*3){ //FIXME nicer limit
            av_log(s->avctx, AV_LOG_ERROR, "encoded frame too large\n");
            return;
        }
        for(x=0; x<w; x++){
            if(s->motion_est == FF_ME_ITER || !search)
                encode_q_branch2(s, 0, x, y);
            else
                encode_q_branch (s, 0, x, y);
        }
    }
}

static int encode_slice(AVCodecContext *c, void *arg)
{
    FFV1Context *fs  = *(void **)arg;
    FFV1Context *f   = fs->avctx->priv_data;
    int width        = fs->slice_width;
    int height       = fs->slice_height;
    int x            = fs->slice_x;
    int y            = fs->slice_y;
    const AVFrame *const p = f->picture.f;
    const int ps     = av_pix_fmt_desc_get(c->pix_fmt)->comp[0].step;
    int ret;
    RangeCoder c_bak = fs->c;
    const uint8_t *planes[3] = {p->data[0] + ps*x + y*p->linesize[0],
                                p->data[1] + ps*x + y*p->linesize[1],
                                p->data[2] + ps*x + y*p->linesize[2]};

    fs->slice_coding_mode = 0;
    if (f->version > 3) {
        choose_rct_params(fs, planes, p->linesize, width, height);
    } else {
        fs->slice_rct_by_coef = 1;
        fs->slice_rct_ry_coef = 1;
    }

retry:
    if (f->key_frame)
        ff_ffv1_clear_slice_state(f, fs);
    if (f->version > 2) {
        encode_slice_header(f, fs);
    }
    if (fs->ac == AC_GOLOMB_RICE) {
        if (f->version > 2)
            put_rac(&fs->c, (uint8_t[]) { 129 }, 0);
        fs->ac_byte_count = f->version > 2 || (!x && !y) ? ff_rac_terminate(&fs->c) : 0;
        init_put_bits(&fs->pb,
                      fs->c.bytestream_start + fs->ac_byte_count,
                      fs->c.bytestream_end - fs->c.bytestream_start - fs->ac_byte_count);
    }

    if (f->colorspace == 0 && c->pix_fmt != AV_PIX_FMT_YA8) {
        const int chroma_width  = AV_CEIL_RSHIFT(width,  f->chroma_h_shift);
        const int chroma_height = AV_CEIL_RSHIFT(height, f->chroma_v_shift);
        const int cx            = x >> f->chroma_h_shift;
        const int cy            = y >> f->chroma_v_shift;

        ret = encode_plane(fs, p->data[0] + ps*x + y*p->linesize[0], width, height, p->linesize[0], 0, 1);

        if (f->chroma_planes) {
            ret |= encode_plane(fs, p->data[1] + ps*cx+cy*p->linesize[1], chroma_width, chroma_height, p->linesize[1], 1, 1);
            ret |= encode_plane(fs, p->data[2] + ps*cx+cy*p->linesize[2], chroma_width, chroma_height, p->linesize[2], 1, 1);
        }
        if (fs->transparency)
            ret |= encode_plane(fs, p->data[3] + ps*x + y*p->linesize[3], width, height, p->linesize[3], 2, 1);
    } else if (c->pix_fmt == AV_PIX_FMT_YA8) {
        ret  = encode_plane(fs, p->data[0] +     ps*x + y*p->linesize[0], width, height, p->linesize[0], 0, 2);
        ret |= encode_plane(fs, p->data[0] + 1 + ps*x + y*p->linesize[0], width, height, p->linesize[0], 1, 2);
    } else {
        ret = encode_rgb_frame(fs, planes, width, height, p->linesize);
    }
    emms_c();

    if (ret < 0) {
        av_assert0(fs->slice_coding_mode == 0);
        if (fs->version < 4 || !fs->ac) {
            av_log(c, AV_LOG_ERROR, "Buffer too small\n");
            return ret;
        }
        av_log(c, AV_LOG_DEBUG, "Coding slice as PCM\n");
        fs->slice_coding_mode = 1;
        fs->c = c_bak;
        goto retry;
    }

    return 0;
}

static int encode_frame(AVCodecContext *avctx, AVPacket *pkt,
                        const AVFrame *pict, int *got_packet)
{
    FFV1Context *f      = avctx->priv_data;
    if (f->p_frame) {
        if (f->last_picture.f)
            av_frame_unref(f->last_picture.f);
        FFSWAP(ThreadFrame, f->picture, f->last_picture);
    }
    RangeCoder *const c = &f->slice_context[0]->c;
    AVFrame *const p    = f->picture.f;
    int used_count      = 0;
    uint8_t keystate    = 128;
    uint8_t *buf_p;
    AVFrame *pic = NULL;
    const int width= f->avctx->width;
    const int height= f->avctx->height;
    int plane_index, i, ret;
    int64_t maxsize =   AV_INPUT_BUFFER_MIN_SIZE
                      + avctx->width*avctx->height*35LL*4;

    if(!pict) {
        if (avctx->flags & AV_CODEC_FLAG_PASS1) {
            int j, k, m;
            char *p   = avctx->stats_out;
            char *end = p + STATS_OUT_SIZE;

            memset(f->rc_stat, 0, sizeof(f->rc_stat));
            for (i = 0; i < f->quant_table_count; i++)
                memset(f->rc_stat2[i], 0, f->context_count[i] * sizeof(*f->rc_stat2[i]));

            av_assert0(f->slice_count == f->max_slice_count);
            for (j = 0; j < f->slice_count; j++) {
                FFV1Context *fs = f->slice_context[j];
                for (i = 0; i < 256; i++) {
                    f->rc_stat[i][0] += fs->rc_stat[i][0];
                    f->rc_stat[i][1] += fs->rc_stat[i][1];
                }
                for (i = 0; i < f->quant_table_count; i++) {
                    for (k = 0; k < f->context_count[i]; k++)
                        for (m = 0; m < 32; m++) {
                            f->rc_stat2[i][k][m][0] += fs->rc_stat2[i][k][m][0];
                            f->rc_stat2[i][k][m][1] += fs->rc_stat2[i][k][m][1];
                        }
                }
            }

            for (j = 0; j < 256; j++) {
                snprintf(p, end - p, "%" PRIu64 " %" PRIu64 " ",
                        f->rc_stat[j][0], f->rc_stat[j][1]);
                p += strlen(p);
            }
            snprintf(p, end - p, "\n");

            for (i = 0; i < f->quant_table_count; i++) {
                for (j = 0; j < f->context_count[i]; j++)
                    for (m = 0; m < 32; m++) {
                        snprintf(p, end - p, "%" PRIu64 " %" PRIu64 " ",
                                f->rc_stat2[i][j][m][0], f->rc_stat2[i][j][m][1]);
                        p += strlen(p);
                    }
            }
            snprintf(p, end - p, "%d\n", f->gob_count);
        }
        return 0;
    }

    if (f->version > 3)
        maxsize = AV_INPUT_BUFFER_MIN_SIZE + avctx->width*avctx->height*3LL*4;

    if (f->p_frame) {
        maxsize += f->b_width*f->b_height*MB_SIZE*MB_SIZE*3;
    }

    if ((ret = ff_alloc_packet2(avctx, pkt, maxsize, 0)) < 0)
        return ret;

    ff_init_range_encoder(c, pkt->data, pkt->size);
    ff_build_rac_states(c, 0.05 * (1LL << 32), 256 - 8);

    if (f->p_frame) {
        //av_frame_copy_props(f->input_picture, pict);
        //av_image_copy(f->input_picture->data, f->input_picture->linesize, pict->data, pict->linesize, f->input_picture->format, width, height);
        av_frame_copy(f->input_picture, pict);
        for(i=0; i < f->nb_planes; i++)
        {
            int hshift= i ? f->chroma_h_shift : 0;
            int vshift= i ? f->chroma_v_shift : 0;
            f->mpvencdsp.draw_edges(f->input_picture->data[i], f->input_picture->linesize[i],
                                    AV_CEIL_RSHIFT(width, hshift), AV_CEIL_RSHIFT(height, vshift),
                                    EDGE_WIDTH >> hshift, EDGE_WIDTH >> vshift,
                                    EDGE_TOP | EDGE_BOTTOM);
        }
        emms_c();
        pic = f->input_picture;
        pic->pict_type = pict->pict_type;
        pic->quality = pict->quality;

        f->m.picture_number= avctx->frame_number;
    }

    av_frame_unref(p);
    if ((ret = av_frame_ref(p, pict)) < 0)
        return ret;
#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
    avctx->coded_frame->pict_type = AV_PICTURE_TYPE_I;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    if (avctx->gop_size == 0 || f->picture_number % avctx->gop_size == 0) {
        put_rac(c, &keystate, 1);
        f->key_frame = 1;
        f->gob_count++;
        write_header(f);
    } else {
        put_rac(c, &keystate, 0);
        f->key_frame = 0;
    }

    if (f->p_frame) {
        //RangeCoder tmp = *c;
        write_p_header(f);
        //*c = tmp;

        f->m.pict_type = pic->pict_type = f->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

        if (pic->quality) {
            f->lambda = pic->quality * 3/2;
        }
        if (!pic->quality && (avctx->flags & AV_CODEC_FLAG_QSCALE)) {
            f->lambda = 0;
        }

        if (f->current_picture->data[0]
    /*#if FF_API_EMU_EDGE
            && !(f->avctx->flags&CODEC_FLAG_EMU_EDGE)
    #endif*/
            ) {
            int w = f->avctx->width;
            int h = f->avctx->height;

            f->mpvencdsp.draw_edges(f->current_picture->data[0],
                                    f->current_picture->linesize[0], w   , h   ,
                                    EDGE_WIDTH  , EDGE_WIDTH  , EDGE_TOP | EDGE_BOTTOM);
            if (f->current_picture->data[2]) {
                f->mpvencdsp.draw_edges(f->current_picture->data[1],
                                        f->current_picture->linesize[1], w>>f->chroma_h_shift, h>>f->chroma_v_shift,
                                        EDGE_WIDTH>>f->chroma_h_shift, EDGE_WIDTH>>f->chroma_v_shift, EDGE_TOP | EDGE_BOTTOM);
                f->mpvencdsp.draw_edges(f->current_picture->data[2],
                                        f->current_picture->linesize[2], w>>f->chroma_h_shift, h>>f->chroma_v_shift,
                                        EDGE_WIDTH>>f->chroma_h_shift, EDGE_WIDTH>>f->chroma_v_shift, EDGE_TOP | EDGE_BOTTOM);
            }
        }

        ff_ffv1_frame_start(f);
#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
        av_frame_unref(avctx->coded_frame);
        ret = av_frame_ref(avctx->coded_frame, f->current_picture);
        if (ret < 0)
            return ret;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

        f->m.current_picture_ptr= &f->m.current_picture;
        f->m.current_picture.f = f->current_picture;
        f->m.current_picture.f->pts = pict->pts;
        if(pic->pict_type == AV_PICTURE_TYPE_P){
            int block_width = (width +15)>>4;
            int block_height= (height+15)>>4;
            int stride= f->current_picture->linesize[0];

            av_assert0(f->current_picture->data[0]);
            av_assert0(f->last_pictures[0]->data[0]);

            f->m.avctx= f->avctx;
            f->m.last_picture.f = f->last_pictures[0];
            f->m.new_picture.f = f->input_picture;
            f->m.last_picture_ptr= &f->m.last_picture;
            f->m.linesize = stride;
            f->m.uvlinesize= f->current_picture->linesize[1];
            f->m.width = width;
            f->m.height= height;
            f->m.mb_width = block_width;
            f->m.mb_height= block_height;
            f->m.mb_stride=   f->m.mb_width+1;
            f->m.b8_stride= 2*f->m.mb_width+1;
            f->m.f_code=1;
            f->m.pict_type = pic->pict_type;
#if FF_API_MOTION_EST
FF_DISABLE_DEPRECATION_WARNINGS
            f->m.me_method= f->avctx->me_method;
FF_ENABLE_DEPRECATION_WARNINGS
#endif
            f->m.motion_est= f->motion_est;
            f->m.me.scene_change_score=0;
            f->m.me.dia_size = avctx->dia_size;
            f->m.quarter_sample= (f->avctx->flags & AV_CODEC_FLAG_QPEL)!=0;
            f->m.out_format= FMT_H263;
            f->m.unrestricted_mv= 1;

            f->m.lambda = f->lambda;
            f->m.qscale= (f->m.lambda*139 + FF_LAMBDA_SCALE*64) >> (FF_LAMBDA_SHIFT + 7);
            f->lambda2= f->m.lambda2= (f->m.lambda*f->m.lambda + FF_LAMBDA_SCALE/2) >> FF_LAMBDA_SHIFT;

            f->m.mecc= f->mecc; //move
            f->m.qdsp= f->qdsp; //move
            f->m.hdsp = f->hdsp;
            ff_init_me(&f->m);
            f->hdsp = f->m.hdsp;
            f->mecc= f->m.mecc;
        }

        f->m.pict_type = pic->pict_type;

        ff_ffv1_common_init_after_header(avctx);

        f->m.misc_bits = 8*(c->bytestream - c->bytestream_start);
        //tmp = *c;
        encode_blocks(f, 1);
        //*c = tmp;
        f->m.mv_bits = 8*(c->bytestream - c->bytestream_start) - f->m.misc_bits;

        for(plane_index=0; plane_index < f->nb_planes; plane_index++){
            PlaneContext *p= &f->plane[plane_index];
            int w= p->width;
            int h= p->height;

            if(pic->pict_type == AV_PICTURE_TYPE_I){
                //av_frame_copy_props(f->current_picture, pict);
                //av_image_copy(f->current_picture->data, f->current_picture->linesize, pict->data, pict->linesize, f->current_picture->format, width, height);
                av_frame_copy(f->current_picture, pict);
                break;
            } else {
                memset(f->spatial_idwt_buffer, 0, sizeof(IDWTELEM)*w*h);
                predict_plane(f, f->spatial_idwt_buffer, plane_index, 1);
            }
        }

        if (!f->key_frame) {
            if ((ret = ff_frame_diff(f, pict)) < 0) {
                return ret;
            }
            //av_frame_copy(f->picture.f, f->current_picture);
            av_frame_copy(f->current_picture, pict);
        }

        ff_ffv1_release_buffer(avctx);

        f->current_picture->coded_picture_number = avctx->frame_number;
        f->current_picture->pict_type = pic->pict_type;
        f->current_picture->quality = pic->quality;
        f->m.frame_bits = 8*(c->bytestream - c->bytestream_start);
        f->m.p_tex_bits = f->m.frame_bits - f->m.misc_bits - f->m.mv_bits;
        f->m.current_picture.f->display_picture_number =
        f->m.current_picture.f->coded_picture_number   = avctx->frame_number;
        f->m.current_picture.f->quality                = pic->quality;
        f->m.total_bits += 8*(c->bytestream - c->bytestream_start);

        f->m.last_pict_type = f->m.pict_type;
        //avctx->frame_bits = f->m.frame_bits;
        //avctx->mv_bits = f->m.mv_bits;
        //avctx->misc_bits = f->m.misc_bits;
        //avctx->p_tex_bits = f->m.p_tex_bits;

        emms_c();
    }

    if (f->ac == AC_RANGE_CUSTOM_TAB) {
        int i;
        for (i = 1; i < 256; i++) {
            c->one_state[i]        = f->state_transition[i];
            c->zero_state[256 - i] = 256 - c->one_state[i];
        }
    }

    for (i = 1; i < f->slice_count; i++) {
        FFV1Context *fs = f->slice_context[i];
        uint8_t *start  = pkt->data + (pkt->size - used_count) * (int64_t)i / f->slice_count;
        int len         = pkt->size / f->slice_count;
        ff_init_range_encoder(&fs->c, start, len);
    }
    avctx->execute(avctx, encode_slice, &f->slice_context[0], NULL,
                   f->slice_count, sizeof(void *));

    buf_p = pkt->data;
    for (i = 0; i < f->slice_count; i++) {
        FFV1Context *fs = f->slice_context[i];
        int bytes;

        if (fs->ac != AC_GOLOMB_RICE) {
            uint8_t state = 129;
            put_rac(&fs->c, &state, 0);
            bytes = ff_rac_terminate(&fs->c);
        } else {
            flush_put_bits(&fs->pb); // FIXME: nicer padding
            bytes = fs->ac_byte_count + (put_bits_count(&fs->pb) + 7) / 8;
        }
        if (i > 0 || f->version > 2) {
            av_assert0(bytes < pkt->size / f->slice_count);
            memmove(buf_p, fs->c.bytestream_start, bytes);
            av_assert0(bytes < (1 << 24));
            AV_WB24(buf_p + bytes, bytes);
            bytes += 3;
        }
        if (f->ec) {
            unsigned v;
            buf_p[bytes++] = 0;
            v = av_crc(av_crc_get_table(AV_CRC_32_IEEE), 0, buf_p, bytes);
            AV_WL32(buf_p + bytes, v);
            bytes += 4;
        }
        buf_p += bytes;
    }

    if (avctx->flags & AV_CODEC_FLAG_PASS1)
        avctx->stats_out[0] = '\0';

#if FF_API_CODED_FRAME
FF_DISABLE_DEPRECATION_WARNINGS
    avctx->coded_frame->key_frame = f->key_frame;
FF_ENABLE_DEPRECATION_WARNINGS
#endif

    f->picture_number++;
    pkt->size   = buf_p - pkt->data;
    pkt->pts    =
    pkt->dts    = pict->pts;
    pkt->flags |= AV_PKT_FLAG_KEY * f->key_frame;
    *got_packet = 1;

    if (f->p_frame) {
        if (f->picture.f)
            av_frame_unref(f->picture.f);
        if ((ret = av_frame_ref(f->picture.f, pict)) < 0)
            return ret;
        if (f->last_picture.f)
            av_frame_unref(f->last_picture.f);
    }

    //fprintf(stderr, "size: %d\n", pkt->size);
    //fflush(stderr);

    return 0;
}

static av_cold int encode_close(AVCodecContext *avctx)
{
    FFV1Context *f = avctx->priv_data;

    ff_ffv1_close(avctx);
    av_frame_free(&f->input_picture);
    return 0;
}

#define OFFSET(x) offsetof(FFV1Context, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    FF_MPV_COMMON_OPTS
    { "iter",           NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ITER }, 0, 0, FF_MPV_OPT_FLAGS, "motion_est" },
    { "slicecrc", "Protect slices with CRCs", OFFSET(ec), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, VE },
    { "pframe", "Using P frames", OFFSET(p_frame), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
    { "coder", "Coder type", OFFSET(ac), AV_OPT_TYPE_INT,
            { .i64 = 0 }, -2, 2, VE, "coder" },
        { "rice", "Golomb rice", 0, AV_OPT_TYPE_CONST,
            { .i64 = AC_GOLOMB_RICE }, INT_MIN, INT_MAX, VE, "coder" },
        { "range_def", "Range with default table", 0, AV_OPT_TYPE_CONST,
            { .i64 = AC_RANGE_DEFAULT_TAB_FORCE }, INT_MIN, INT_MAX, VE, "coder" },
        { "range_tab", "Range with custom table", 0, AV_OPT_TYPE_CONST,
            { .i64 = AC_RANGE_CUSTOM_TAB }, INT_MIN, INT_MAX, VE, "coder" },
        { "ac", "Range with custom table (the ac option exists for compatibility and is deprecated)", 0, AV_OPT_TYPE_CONST,
            { .i64 = 1 }, INT_MIN, INT_MAX, VE, "coder" },
    { "context", "Context model", OFFSET(context_model), AV_OPT_TYPE_INT,
            { .i64 = 0 }, 0, 1, VE },

    { NULL }
};

static const AVClass ffv1_class = {
    .class_name = "ffv1 encoder",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

#if FF_API_CODER_TYPE
static const AVCodecDefault ffv1_defaults[] = {
    { "coder", "-1" },
    { "me_method", "iter" },
    { "flags", "+qpel+mv4" },
    { NULL },
};
#endif

AVCodec ff_ffv1_encoder = {
    .name           = "ffv1",
    .long_name      = NULL_IF_CONFIG_SMALL("FFmpeg video codec #1"),
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_FFV1,
    .priv_data_size = sizeof(FFV1Context),
    .init           = encode_init,
    .encode2        = encode_frame,
    .close          = encode_close,
    .capabilities   = AV_CODEC_CAP_SLICE_THREADS | AV_CODEC_CAP_DELAY,
    .pix_fmts       = (const enum AVPixelFormat[]) {
        AV_PIX_FMT_YUV420P,   AV_PIX_FMT_YUVA420P,  AV_PIX_FMT_YUVA422P,  AV_PIX_FMT_YUV444P,
        AV_PIX_FMT_YUVA444P,  AV_PIX_FMT_YUV440P,   AV_PIX_FMT_YUV422P,   AV_PIX_FMT_YUV411P,
        AV_PIX_FMT_YUV410P,   AV_PIX_FMT_0RGB32,    AV_PIX_FMT_RGB32,     AV_PIX_FMT_YUV420P16,
        AV_PIX_FMT_YUV422P16, AV_PIX_FMT_YUV444P16, AV_PIX_FMT_YUV444P9,  AV_PIX_FMT_YUV422P9,
        AV_PIX_FMT_YUV420P9,  AV_PIX_FMT_YUV420P10, AV_PIX_FMT_YUV422P10, AV_PIX_FMT_YUV444P10,
        AV_PIX_FMT_YUVA444P16, AV_PIX_FMT_YUVA422P16, AV_PIX_FMT_YUVA420P16,
        AV_PIX_FMT_YUVA444P10, AV_PIX_FMT_YUVA422P10, AV_PIX_FMT_YUVA420P10,
        AV_PIX_FMT_YUVA444P9, AV_PIX_FMT_YUVA422P9, AV_PIX_FMT_YUVA420P9,
        AV_PIX_FMT_GRAY16,    AV_PIX_FMT_GRAY8,     AV_PIX_FMT_GBRP9,     AV_PIX_FMT_GBRP10,
        AV_PIX_FMT_GBRP12,    AV_PIX_FMT_GBRP14,
        AV_PIX_FMT_YA8,
        AV_PIX_FMT_NONE

    },
#if FF_API_CODER_TYPE
    .defaults       = ffv1_defaults,
#endif
    .priv_class     = &ffv1_class,
};
