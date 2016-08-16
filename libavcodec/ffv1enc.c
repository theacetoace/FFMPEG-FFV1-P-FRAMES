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

#include "obme.h"

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
    AVFrame *prev     = f->obmc.current_picture;
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
                f->c_image_line_buf[x] = (f->c_image_line_buf[x] - f->p_image_line_buf[x] + (max_val >> 2)) & (max_val - 1);
            }
            av_write_image_line(f->c_image_line_buf,
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

typedef struct RangeEncoderContext {
    RangeCoder c;
    uint8_t buffer[1024];
    uint8_t state[128 + 32*128];
    uint8_t *pbbak;
    uint8_t *pbbak_start;
    int base_bits;
} RangeEncoderContext;

static void put_encoder_rac(ObmcCoderContext *c, int ctx, int v)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    RangeCoder *rc = &f->slice_context[0]->c;
    uint8_t *state = f->block_state;
    if (c->priv_data) {
        RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;
        rc = &coder->c; state = coder->state;
    }
    put_rac(rc, &state[ctx], v);
}

static void put_encoder_symbol(ObmcCoderContext *c, int ctx, int v, int sign)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    RangeCoder *rc = &f->slice_context[0]->c;
    uint8_t *state = f->block_state;
    if (c->priv_data) {
        RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;
        rc = &coder->c; state = coder->state;
    }
    put_symbol(rc, &state[ctx], v, sign);
}

static void ff_ffv1_init_encode_callbacks(ObmcCoderContext *, AVCodecContext *);

static void init_frame_encoder(AVCodecContext *avctx, ObmcCoderContext *c)
{
    FFV1Context *f = (FFV1Context *)avctx->priv_data;
    RangeCoder *const rc = &f->slice_context[0]->c;
    RangeEncoderContext *coder = av_mallocz(sizeof(RangeEncoderContext));
    c->priv_data = coder;

    coder->pbbak = rc->bytestream;
    coder->pbbak_start = rc->bytestream_start;
    coder->base_bits = get_rac_count(rc) - 8*(rc->bytestream - rc->bytestream_start);
    coder->c = *rc;
    coder->c.bytestream_start = coder->c.bytestream = coder->buffer; //FIXME end/start? and at the other stoo
    memcpy(coder->state, f->block_state, sizeof(f->block_state));

    ff_ffv1_init_encode_callbacks(c, avctx);
}

static void free_coder(ObmcCoderContext *c)
{
    if (c->priv_data) {
        av_freep(&c->priv_data);
    }
}

static void copy_coder(ObmcCoderContext *c)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    RangeCoder *const rc = &f->slice_context[0]->c;
    RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;

    int len = coder->c.bytestream - coder->c.bytestream_start;

    memcpy(coder->pbbak, coder->buffer, len);
    *rc = coder->c;
    rc->bytestream_start= coder->pbbak_start;
    rc->bytestream= coder->pbbak + len;
    memcpy(f->block_state, coder->state, sizeof(f->block_state));
}

static void reset_coder(ObmcCoderContext *c)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    RangeCoder *const rc = &f->slice_context[0]->c;
    RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;

    *rc = coder->c;
    rc->bytestream_start= coder->pbbak_start;
    rc->bytestream= coder->pbbak;
    memcpy(f->block_state, coder->state, sizeof(f->block_state));
}

static void put_level_break(ObmcCoderContext *c, int ctx, int v)
{
    put_encoder_rac(c, ctx, v);
}

static void put_block_type  (struct ObmcCoderContext *c, int ctx, int type)
{
    put_encoder_rac(c, ctx, type);
}

static void put_best_ref    (struct ObmcCoderContext *c, int ctx, int best_ref)
{
    put_encoder_symbol(c, ctx, best_ref, 0);
}

static void put_block_mv    (struct ObmcCoderContext *c, int ctx_mx, int ctx_my, int mx, int my)
{
    put_encoder_symbol(c, ctx_mx, mx, 1);
    put_encoder_symbol(c, ctx_my, my, 1);
}

static void put_block_color (struct ObmcCoderContext *c, int ctx_l, int ctx_cb, int ctx_cr, int l, int cb, int cr)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    put_encoder_symbol(c, ctx_l, l, 1);
    if (f->obmc.nb_planes > 2) {
        put_encoder_symbol(c, ctx_cb, cb, 1);
        put_encoder_symbol(c, ctx_cr, cr, 1);
    }
}

static int get_coder_bits(ObmcCoderContext *c)
{
    RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;
    return get_rac_count(&coder->c) - coder->base_bits;
}

static int get_coder_available_bytes(ObmcCoderContext *c)
{
    FFV1Context *f = (FFV1Context *)c->avctx->priv_data;
    RangeCoder *rc = &f->slice_context[0]->c;
    if (c->priv_data) {
        RangeEncoderContext *coder = (RangeEncoderContext *)c->priv_data;
        rc = &coder->c;
    }
    return rc->bytestream_end - rc->bytestream;
}

static void ff_ffv1_init_encode_callbacks(ObmcCoderContext *c, AVCodecContext *avctx)
{
    c->avctx            = avctx;
    c->put_level_break  = put_level_break;
    c->put_block_type   = put_block_type;
    c->put_block_color  = put_block_color;
    c->put_best_ref     = put_best_ref;
    c->put_block_mv     = put_block_mv;

    c->init_frame_coder = init_frame_encoder;
    c->reset_coder      = reset_coder;
    c->copy_coder       = copy_coder;
    c->free             = free_coder;

    c->get_bits         = get_coder_bits;
    c->available_bytes  = get_coder_available_bytes;

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
        memset(f->block_state, MID_STATE, sizeof(f->block_state));
        put_symbol(c, state, f->obmc.max_ref_frames-1, 0);
    }
    if (!f->key_frame) { //FIXME update_mc
        for(plane_index=0; plane_index<FFMIN(f->obmc.nb_planes, 2); plane_index++){
            PlaneObmc *p= &f->obmc.plane[plane_index];
            put_rac(c, state, p->diag_mc);
            put_symbol(c, state, p->htaps/2-1, 0);
            for(i= p->htaps/2; i; i--)
                put_symbol(c, state, FFABS(p->hcoeff[i]), 0);
        }
    }

    put_symbol(c, state, f->obmc.mv_scale, 0);
    put_symbol(c, state, f->obmc.block_max_depth, 0);
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
    int i, j, k, m, ret;

    if ((ret = ff_ffv1_common_init(avctx)) < 0)
        return ret;

    s->version = 0;

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

    if (avctx->width % 16 || avctx->height % 16) {
        s->p_frame = 0;
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

    ff_obmc_encode_init(&s->obmc, avctx);
    ff_ffv1_init_encode_callbacks(&s->obmc.obmc_coder, avctx);

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
        maxsize += f->obmc.b_width*f->obmc.b_height*MB_SIZE*MB_SIZE*3;
    }

    if ((ret = ff_alloc_packet2(avctx, pkt, maxsize, 0)) < 0)
        return ret;

    ff_init_range_encoder(c, pkt->data, pkt->size);
    ff_build_rac_states(c, 0.05 * (1LL << 32), 256 - 8);

    if (f->p_frame) {
        av_frame_copy(f->obmc.input_picture, pict);
        for(i=0; i < f->obmc.nb_planes; i++)
        {
            int hshift= i ? f->chroma_h_shift : 0;
            int vshift= i ? f->chroma_v_shift : 0;
            f->obmc.mpvencdsp.draw_edges(f->obmc.input_picture->data[i], f->obmc.input_picture->linesize[i],
                                    AV_CEIL_RSHIFT(width, hshift), AV_CEIL_RSHIFT(height, vshift),
                                    EDGE_WIDTH >> hshift, EDGE_WIDTH >> vshift,
                                    EDGE_TOP | EDGE_BOTTOM);
        }
        emms_c();
        pic = f->obmc.input_picture;
        pic->pict_type = pict->pict_type;
        pic->quality = pict->quality;

        f->obmc.m.picture_number= avctx->frame_number;
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
        f->obmc.keyframe = 1;
        f->gob_count++;
        write_header(f);
    } else {
        put_rac(c, &keystate, 0);
        f->key_frame = 0;
        f->obmc.keyframe = 0;
    }

    if (f->p_frame) {
        write_p_header(f);

        f->obmc.m.pict_type = pic->pict_type = f->key_frame ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_P;

        ff_obmc_pre_encode_frame(&f->obmc, avctx, pict);

        ff_obmc_common_init_after_header(&f->obmc);

        f->obmc.m.misc_bits = 8*(c->bytestream - c->bytestream_start);
        ff_obmc_encode_blocks(&f->obmc, 1);
        f->obmc.m.mv_bits = 8*(c->bytestream - c->bytestream_start) - f->obmc.m.misc_bits;

        for(plane_index=0; plane_index < f->obmc.nb_planes; plane_index++){
            PlaneObmc *p= &f->obmc.plane[plane_index];
            int w= p->width;
            int h= p->height;

            if(pic->pict_type == AV_PICTURE_TYPE_I) {
                av_frame_copy(f->obmc.current_picture, pict);
                break;
            } else {
                memset(f->obmc.spatial_idwt_buffer, 0, sizeof(IDWTELEM)*w*h);
                predict_plane(&f->obmc, f->obmc.spatial_idwt_buffer, plane_index, 1);
            }
        }

        if (!f->key_frame) {
            if ((ret = ff_frame_diff(f, pict)) < 0) {
                return ret;
            }
            av_frame_copy(f->obmc.current_picture, pict);
        }

        ff_obmc_release_buffer(&f->obmc);

        f->obmc.current_picture->coded_picture_number = avctx->frame_number;
        f->obmc.current_picture->pict_type = pic->pict_type;
        f->obmc.current_picture->quality = pic->quality;
        f->obmc.m.frame_bits = 8*(c->bytestream - c->bytestream_start);
        f->obmc.m.p_tex_bits = f->obmc.m.frame_bits - f->obmc.m.misc_bits - f->obmc.m.mv_bits;
        f->obmc.m.current_picture.f->display_picture_number =
        f->obmc.m.current_picture.f->coded_picture_number   = avctx->frame_number;
        f->obmc.m.current_picture.f->quality                = pic->quality;
        f->obmc.m.total_bits += 8*(c->bytestream - c->bytestream_start);

        f->obmc.m.last_pict_type = f->obmc.m.pict_type;

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

    return 0;
}

static av_cold int encode_close(AVCodecContext *avctx)
{
    FFV1Context *f = avctx->priv_data;

    ff_ffv1_close(avctx);
    av_frame_free(&f->obmc.input_picture);
    return 0;
}

#define OFFSET(x) offsetof(FFV1Context, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    FF_MPV_COMMON_OPTS
    { "iter",           NULL, 0, AV_OPT_TYPE_CONST, { .i64 = FF_ME_ITER }, 0, 0, FF_MPV_OPT_FLAGS, "motion_est" },
    { "slicecrc", "Protect slices with CRCs", OFFSET(ec), AV_OPT_TYPE_BOOL, { .i64 = -1 }, -1, 1, VE },
    { "pframe", "Use P frames", OFFSET(p_frame), AV_OPT_TYPE_BOOL, { .i64 = 0 }, 0, 1, VE },
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
