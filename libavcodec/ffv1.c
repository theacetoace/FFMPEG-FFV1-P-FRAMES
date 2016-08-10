/*
 * FFV1 codec for libavcodec
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
 * FF Video Codec 1 (a lossless codec)
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
#include "rangecoder.h"
#include "golomb.h"
#include "mathops.h"
#include "ffv1.h"
#include "ffv1data.h"
#include "me_cmp.h"
#include "h263.h"

static void copy_image_with_border(
    FFV1Context *f, ColorImage *out_img,
    const AVFrame *input_image, int req_border)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(input_image->format);
    int width  = f->width;
    int height = f->height;
    int i, y, k;
    const int cw = AV_CEIL_RSHIFT(width, desc->log2_chroma_w);
    const int ch = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);
    
    const int border = (req_border+15)/16*16;
    
    for (i = 0; i < desc->nb_components; i++) {
        const int w1 = (i == 1 || i == 2) ? cw : width;
        const int h1 = (i == 1 || i == 2) ? ch : height;
        
        const int out_stride = (w1+2*border);
        const int out_total_height = (h1+2*border);
        
        memset(out_img->plane[i].mem_start, 0, out_stride*out_total_height*sizeof(*out_img->plane[i].mem_start));
        
        uint16_t* const memory_ptr   = out_img->plane[i].mem_start;
        uint16_t* const output_image = out_img->plane[i].img; // logical output image origin (0,0)
        
        // copy main image content
    
        for (y = 0; y < h1; y++) {
            memset(f->c_image_line_buf, 0, w1 * sizeof(*f->c_image_line_buf));
            av_read_image_line(f->c_image_line_buf,
                               (void *)input_image->data,
                               input_image->linesize,
                               desc,
                               0, y, i, w1, 0);
            memcpy(output_image + y*out_stride, f->c_image_line_buf, w1 * sizeof(*f->c_image_line_buf));
        }

        // top/bottom borders
        memset(f->c_image_line_buf, 0, w1 * sizeof(*f->c_image_line_buf));
        av_read_image_line(f->c_image_line_buf,
                           (void *)input_image->data,
                           input_image->linesize,
                           desc,
                           0, 0, i, w1, 0);
        memset(f->p_image_line_buf, 0, w1 * sizeof(*f->p_image_line_buf));
        av_read_image_line(f->p_image_line_buf,
                           (void *)input_image->data,
                           input_image->linesize,
                           desc,
                           0, h1-1, i, w1, 0);

        for (k = 0; k < border; k++) {
            memcpy(output_image-(k+1)*out_stride, f->c_image_line_buf, w1 * sizeof(*f->c_image_line_buf));
            memcpy(output_image+(h1+k)*out_stride, f->p_image_line_buf, w1 * sizeof(*f->p_image_line_buf));
        }

        // left/right borders

        for (k = 0; k < border; k++) {
            for (y = -border; y < h1 + border; y++)
            {
                *(output_image  -k-1+y*out_stride) = output_image[y*out_stride];
                *(output_image+w1+k  +y*out_stride) = output_image[y*out_stride+w1-1];
            }
        }
    }
}

static void free_color_image(ColorImage *img)
{
    int c;
    for (c = 0; c < 3; c++) {
        if (img->plane[c].mem_start) {
            av_freep(img->plane[c].mem_start);
            img->plane[c].mem_start=NULL;
            img->plane[c].img=NULL;
        }
    }
}

static void buildIntegralImage_scalar(uint32_t* integral_image, int integral_stride,
    const uint16_t* current_image, int current_image_stride,
    const uint16_t* compare_image, int compare_image_stride,
    int  w, int  h, int dx, int dy)
{
    memset(integral_image - 1 - integral_stride, 0, (w+1)*sizeof(uint32_t));

    for (int y = 0; y < h; y++) {
        const uint16_t* p1 = current_image +  y    *current_image_stride;
        const uint16_t* p2 = compare_image + (y+dy)*compare_image_stride + dx;
        uint32_t* out = integral_image + y*integral_stride - 1;

        *out++ = 0;

        for (int x = 0; x < w; x++)
        {
            int diff = *p1++ - *p2++;
            *out = *(out - 1) + diff * diff;
            out++;
        }

        if (y > 0) {
            out = integral_image + y*integral_stride;

            for (int x=0;x<w;x++) {
                *out += *(out - integral_stride);
                out++;
            }
        }
    }
}

static void NLMeans_mono_multi(FFV1Context *f, 
    AVFrame* out, int plane_id,
    const MonoImage*const* images, int n_images,
    const NLMContext* ctx)
{
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(out->format);
    int i, image_idx, dy, dx, y, x;
    
    const int w = images[0]->w;
    const int h = images[0]->h;

    const int n = (ctx->patch_size|1);
    const int r = (ctx->range     |1);

    const int n_half = (n-1)/2;
    const int r_half = (r-1)/2;


    // temporary pixel sums

    memset(ctx->tmp_data, 0, w*h*sizeof(*ctx->tmp_data));
    struct PixelSum* const tmp_data = ctx->tmp_data;

    // integral image

    const int integral_stride = w+2*16;
    memset(ctx->integral_mem, 0, integral_stride*(h+1)*sizeof(*ctx->integral_mem));
    uint32_t* const integral_mem = ctx->integral_mem;
    uint32_t* const integral = integral_mem + integral_stride + 16;

    // precompute exponential table

    const float weight_factor = 1.0/n/n / (ctx->h_param * ctx->h_param);

#define EXP_TABSIZE 128

    const int table_size=EXP_TABSIZE;
    const float min_weight_in_table = 0.0005;

    float exptable[EXP_TABSIZE];

    const float stretch = table_size/ (-log(min_weight_in_table));
    const float weight_fact_table = weight_factor*stretch;
    const int diff_max = table_size/weight_fact_table;

    for (i = 0; i < table_size; i++) {
        exptable[i] = exp(-i/stretch);
    }
    exptable[table_size-1]=0;



    for (image_idx = 0; image_idx < n_images; image_idx++)
    {
        // copy input image

        const uint16_t* current_image = images[0]->img;
        int current_image_stride = images[0]->stride;

        const uint16_t* compare_image = images[image_idx]->img;
        int compare_image_stride = images[image_idx]->stride;


        // --- iterate through all displacements ---

        for (dy = -r_half ; dy <= r_half ; dy++)
            for (dx = -r_half ; dx <= r_half ; dx++)
            {
                // special, simple implementation for no shift (no difference -> weight 1)

                if (dx==0 && dy==0 && image_idx==0) {
                    for (y = n_half; y < h - /*n +*/ n_half; y++) {
                        for (x = n_half; x < w - /*n +*/ n_half; x++) {
                            tmp_data[y*w+x].weight_sum += 1;
                            tmp_data[y*w+x].pixel_sum  += current_image[y*current_image_stride+x];
                        }
                    }
                    continue;
                }

                // --- regular case ---

                buildIntegralImage_scalar(integral,integral_stride,
                                        current_image, current_image_stride,
                                        compare_image, compare_image_stride,
                                        w, h, dx, dy);

                for (y = 0; y <= h - n; y++) {
                    const uint32_t* integral_ptr1 = integral+(y  -1)*integral_stride-1;
                    const uint32_t* integral_ptr2 = integral+(y+n-1)*integral_stride-1;

                    for (x = 0; x <= w - n; x++) {
                        const int xc = x+n_half;
                        const int yc = y+n_half;

                        // patch difference

                        int diff = (uint32_t)(integral_ptr2[n] - integral_ptr2[0] - integral_ptr1[n] + integral_ptr1[0]);

                        // sum pixel with weight

                        if (diff < diff_max) {
                            int diffidx = diff*weight_fact_table;

                            //float weight = exp(-diff*weightFact);
                            float weight = exptable[diffidx];

                            tmp_data[yc*w+xc].weight_sum += weight;
                            tmp_data[yc*w+xc].pixel_sum  += weight * compare_image[(yc+dy)*compare_image_stride+xc+dx];
                        }

                        integral_ptr1++;
                        integral_ptr2++;
                    }
                }
            }
    }

    // --- fill output image ---

    // copy border area

    {
        const uint16_t *in  = images[0]->img;
        int orig_in_stride = images[0]->stride;

        for (y = 0; y < n_half; y++) {
            av_write_image_line(in+y*orig_in_stride,
                                out->data,
                                out->linesize,
                                desc,
                                0, y, plane_id, w);
        }
        for (y = h - n_half; y < h; y++) { 
            av_write_image_line(in+y*orig_in_stride,
                                out->data,
                                out->linesize,
                                desc,
                                0, y, plane_id, w);
        }
        for (y = n_half; y < h - n_half; y++) {
            av_write_image_line(in+y*orig_in_stride,
                                out->data,
                                out->linesize,
                                desc,
                                0, y, plane_id, n_half);
            av_write_image_line(in+y*orig_in_stride+w-n_half,
                                out->data,
                                out->linesize,
                                desc,
                                w-n_half, y, plane_id, n_half);
        }
    }

    // output main image

    for (y = n_half; y < h - n_half; y++) {
        memset(f->c_image_line_buf, 0, w * sizeof(*f->c_image_line_buf));
        for (x = n_half; x < w - n_half; x++) {
            //*(out+y*out_stride+x) = tmp_data[y*w+x].pixel_sum / tmp_data[y*w+x].weight_sum;
            //if (tmp_data[y*w+x].weight_sum < 1)
            //    fprintf(stderr, "(%d, %d) -> <%f, %f, %u>\n", x, y, tmp_data[y*w+x].pixel_sum, tmp_data[y*w+x].weight_sum, (uint16_t)(tmp_data[y*w+x].pixel_sum / tmp_data[y*w+x].weight_sum));
            f->c_image_line_buf[x] = tmp_data[y*w+x].pixel_sum / tmp_data[y*w+x].weight_sum;
        }
        av_write_image_line(f->c_image_line_buf + n_half,
                            out->data,
                            out->linesize,
                            desc,
                            n_half, y, plane_id, w - 2*n_half);
    }
}

static void NLMeans_color_auto(FFV1Context *f, 
    AVFrame *out, const ColorImage* img, // function takes ownership
    NLMContext* ctx)
{
    //av_assert(ctx->n_frames >= 1);
    //av_assert(ctx->n_frames <= MAX_NLMeansImages);
    
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(out->format);
    int i, c;
    int has_plane[4] = { 0 };
    
    for (i = 0; i < desc->nb_components; i++)
        has_plane[desc->comp[i].plane] = 1;

    for (i = 0; i < desc->nb_components && has_plane[i]; i++)
        memset(out->buf[i]->data, 0, out->buf[i]->size * sizeof(*out->buf[i]->data));

    // process color planes separately

    for (c = 0; c < desc->nb_components; c++) {
        const MonoImage* images[MAX_NLMeansImages];
        for (i = 0; ctx->image_available[i]; i++) {
            images[i] = &ctx->images[i].plane[c];
        }

        NLMeans_mono_multi(f, out, c, images, i, ctx);
    }
}

void ff_ffv1_filter_frame(FFV1Context *f, AVFrame *frame)
{
    NLMContext *nlm = &f->nlm;
    int i;
    
    ColorImage *curr = &nlm->images[nlm->n_frames-1];

    // extend image with border
    for (i=nlm->n_frames-1; i>0; i--) {
        nlm->images[i] = nlm->images[i-1];
        nlm->image_available[i] = nlm->image_available[i-1];
    }
    
    int border = nlm->range/2;
    copy_image_with_border(f, curr, frame, border);

    nlm->images[0] = *curr;
    nlm->image_available[0] = 1;
    
    AVFrame *out = av_frame_clone(frame);
    av_frame_make_writable(out);
    
    NLMeans_color_auto(f, out, curr, nlm);
    //fprintf(stderr, " vs %p\n", bordered_image.plane[0].mem_start);
    //free_color_image(&bordered_image);
    
    av_frame_copy(frame, out);
    av_frame_free(&out);
}

int ff_ffv1_nlm_init(FFV1Context *f)
{
    NLMContext *nlm = &f->nlm;
    
    const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(f->avctx->pix_fmt);
    int width  = f->avctx->width;
    int height = f->avctx->height;
    int i, c;
    const int cw = AV_CEIL_RSHIFT(width, desc->log2_chroma_w);
    const int ch = AV_CEIL_RSHIFT(height, desc->log2_chroma_h);

    nlm->hsub  = desc->log2_chroma_w;
    nlm->vsub  = desc->log2_chroma_h;
    
    nlm->h_param = 8.0;
    nlm->patch_size = 7;
    nlm->range = 3;
    nlm->n_frames = 2;
    
    nlm->tmp_data = av_mallocz_array(width*height, sizeof(struct PixelSum));
    const int integral_stride = width+2*16;
    nlm->integral_mem = av_malloc_array( integral_stride*(height+1), sizeof(uint32_t) );
    
    const int border = (nlm->range/2+15)/16*16;

    for (i = 0; i < MAX_NLMeansImages; i++) {
        nlm->image_available[i] = 0;
        
        for (c = 0; c < desc->nb_components; c++) {
            const int w1 = (c == 1 || c == 2) ? cw : width;
            const int h1 = (c == 1 || c == 2) ? ch : height;
            
            const int out_stride = (w1+2*border);
            const int out_total_height = (h1+2*border);
            
            nlm->images[i].plane[c].mem_start   = av_mallocz_array(out_stride*out_total_height, sizeof(uint16_t));
            nlm->images[i].plane[c].img = nlm->images[i].plane[c].mem_start + border + border*out_stride; // logical output image origin (0,0)
            nlm->images[i].plane[c].stride = out_stride;
            nlm->images[i].plane[c].w = w1;
            nlm->images[i].plane[c].h = h1;
            nlm->images[i].plane[c].border = border;
        }
    }

    return 0;
}

void ff_ffv1_release_buffer(AVCodecContext *avctx)
{
    FFV1Context *s = avctx->priv_data;
    int i;

    if(s->last_pictures[s->max_ref_frames-1]->data[0]){
        av_frame_unref(s->last_pictures[s->max_ref_frames-1]);
        for(i=0; i<9; i++)
            if(s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3]) {
                av_free(s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3] - EDGE_WIDTH*(1+s->current_picture->linesize[i%3]));
                s->halfpel_plane[s->max_ref_frames-1][1+i/3][i%3] = NULL;
            }
    }
}

int ff_ffv1_get_buffer(FFV1Context *s, AVFrame *frame)
{
    int ret, i;
    int edges_needed = av_codec_is_encoder(s->avctx->codec);

    frame->width  = s->avctx->width ;
    frame->height = s->avctx->height;
    if (edges_needed) {
        frame->width  += 2 * EDGE_WIDTH;
        frame->height += 2 * EDGE_WIDTH;
    }
    if ((ret = ff_get_buffer(s->avctx, frame, AV_GET_BUFFER_FLAG_REF)) < 0)
        return ret;
    if (edges_needed) {
        for (i = 0; frame->data[i]; i++) {
            int offset = (EDGE_WIDTH >> (i ? s->chroma_v_shift : 0)) *
                            frame->linesize[i] +
                            (EDGE_WIDTH >> (i ? s->chroma_h_shift : 0));
            frame->data[i] += offset;
        }
        frame->width  = s->avctx->width;
        frame->height = s->avctx->height;
    }

    return 0;
}

static av_cold void init_qexp(void)
{
    int i;
    double v=128;

    for(i=0; i<QROOT; i++)
    {
        ff_qexp[i]= lrintf(v);
        v *= pow(2, 1.0 / QROOT);
    }
}

int ff_ffv1_common_init_after_header(AVCodecContext *avctx) {
    FFV1Context *s = avctx->priv_data;
    int plane_index;//, level, orientation;
    int ret, emu_buf_size;

    if(!s->scratchbuf) {
        if ((ret = ff_get_buffer(s->avctx, s->mconly_picture,
                                 AV_GET_BUFFER_FLAG_REF)) < 0)
            return ret;
        FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->scratchbuf, FFMAX(s->mconly_picture->linesize[0], 2*avctx->width+256), 7*MB_SIZE, fail);
        emu_buf_size = FFMAX(s->mconly_picture->linesize[0], 2*avctx->width+256) * (2 * MB_SIZE + HTAPS_MAX - 1);
        FF_ALLOC_OR_GOTO(avctx, s->emu_edge_buffer, emu_buf_size, fail);
    }

    if(s->mconly_picture->format != avctx->pix_fmt) {
        av_log(avctx, AV_LOG_ERROR, "pixel format changed\n");
        return AVERROR_INVALIDDATA;
    }

    for(plane_index=0; plane_index < s->nb_planes; plane_index++){
        int w= s->avctx->width;
        int h= s->avctx->height;

        if(plane_index){
            w = AV_CEIL_RSHIFT(w, s->chroma_h_shift);
            h = AV_CEIL_RSHIFT(h, s->chroma_v_shift);
        }
        s->plane[plane_index].width = w;
        s->plane[plane_index].height= h;
    }

    return 0;
fail:
    return AVERROR(ENOMEM);
}

static void mc_block(PlaneContext *p, uint8_t *dst, const uint8_t *src, int stride, int b_w, int b_h, int dx, int dy){
    static const uint8_t weight[64]={
    8,7,6,5,4,3,2,1,
    7,7,0,0,0,0,0,1,
    6,0,6,0,0,0,2,0,
    5,0,0,5,0,3,0,0,
    4,0,0,0,4,0,0,0,
    3,0,0,5,0,3,0,0,
    2,0,6,0,0,0,2,0,
    1,7,0,0,0,0,0,1,
    };

    static const uint8_t brane[256]={
    0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x11,0x12,0x12,0x12,0x12,0x12,0x12,0x12,
    0x04,0x05,0xcc,0xcc,0xcc,0xcc,0xcc,0x41,0x15,0x16,0xcc,0xcc,0xcc,0xcc,0xcc,0x52,
    0x04,0xcc,0x05,0xcc,0xcc,0xcc,0x41,0xcc,0x15,0xcc,0x16,0xcc,0xcc,0xcc,0x52,0xcc,
    0x04,0xcc,0xcc,0x05,0xcc,0x41,0xcc,0xcc,0x15,0xcc,0xcc,0x16,0xcc,0x52,0xcc,0xcc,
    0x04,0xcc,0xcc,0xcc,0x41,0xcc,0xcc,0xcc,0x15,0xcc,0xcc,0xcc,0x16,0xcc,0xcc,0xcc,
    0x04,0xcc,0xcc,0x41,0xcc,0x05,0xcc,0xcc,0x15,0xcc,0xcc,0x52,0xcc,0x16,0xcc,0xcc,
    0x04,0xcc,0x41,0xcc,0xcc,0xcc,0x05,0xcc,0x15,0xcc,0x52,0xcc,0xcc,0xcc,0x16,0xcc,
    0x04,0x41,0xcc,0xcc,0xcc,0xcc,0xcc,0x05,0x15,0x52,0xcc,0xcc,0xcc,0xcc,0xcc,0x16,
    0x44,0x45,0x45,0x45,0x45,0x45,0x45,0x45,0x55,0x56,0x56,0x56,0x56,0x56,0x56,0x56,
    0x48,0x49,0xcc,0xcc,0xcc,0xcc,0xcc,0x85,0x59,0x5A,0xcc,0xcc,0xcc,0xcc,0xcc,0x96,
    0x48,0xcc,0x49,0xcc,0xcc,0xcc,0x85,0xcc,0x59,0xcc,0x5A,0xcc,0xcc,0xcc,0x96,0xcc,
    0x48,0xcc,0xcc,0x49,0xcc,0x85,0xcc,0xcc,0x59,0xcc,0xcc,0x5A,0xcc,0x96,0xcc,0xcc,
    0x48,0xcc,0xcc,0xcc,0x49,0xcc,0xcc,0xcc,0x59,0xcc,0xcc,0xcc,0x96,0xcc,0xcc,0xcc,
    0x48,0xcc,0xcc,0x85,0xcc,0x49,0xcc,0xcc,0x59,0xcc,0xcc,0x96,0xcc,0x5A,0xcc,0xcc,
    0x48,0xcc,0x85,0xcc,0xcc,0xcc,0x49,0xcc,0x59,0xcc,0x96,0xcc,0xcc,0xcc,0x5A,0xcc,
    0x48,0x85,0xcc,0xcc,0xcc,0xcc,0xcc,0x49,0x59,0x96,0xcc,0xcc,0xcc,0xcc,0xcc,0x5A,
    };

    static const uint8_t needs[16]={
    0,1,0,0,
    2,4,2,0,
    0,1,0,0,
    15
    };

    int x, y, b, r, l;
    int16_t tmpIt   [64*(32+HTAPS_MAX)];
    uint8_t tmp2t[3][64*(32+HTAPS_MAX)];
    int16_t *tmpI= tmpIt;
    uint8_t *tmp2= tmp2t[0];
    const uint8_t *hpel[11];
    av_assert2(dx<16 && dy<16);
    r= brane[dx + 16*dy]&15;
    l= brane[dx + 16*dy]>>4;

    b= needs[l] | needs[r];
    if(p && !p->diag_mc)
        b= 15;

    if(b&5){
        for(y=0; y < b_h+HTAPS_MAX-1; y++){
            for(x=0; x < b_w; x++){
                int a_1=src[x + HTAPS_MAX/2-4];
                int a0= src[x + HTAPS_MAX/2-3];
                int a1= src[x + HTAPS_MAX/2-2];
                int a2= src[x + HTAPS_MAX/2-1];
                int a3= src[x + HTAPS_MAX/2+0];
                int a4= src[x + HTAPS_MAX/2+1];
                int a5= src[x + HTAPS_MAX/2+2];
                int a6= src[x + HTAPS_MAX/2+3];
                int am=0;
                if(!p || p->fast_mc){
                    am= 20*(a2+a3) - 5*(a1+a4) + (a0+a5);
                    tmpI[x]= am;
                    am= (am+16)>>5;
                }else{
                    am= p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6);
                    tmpI[x]= am;
                    am= (am+32)>>6;
                }

                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            tmpI+= 64;
            tmp2+= 64;
            src += stride;
        }
        src -= stride*y;
    }
    src += HTAPS_MAX/2 - 1;
    tmp2= tmp2t[1];

    if(b&2){
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w+1; x++){
                int a_1=src[x + (HTAPS_MAX/2-4)*stride];
                int a0= src[x + (HTAPS_MAX/2-3)*stride];
                int a1= src[x + (HTAPS_MAX/2-2)*stride];
                int a2= src[x + (HTAPS_MAX/2-1)*stride];
                int a3= src[x + (HTAPS_MAX/2+0)*stride];
                int a4= src[x + (HTAPS_MAX/2+1)*stride];
                int a5= src[x + (HTAPS_MAX/2+2)*stride];
                int a6= src[x + (HTAPS_MAX/2+3)*stride];
                int am=0;
                if(!p || p->fast_mc)
                    am= (20*(a2+a3) - 5*(a1+a4) + (a0+a5) + 16)>>5;
                else
                    am= (p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6) + 32)>>6;

                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            src += stride;
            tmp2+= 64;
        }
        src -= stride*y;
    }
    src += stride*(HTAPS_MAX/2 - 1);
    tmp2= tmp2t[2];
    tmpI= tmpIt;
    if(b&4){
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                int a_1=tmpI[x + (HTAPS_MAX/2-4)*64];
                int a0= tmpI[x + (HTAPS_MAX/2-3)*64];
                int a1= tmpI[x + (HTAPS_MAX/2-2)*64];
                int a2= tmpI[x + (HTAPS_MAX/2-1)*64];
                int a3= tmpI[x + (HTAPS_MAX/2+0)*64];
                int a4= tmpI[x + (HTAPS_MAX/2+1)*64];
                int a5= tmpI[x + (HTAPS_MAX/2+2)*64];
                int a6= tmpI[x + (HTAPS_MAX/2+3)*64];
                int am=0;
                if(!p || p->fast_mc)
                    am= (20*(a2+a3) - 5*(a1+a4) + (a0+a5) + 512)>>10;
                else
                    am= (p->hcoeff[0]*(a2+a3) + p->hcoeff[1]*(a1+a4) + p->hcoeff[2]*(a0+a5) + p->hcoeff[3]*(a_1+a6) + 2048)>>12;
                if(am&(~255)) am= ~(am>>31);
                tmp2[x]= am;
            }
            tmpI+= 64;
            tmp2+= 64;
        }
    }

    hpel[ 0]= src;
    hpel[ 1]= tmp2t[0] + 64*(HTAPS_MAX/2-1);
    hpel[ 2]= src + 1;

    hpel[ 4]= tmp2t[1];
    hpel[ 5]= tmp2t[2];
    hpel[ 6]= tmp2t[1] + 1;

    hpel[ 8]= src + stride;
    hpel[ 9]= hpel[1] + 64;
    hpel[10]= hpel[8] + 1;

#define MC_STRIDE(x) (needs[x] ? 64 : stride)

    if(b==15){
        int dxy = dx / 8 + dy / 8 * 4;
        const uint8_t *src1 = hpel[dxy    ];
        const uint8_t *src2 = hpel[dxy + 1];
        const uint8_t *src3 = hpel[dxy + 4];
        const uint8_t *src4 = hpel[dxy + 5];
        int stride1 = MC_STRIDE(dxy);
        int stride2 = MC_STRIDE(dxy + 1);
        int stride3 = MC_STRIDE(dxy + 4);
        int stride4 = MC_STRIDE(dxy + 5);
        dx&=7;
        dy&=7;
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                dst[x]= ((8-dx)*(8-dy)*src1[x] + dx*(8-dy)*src2[x]+
                         (8-dx)*   dy *src3[x] + dx*   dy *src4[x]+32)>>6;
            }
            src1+=stride1;
            src2+=stride2;
            src3+=stride3;
            src4+=stride4;
            dst +=stride;
        }
    }else{
        const uint8_t *src1= hpel[l];
        const uint8_t *src2= hpel[r];
        int stride1 = MC_STRIDE(l);
        int stride2 = MC_STRIDE(r);
        int a= weight[((dx&7) + (8*(dy&7)))];
        int b= 8-a;
        for(y=0; y < b_h; y++){
            for(x=0; x < b_w; x++){
                dst[x]= (a*src1[x] + b*src2[x] + 4)>>3;
            }
            src1+=stride1;
            src2+=stride2;
            dst +=stride;
        }
    }
}

#define mca(dx,dy,b_w)\
static void mc_block_hpel ## dx ## dy ## b_w(uint8_t *dst, const uint8_t *src, ptrdiff_t stride, int h){\
    av_assert2(h==b_w);\
    mc_block(NULL, dst, src-(HTAPS_MAX/2-1)-(HTAPS_MAX/2-1)*stride, stride, b_w, b_w, dx, dy);\
}

mca( 0, 0,16)
mca( 8, 0,16)
mca( 0, 8,16)
mca( 8, 8,16)
mca( 0, 0,8)
mca( 8, 0,8)
mca( 0, 8,8)
mca( 8, 8,8)

void ff_ffv1_pred_block(FFV1Context *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride,
                     int sx, int sy, int b_w, int b_h, const BlockNode *block,
                     int plane_index, int w, int h)
{
    if(block->type & BLOCK_INTRA){
        int x, y;
        const unsigned color  = block->color[plane_index];
        const unsigned color4 = color*0x01010101;
        if(b_w==32){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
                *(uint32_t*)&dst[8 + y*stride]= color4;
                *(uint32_t*)&dst[12+ y*stride]= color4;
                *(uint32_t*)&dst[16+ y*stride]= color4;
                *(uint32_t*)&dst[20+ y*stride]= color4;
                *(uint32_t*)&dst[24+ y*stride]= color4;
                *(uint32_t*)&dst[28+ y*stride]= color4;
            }
        }else if(b_w==16){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
                *(uint32_t*)&dst[8 + y*stride]= color4;
                *(uint32_t*)&dst[12+ y*stride]= color4;
            }
        }else if(b_w==8){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
                *(uint32_t*)&dst[4 + y*stride]= color4;
            }
        }else if(b_w==4){
            for(y=0; y < b_h; y++){
                *(uint32_t*)&dst[0 + y*stride]= color4;
            }
        }else{
            for(y=0; y < b_h; y++){
                for(x=0; x < b_w; x++){
                    dst[x + y*stride]= color;
                }
            }
        }
    }else{
        uint8_t *src= s->last_pictures[block->ref]->data[plane_index];
        const int scale= plane_index ?  (2*s->mv_scale)>>s->chroma_h_shift : 2*s->mv_scale;
        int mx= block->mx*scale;
        int my= block->my*scale;
        const int dx= mx&15;
        const int dy= my&15;
        const int tab_index= 3 - (b_w>>2) + (b_w>>4);
        sx += (mx>>4) - (HTAPS_MAX/2-1);
        sy += (my>>4) - (HTAPS_MAX/2-1);
        src += sx + sy*stride;
        if(   (unsigned)sx >= FFMAX(w - b_w - (HTAPS_MAX-2), 0)
           || (unsigned)sy >= FFMAX(h - b_h - (HTAPS_MAX-2), 0)){
            s->vdsp.emulated_edge_mc(tmp + MB_SIZE, src,
                                     stride, stride,
                                     b_w+HTAPS_MAX-1, b_h+HTAPS_MAX-1,
                                     sx, sy, w, h);
            src= tmp + MB_SIZE;
        }

        av_assert2(s->chroma_h_shift == s->chroma_v_shift); // only one mv_scale

        av_assert2((tab_index>=0 && tab_index<4) || b_w==32);
        if(    (dx&3) || (dy&3)
            || !(b_w == b_h || 2*b_w == b_h || b_w == 2*b_h)
            || (b_w&(b_w-1))
            || b_w == 1
            || b_h == 1
            || !s->plane[plane_index].fast_mc )
            mc_block(&s->plane[plane_index], dst, src, stride, b_w, b_h, dx, dy);
        else if(b_w==32){
            int y;
            for(y=0; y<b_h; y+=16){
                s->h264qpel.put_h264_qpel_pixels_tab[0][dy+(dx>>2)](dst + y*stride, src + 3 + (y+3)*stride,stride);
                s->h264qpel.put_h264_qpel_pixels_tab[0][dy+(dx>>2)](dst + 16 + y*stride, src + 19 + (y+3)*stride,stride);
            }
        }else if(b_w==b_h)
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst,src + 3 + 3*stride,stride);
        else if(b_w==2*b_h){
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index+1][dy+(dx>>2)](dst    ,src + 3       + 3*stride,stride);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index+1][dy+(dx>>2)](dst+b_h,src + 3 + b_h + 3*stride,stride);
        }else{
            av_assert2(2*b_w==b_h);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst           ,src + 3 + 3*stride           ,stride);
            s->h264qpel.put_h264_qpel_pixels_tab[tab_index  ][dy+(dx>>2)](dst+b_w*stride,src + 3 + 3*stride+b_w*stride,stride);
        }
    }
}

#define USE_HALFPEL_PLANE 1

static int halfpel_interpol(FFV1Context *s, uint8_t *halfpel[4][4], AVFrame *frame){
    int p,x,y;

    for(p=0; p < s->nb_planes; p++){
        int is_chroma= !!p;
        int w= is_chroma ? AV_CEIL_RSHIFT(s->avctx->width,  s->chroma_h_shift) : s->avctx->width;
        int h= is_chroma ? AV_CEIL_RSHIFT(s->avctx->height, s->chroma_v_shift) : s->avctx->height;
        int ls= frame->linesize[p];
        uint8_t *src= frame->data[p];

        halfpel[1][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        halfpel[2][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        halfpel[3][p] = av_malloc_array(ls, (h + 2 * EDGE_WIDTH));
        if (!halfpel[1][p] || !halfpel[2][p] || !halfpel[3][p]) {
            av_freep(&halfpel[1][p]);
            av_freep(&halfpel[2][p]);
            av_freep(&halfpel[3][p]);
            return AVERROR(ENOMEM);
        }
        halfpel[1][p] += EDGE_WIDTH * (1 + ls);
        halfpel[2][p] += EDGE_WIDTH * (1 + ls);
        halfpel[3][p] += EDGE_WIDTH * (1 + ls);

        halfpel[0][p]= src;
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[1][p][i]= (
                    20*(src[i] + src[i+1<h*w?i+1:i-1]) - 
                    5*(src[i-1<0?i+1:i-1] + src[i+2<h*w?i+2:i-2]) + 
                    (src[i-2<0?i+2:i-2] + src[i+3<h*w?i+3:i-3]) + 16 
                )>>5;
            }
        }
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[2][p][i]= (
                    20*(src[i] + src[i+ls<h*w?i+ls:i-ls]) - 
                    5*(src[i-ls<0?i+ls:i-ls] + src[i+2*ls<h*w?i+2*ls:i-2*ls]) + 
                    (src[i-2*ls<0?i+2*ls:i-2*ls] + src[i+3*ls<h*w?i+3*ls:i-3*ls]) + 16 
                )>>5;
            }
        }
        src= halfpel[1][p];
        for(y=0; y<h; y++){
            for(x=0; x<w; x++){
                int i= y*ls + x;

                halfpel[3][p][i]= (
                    20*(src[i] + src[i+ls<h*w?i+ls:i-ls]) - 
                    5*(src[i-ls<0?i+ls:i-ls] + src[i+2*ls<h*w?i+2*ls:i-2*ls]) + 
                    (src[i-2*ls<0?i+2*ls:i-2*ls] + src[i+3*ls<h*w?i+3*ls:i-3*ls]) + 16 
                )>>5;
            }
        }

//FIXME border!
    }
    return 0;
}

int ff_ffv1_frame_start(FFV1Context *f)
{
    AVFrame *tmp;
    int i, ret;

    ff_ffv1_release_buffer(f->avctx);

    tmp= f->last_pictures[f->max_ref_frames-1];
    for(i=f->max_ref_frames-1; i>0; i--)
        f->last_pictures[i] = f->last_pictures[i-1];
    memmove(f->halfpel_plane+1, f->halfpel_plane, (f->max_ref_frames-1)*sizeof(void*)*4*4);
    if(USE_HALFPEL_PLANE && f->current_picture->data[0]) {
        if((ret = halfpel_interpol(f, f->halfpel_plane[0], f->current_picture)) < 0)
            return ret;
    }
    f->last_pictures[0] = f->current_picture;
    f->current_picture = tmp;
    av_frame_copy_props(f->current_picture, f->last_pictures[0]);

    if (f->key_frame) {
        f->ref_frames= 0;
    } else {
        int i;
        for(i=0; i<f->max_ref_frames && f->last_pictures[i]->data[0]; i++)
            if(i && f->last_pictures[i-1]->key_frame)
                break;
        f->ref_frames= i;
        if(f->ref_frames==0){
            av_log(f->avctx,AV_LOG_ERROR, "No reference frames\n");
            return AVERROR_INVALIDDATA;
        }
    }
    if ((ret = ff_ffv1_get_buffer(f, f->current_picture)) < 0)
        return ret;

    f->current_picture->key_frame= f->key_frame;

    return 0;
}

int ff_ffv1_alloc_blocks(FFV1Context *s){
    int w= AV_CEIL_RSHIFT(s->avctx->width,  LOG2_MB_SIZE);
    int h= AV_CEIL_RSHIFT(s->avctx->height, LOG2_MB_SIZE);

    s->b_width = w;
    s->b_height= h;

    av_free(s->block);
    s->block= av_mallocz_array(w * h,  sizeof(BlockNode) << (s->block_max_depth*2));
    if (!s->block)
        return AVERROR(ENOMEM);

    return 0;
}

void ff_ffv1_reset_contexts(FFV1Context *s)
{
    memset(s->block_state, MID_STATE, sizeof(s->block_state));

    /*f->last_spatial_decomposition_type=
    f->last_qlog=
    f->last_qbias=
    f->last_mv_scale=
    f->last_block_max_depth= 0;
    for(plane_index=0; plane_index<2; plane_index++){
        PlaneContext *p= &f->plane[plane_index];
        f->last_htaps=0;
        f->last_diag_mc=0;
        memset(p->last_hcoeff, 0, sizeof(p->last_hcoeff));
    }*/
}

av_cold int ff_ffv1_common_init(AVCodecContext *avctx)
{
    FFV1Context *s = avctx->priv_data;

    if (!avctx->width || !avctx->height)
        return AVERROR_INVALIDDATA;

    s->avctx = avctx;
    s->flags = avctx->flags;

    int width, height;
    int i, j;

    width = avctx->width;
    height = avctx->height;

    s->max_ref_frames=1;

    ff_me_cmp_init(&s->mecc, avctx);
    ff_hpeldsp_init(&s->hdsp, avctx->flags);
    ff_videodsp_init(&s->vdsp, 8);
    ff_dwt_init(&s->dwt);
    ff_h264qpel_init(&s->h264qpel, 8);

#define mcf(dx,dy)\
    s->qdsp.put_qpel_pixels_tab       [0][dy+dx/4]=\
    s->qdsp.put_no_rnd_qpel_pixels_tab[0][dy+dx/4]=\
        s->h264qpel.put_h264_qpel_pixels_tab[0][dy+dx/4];\
    s->qdsp.put_qpel_pixels_tab       [1][dy+dx/4]=\
    s->qdsp.put_no_rnd_qpel_pixels_tab[1][dy+dx/4]=\
        s->h264qpel.put_h264_qpel_pixels_tab[1][dy+dx/4];

    mcf( 0, 0)
    mcf( 4, 0)
    mcf( 8, 0)
    mcf(12, 0)
    mcf( 0, 4)
    mcf( 4, 4)
    mcf( 8, 4)
    mcf(12, 4)
    mcf( 0, 8)
    mcf( 4, 8)
    mcf( 8, 8)
    mcf(12, 8)
    mcf( 0,12)
    mcf( 4,12)
    mcf( 8,12)
    mcf(12,12)

#define mcfh(dx,dy)\
    s->hdsp.put_pixels_tab       [0][dy/4+dx/8]=\
    s->hdsp.put_no_rnd_pixels_tab[0][dy/4+dx/8]=\
        mc_block_hpel ## dx ## dy ## 16;\
    s->hdsp.put_pixels_tab       [1][dy/4+dx/8]=\
    s->hdsp.put_no_rnd_pixels_tab[1][dy/4+dx/8]=\
        mc_block_hpel ## dx ## dy ## 8;

    mcfh(0, 0)
    mcfh(8, 0)
    mcfh(0, 8)
    mcfh(8, 8)

    init_qexp();

    FF_ALLOCZ_ARRAY_OR_GOTO(avctx, s->spatial_idwt_buffer, width, height * sizeof(IDWTELEM), fail);

    for(i=0; i<MAX_REF_FRAMES; i++) {
        for(j=0; j<MAX_REF_FRAMES; j++)
            ff_scale_mv_ref[i][j] = 256*(i+1)/(j+1);
        s->last_pictures[i] = av_frame_alloc();
        if (!s->last_pictures[i])
            goto fail;
    }

    s->mconly_picture = av_frame_alloc();
    s->current_picture = av_frame_alloc();
    if (!s->mconly_picture || !s->current_picture)
        goto fail;

    /* new end */
    s->picture.f = av_frame_alloc();
    s->last_picture.f = av_frame_alloc();
    s->residual.f = av_frame_alloc();
    if (!s->picture.f || !s->last_picture.f || !s->residual.f)
        goto fail;

    s->width  = avctx->width;
    s->height = avctx->height;

    s->c_image_line_buf = av_mallocz_array(sizeof(*s->c_image_line_buf), 2 * s->width);
    s->p_image_line_buf = av_mallocz_array(sizeof(*s->p_image_line_buf), 2 * s->width);
    if (!s->c_image_line_buf || !s->p_image_line_buf)
        goto fail;

    // defaults
    s->num_h_slices = 1;
    s->num_v_slices = 1;

    return 0;
fail:
    return AVERROR(ENOMEM);
}

av_cold int ff_ffv1_init_slice_state(FFV1Context *f, FFV1Context *fs)
{
    int j, i;

    fs->plane_count  = f->plane_count;
    fs->transparency = f->transparency;
    for (j = 0; j < f->plane_count; j++) {
        PlaneContext *const p = &fs->plane[j];

        if (fs->ac != AC_GOLOMB_RICE) {
            if (!p->state)
                p->state = av_malloc_array(p->context_count, CONTEXT_SIZE *
                                     sizeof(uint8_t));
            if (!p->state)
                return AVERROR(ENOMEM);
        } else {
            if (!p->vlc_state) {
                p->vlc_state = av_mallocz_array(p->context_count, sizeof(VlcState));
                if (!p->vlc_state)
                    return AVERROR(ENOMEM);
                for (i = 0; i < p->context_count; i++) {
                    p->vlc_state[i].error_sum = 4;
                    p->vlc_state[i].count     = 1;
                }
            }
        }
    }

    if (fs->ac == AC_RANGE_CUSTOM_TAB) {
        //FIXME only redo if state_transition changed
        for (j = 1; j < 256; j++) {
            fs->c. one_state[      j] = f->state_transition[j];
            fs->c.zero_state[256 - j] = 256 - fs->c.one_state[j];
        }
    }

    return 0;
}

av_cold int ff_ffv1_init_slices_state(FFV1Context *f)
{
    int i, ret;
    for (i = 0; i < f->max_slice_count; i++) {
        FFV1Context *fs = f->slice_context[i];
        if ((ret = ff_ffv1_init_slice_state(f, fs)) < 0)
            return AVERROR(ENOMEM);
    }
    return 0;
}

av_cold int ff_ffv1_init_slice_contexts(FFV1Context *f)
{
    int i;

    f->max_slice_count = f->num_h_slices * f->num_v_slices;
    av_assert0(f->max_slice_count > 0);

    for (i = 0; i < f->max_slice_count; i++) {
        int sx          = i % f->num_h_slices;
        int sy          = i / f->num_h_slices;
        int sxs         = f->avctx->width  *  sx      / f->num_h_slices;
        int sxe         = f->avctx->width  * (sx + 1) / f->num_h_slices;
        int sys         = f->avctx->height *  sy      / f->num_v_slices;
        int sye         = f->avctx->height * (sy + 1) / f->num_v_slices;
        FFV1Context *fs = av_mallocz(sizeof(*fs));

        if (!fs)
            goto memfail;

        f->slice_context[i] = fs;
        memcpy(fs, f, sizeof(*fs));
        memset(fs->rc_stat2, 0, sizeof(fs->rc_stat2));

        fs->slice_width  = sxe - sxs;
        fs->slice_height = sye - sys;
        fs->slice_x      = sxs;
        fs->slice_y      = sys;

        fs->sample_buffer = av_malloc_array((fs->width + 6), 3 * MAX_PLANES *
                                      sizeof(*fs->sample_buffer));
        if (!fs->sample_buffer) {
            av_freep(&f->slice_context[i]);
            goto memfail;
        }
    }
    return 0;

memfail:
    while(--i >= 0) {
        av_freep(&f->slice_context[i]->sample_buffer);
        av_freep(&f->slice_context[i]);
    }
    return AVERROR(ENOMEM);
}

int ff_ffv1_allocate_initial_states(FFV1Context *f)
{
    int i;

    for (i = 0; i < f->quant_table_count; i++) {
        f->initial_states[i] = av_malloc_array(f->context_count[i],
                                         sizeof(*f->initial_states[i]));
        if (!f->initial_states[i])
            return AVERROR(ENOMEM);
        memset(f->initial_states[i], 128,
               f->context_count[i] * sizeof(*f->initial_states[i]));
    }
    return 0;
}

void ff_ffv1_clear_slice_state(FFV1Context *f, FFV1Context *fs)
{
    int i, j;

    for (i = 0; i < f->plane_count; i++) {
        PlaneContext *p = &fs->plane[i];

        p->interlace_bit_state[0] = 128;
        p->interlace_bit_state[1] = 128;

        if (fs->ac != AC_GOLOMB_RICE) {
            if (f->initial_states[p->quant_table_index]) {
                memcpy(p->state, f->initial_states[p->quant_table_index],
                       CONTEXT_SIZE * p->context_count);
            } else
                memset(p->state, 128, CONTEXT_SIZE * p->context_count);
        } else {
            for (j = 0; j < p->context_count; j++) {
                p->vlc_state[j].drift     = 0;
                p->vlc_state[j].error_sum = 4;    //FFMAX((RANGE + 32)/64, 2);
                p->vlc_state[j].bias      = 0;
                p->vlc_state[j].count     = 1;
            }
        }
    }
}


av_cold int ff_ffv1_close(AVCodecContext *avctx)
{
    FFV1Context *s = avctx->priv_data;
    int i, j;

    if (s->picture.f)
        ff_thread_release_buffer(avctx, &s->picture);
    av_frame_free(&s->picture.f);

    if (s->last_picture.f)
        ff_thread_release_buffer(avctx, &s->last_picture);
    av_frame_free(&s->last_picture.f);

    if (s->residual.f)
        ff_thread_release_buffer(avctx, &s->residual);
    av_frame_free(&s->residual.f);

    for (j = 0; j < s->max_slice_count; j++) {
        FFV1Context *fs = s->slice_context[j];
        for (i = 0; i < s->plane_count; i++) {
            PlaneContext *p = &fs->plane[i];

            av_freep(&p->state);
            av_freep(&p->vlc_state);
        }
        av_freep(&fs->sample_buffer);
    }

    if (s->p_image_line_buf)
        av_freep(&s->p_image_line_buf);
    if (s->c_image_line_buf)
        av_freep(&s->c_image_line_buf);

    av_freep(&avctx->stats_out);
    for (j = 0; j < s->quant_table_count; j++) {
        av_freep(&s->initial_states[j]);
        for (i = 0; i < s->max_slice_count; i++) {
            FFV1Context *sf = s->slice_context[i];
            av_freep(&sf->rc_stat2[j]);
        }
        av_freep(&s->rc_stat2[j]);
    }

    for (i = 0; i < s->max_slice_count; i++)
        av_freep(&s->slice_context[i]);

    av_freep(&s->spatial_idwt_buffer);

    s->m.me.temp= NULL;
    av_freep(&s->m.me.scratchpad);
    av_freep(&s->m.me.map);
    av_freep(&s->m.me.score_map);
    av_freep(&s->m.sc.obmc_scratchpad);

    av_freep(&s->block);
    av_freep(&s->scratchbuf);
    av_freep(&s->emu_edge_buffer);

    for(i=0; i<MAX_REF_FRAMES; i++){
        av_freep(&s->ref_mvs[i]);
        av_freep(&s->ref_scores[i]);
        if(s->last_pictures[i] && s->last_pictures[i]->data[0]) {
            av_assert0(s->last_pictures[i]->data[0] != s->current_picture->data[0]);
        }
        av_frame_free(&s->last_pictures[i]);
    }

    av_frame_free(&s->mconly_picture);
    av_frame_free(&s->current_picture);

    return 0;
}
