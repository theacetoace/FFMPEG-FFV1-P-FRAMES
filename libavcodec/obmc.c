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

#include "obmc.h"
#include "obmc_data.h"

static void mc_block(PlaneObmc *p, uint8_t *dst, const uint8_t *src, int stride, int b_w, int b_h, int dx, int dy){
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

void ff_obmc_pred_block(OBMCContext *s, uint8_t *dst, uint8_t *tmp, ptrdiff_t stride,
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

int ff_obmc_get_buffer(OBMCContext *s, AVFrame *frame)
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

void ff_obmc_release_buffer(OBMCContext *s)
{
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

static int halfpel_interpol(OBMCContext *s, uint8_t *halfpel[4][4], AVFrame *frame)
{
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

int ff_obmc_common_init_after_header(OBMCContext *s) 
{
    int plane_index;//, level, orientation;
    int ret, emu_buf_size;

    if(!s->scratchbuf) {
        if ((ret = ff_get_buffer(s->avctx, s->mconly_picture, AV_GET_BUFFER_FLAG_REF)) < 0)
            return ret;
        FF_ALLOCZ_ARRAY_OR_GOTO(s->avctx, s->scratchbuf, FFMAX(s->mconly_picture->linesize[0], 2*s->avctx->width+256), 7*MB_SIZE, fail);
        emu_buf_size = FFMAX(s->mconly_picture->linesize[0], 2*s->avctx->width+256) * (2 * MB_SIZE + HTAPS_MAX - 1);
        FF_ALLOC_OR_GOTO(s->avctx, s->emu_edge_buffer, emu_buf_size, fail);
    }

    if(s->mconly_picture->format != s->avctx->pix_fmt) {
        av_log(s->avctx, AV_LOG_ERROR, "pixel format changed\n");
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

int ff_obmc_frame_start(OBMCContext *f)
{
    AVFrame *tmp;
    int i, ret;
    
    int USE_HALFPEL_PLANE = 0;
    
    switch(f->avctx->codec_id) {
    case AV_CODEC_ID_FFV1:
        USE_HALFPEL_PLANE = 1;
        break;
    default:
        break;
    }

    ff_obmc_release_buffer(f);

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
    if ((ret = ff_obmc_get_buffer(f, f->current_picture)) < 0)
        return ret;

    f->current_picture->key_frame= f->key_frame;

    return 0;
}

int ff_obmc_alloc_blocks(OBMCContext *s)
{
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

av_cold int ff_obmc_common_init(OBMCContext *s, AVCodecContext *avctx)
{
    s->avctx = avctx;
    
    s->obmc_coder.priv_data = NULL;
    s->obmc_coder.avctx = NULL;
    
    int width, height, i, j;

    width = avctx->width;
    height = avctx->height;
    
    ff_me_cmp_init(&s->mecc, avctx);
    ff_hpeldsp_init(&s->hdsp, avctx->flags);
    ff_videodsp_init(&s->vdsp, 8);
    ff_dwt_init(&s->dwt);
    ff_h264qpel_init(&s->h264qpel, 8);
    
    s->max_ref_frames = 1;

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
fail:
    return AVERROR(ENOMEM);
}

av_cold int ff_obmc_close(OBMCContext *s)
{
    int i;
    
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