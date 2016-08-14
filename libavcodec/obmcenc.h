#ifndef AVCODEC_OBMCENC_H
#define AVCODEC_OBMCENC_H

#include "obmc.h"

#define FF_ME_ITER 50

int obmc_encode_init(OBMCContext *s, AVCodecContext *avctx);
int obmc_pre_encode_frame(OBMCContext *f, AVCodecContext *avctx, const AVFrame *pict);
void obmc_encode_blocks(OBMCContext *s, int search);

#endif /* AVCODEC_OBMCENC_H */