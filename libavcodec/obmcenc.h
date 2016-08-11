#ifndef AVCODEC_OBMCENC_H
#define AVCODEC_OBMCENC_H

#include "obmc.h"

#define FF_ME_ITER 50

int obmc_encode_init(OBMCContext *s, AVCodecContext *avctx);
void obmc_encode_frame(OBMCContext *f, AVCodecContext *avctx, const AVFrame *pict);

#endif /* AVCODEC_OBMCENC_H */