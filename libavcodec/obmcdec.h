#ifndef AVCODEC_OBMCENC_H
#define AVCODEC_OBMCENC_H

#include "obmc.h"

int obmc_decode_init(OBMCContext *f);
void obmc_decode_frame(OBMCContext *f, AVCodecContext *avctx);

#endif /* AVCODEC_OBMCENC_H */