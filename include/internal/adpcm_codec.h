
#ifndef _ADPCM_CODEC_H
#define _ADPCM_CODEC_H

#include "brcm_fw_types.h"

#define MODIFIED_EMBEDDED_ENCODE_C 1
#define READ_FROM_BUFFER (1 && MODIFIED_EMBEDDED_ENCODE_C)
#define TEST_PROFILE 0

typedef struct CodecState_t
{
	int valprev;
	int index;
}CodecState;


void encode(CodecState* state, INT16* input, int numSamples, UINT8* output);
#if !MODIFIED_EMBEDDED_ENCODE_C
void decode(CodecState* state, UINT8* input, int numSamples, INT16* output);

void initDecode68000();
void decode68000(CodecState* state, UINT8* input, int numSamples, INT16* output);
#endif //#if !MODIFIED_EMBEDDED_ENCODE_C


#endif
