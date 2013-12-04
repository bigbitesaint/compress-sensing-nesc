#ifndef _LOSSLESS_H_
#define _LOSSLESS_H_
#define ENTRY_SIZE 4
#define ENTRY uint32_t
#define RATIO 2

#define BLOCK_SIZE 256

#define COMPRESS_SIZE BLOCK_SIZE
#define SAMPLE_RATE 16


// double buffer
ENTRY double_buffer[2][BLOCK_SIZE];


#endif
