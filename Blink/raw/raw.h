#ifndef RAW_H
#define RAW_H



#define ENTRY_SIZE 4
#define ENTRY uint32_t

#define BLOCK_SIZE 1024
#define COMPRESS_SIZE BLOCK_SIZE
#define SAMPLE_RATE 1

// the ratio between our buffer and send() interface
#define RATIO 2

#ifdef CIRCULAR_BUFFER
uint32_t read_ptr=0;
uint32_t write_ptr=0;
ENTRY write_buffer[BLOCK_SIZE];
#else
uint8_t clean_buffer = 0;
ENTRY write_buffer[2][BLOCK_SIZE];
#endif

ENTRY* output_buffer = NULL;

#endif
