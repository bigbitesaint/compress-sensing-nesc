#ifndef COMPRESS_SENSING_H
#define COMPRESS_SENSING_H
/* so there's totally ENTRY_SIZE*BLOCK_SIZE bytes reserved */
#define ENTRY_SIZE 4
#define ENTRY uint32_t
#define RATIO 2


#ifdef USE_GMATRIX
#define ACC_ENTRY float
#endif


#ifdef USE_BMATRIX
#define ACC_ENTRY uint32_t
#endif


#ifdef USE_SMATRIX
#define ACC_ENTRY uint32_t
#endif

#define BLOCK_SIZE 512
#define COMPRESS_SIZE (BLOCK_SIZE/4)
#define SAMPLE_RATE 16

uint8_t clean_buffer = 0;
ACC_ENTRY output_buffer[2][COMPRESS_SIZE];


#ifdef USE_SMATRIX
uint16_t rand_array[COMPRESS_SIZE];
#define SPARSITY 4
#endif


#define print_float(x) printfz1("%d.%d\n",(int)(x), ((int)((x)*100.0))%100)


#endif
