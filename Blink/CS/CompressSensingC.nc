#include <math.h>
#include "printfZ1.h"
#include "compress_sensing.h"


#ifdef USE_GMATRIX
//#include "ziggurat/ziggurat.h"
#include "box_muller/box_muller.h"
#endif

module CompressSensingC {
	provides interface Compressor;
	uses interface LocalTime<TMilli>;
}

implementation {
	uint16_t psize;
	uint32_t total_time=0;
	uint16_t total_run=0;
	ACC_ENTRY* buffer_ptr;
	uint32_t seed;


#ifdef USE_GMATRIX
	// for gaussian
   float fn[128];
   int kn[128];
   float wn[128];
	uint8_t csReady=0;
#endif



	// explicitly import the random number 
	uint32_t rand()
	{
		uint32_t mlcg,p,q;
   	uint64_t tmpseed;
   	atomic
   	{
			tmpseed =  (uint64_t)33614U * (uint64_t)seed;
			q = tmpseed; 	/* low */
			q = q >> 1;
			p = tmpseed >> 32 ;		/* hi */
			mlcg = p + q;
       	if (mlcg & 0x80000000) { 
				mlcg = mlcg & 0x7FFFFFFF;
				mlcg++;
			}
			seed = mlcg;
    	}
   	return mlcg; 
	}


	command void Compressor.set(ENTRY* plaintext, uint16_t size)
	{
		uint16_t i;
		// buffer swap
		buffer_ptr = output_buffer[clean_buffer];
		clean_buffer = !clean_buffer;

		memset(buffer_ptr, 0, COMPRESS_SIZE * sizeof(ACC_ENTRY));
		psize = size;
		// set the seed
		seed = 100;

#ifdef USE_SMATRIX
		// for binary
		for (i=0;i<COMPRESS_SIZE; ++i)
			rand_array[i] = i;
#endif

#ifdef USE_GMATRIX
		if (!csReady)
		{
			// for ziggurate
//   		r4_nor_setup ( kn, fn, wn );

			// for box-muller
			build_tlp();
			csReady = 1;
		}
#endif


	}

	command void Compressor.compress(uint16_t pos)
	{
		uint16_t i,j;
		uint32_t r;
		uint32_t start_time, end_time;
		float t;
#ifdef USE_GMATRIX
	#if 0
		uint32_t local_seed = 12345678;
		uint16_t rv1, rv2;
		float value;
	#else
		float value[2];
	#endif
#endif

		start_time = call LocalTime.get();


#ifdef USE_GMATRIX
		r = -1;
		t = 1.0f / sqrtf(COMPRESS_SIZE);
		for (j=0; j<BLOCK_SIZE; ++j)
		{
		for (i=0; i<COMPRESS_SIZE; ++i)
		{
	#if 0
			// this value is from mean=0, dev=1;
			value = r4_nor ( &local_seed, kn, fn, wn , rv1, rv2);			
			buffer_ptr[i] += (float)pos * value;
	#else
			if (r == -1)
			{
				value[0] = (float)((uint16_t)rand())/65535.0f;
				value[1] = (float)((uint16_t)rand())/65535.0f;
				norm(0, t, &value[0], &value[1]);
				r = 1;
			}
			buffer_ptr[i] += (float)pos * value[r] * t;
			r -= 1;
	#endif
		}
		}
#endif


#ifdef USE_BMATRIX
//		t = 1 / sqrtf(COMPRESS_SIZE);
		for (j=0; j<BLOCK_SIZE; ++j)
		{
		for (i=0; i<COMPRESS_SIZE; ++i)
		{
			r = rand();
			if (r & 0x0001)
				buffer_ptr[i] += pos;
			else
				buffer_ptr[i] -= pos;
		}
		}
#endif

#ifdef USE_SMATRIX
		for (i=0; i<SPARSITY; ++i)
		{
			r = rand();
//			printfz1("%2lu ", r);

			// permute 
			r = i+ (r % (COMPRESS_SIZE-i));

			// swap 
			t = rand_array[i];
			rand_array[i] = rand_array[r];
			rand_array[r] = t;

			// sum 
			buffer_ptr[ rand_array[i] ] += pos;
/*			if ( (total_run%BLOCK_SIZE) < 50)
				printfz1("%d: %2d\n", total_run, rand_array[i]);*/
		}
#endif

		end_time = call LocalTime.get();
		total_time += end_time - start_time;
		total_run ++;


		if (total_run%BLOCK_SIZE == 0)
		{
			call Compressor.summarize();
			signal Compressor.complete(COMPRESS_SIZE);
		}

	}

	command void Compressor.summarize()
	{
		printfz1("\nAverage Time: %lu/%u\n", total_time, total_run);
	}

}
