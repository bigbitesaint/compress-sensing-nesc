#include "slzw/slzw.h"

module SlzwC {
	provides interface Compressor;
	uses interface LocalTime<TMilli>;
	uses interface Random;
}

implementation {
	uint16_t psize;
	static uint16_t total_time=0;
	static uint16_t total_run=0;
	command void Compressor.compress(uint16_t len)
	{
		uint32_t start_time, end_time;
		uint16_t i;
		uint16_t compressed_size;

		printfz1("Before compression...\n");
		for (i=0; i< BLOCK_SIZE * 4; ++i)
			printfz1("%u ",((uint8_t*)(write_buffer))[i]);
		printfz1("\n");



		start_time = call LocalTime.get();
		compressed_size = slzw_compress(psize, 32);
		end_time = call LocalTime.get();

		total_time += end_time - start_time;
		total_run ++;

		signal Compressor.complete(compressed_size);
	}

	command void Compressor.set(uint32_t* plaintext, uint16_t size)
	{
		psize = size*sizeof(uint32_t);
		memcpy(write_buffer, plaintext, psize);
	}

	command void Compressor.summarize()
	{
		printfz1("Average Time: %u/%u\n", total_time, total_run);
	}
}
