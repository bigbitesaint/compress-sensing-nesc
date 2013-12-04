#include "printfZ1.h"
#include "base_line.h"
#include "soft_aes/AES.h"

module BaseLineC {
	provides interface Compressor;
	uses interface LocalTime<TMilli>;
}

implementation {
	uint16_t psize;
	uint16_t total_time=0;
	uint16_t total_run=0;
	ENTRY *buffer_ptr;


#ifndef	ITERATIVE
	uint16_t g_psize=0;
	uint8_t division = 4;

	task void compress_routine()
	{
		uint16_t i,j;
		uint32_t start_time, end_time;
		uint32_t ut;
		float ft;
		
		printfz1("[%6lu] compression [%u/%u]\n", call LocalTime.get(), g_psize, division-1);
		start_time = call LocalTime.get();
		memset(sparse_buffer, 0, sizeof(sparse_buffer) );
		

		for (i=g_psize; i< psize ; i+= division)
		{
			for (j=0; j<psize; ++j)
			{
				sparse_buffer[i] +=  HWT[ i*psize + j ] *  (float)buffer_ptr[j];
			}
		}


		g_psize ++;

		if (g_psize == division)
		{

			/* position */
			for (i=0; i< psize ; ++i)
				pos_buffer[i] = i;

			/* insertion sort */
			for (i=0; i< psize; ++i)
			{
				for (j=i; j>0; --j)
				{
					/* sort in descending order */
					if ( abs(sparse_buffer[j]) > abs(sparse_buffer[j-1]) )
					{
						/* swap */
						ft = sparse_buffer[j];
						sparse_buffer[j] = sparse_buffer[j-1];
						sparse_buffer[j-1] = ft;

						ut = pos_buffer[j];
						pos_buffer[j] = pos_buffer[j-1];
						pos_buffer[j-1] = ut;
					}
				}
			}
			end_time = call LocalTime.get();
	//		printfz1("Compression: %lu ms\n", end_time-start_time);
			printfz1("[%6lu] end compression\n", call LocalTime.get());
			total_time += end_time - start_time;
			total_run ++;


			/* fill in the output buffer */
			for (i=0, j=0; i< COMPRESS_SIZE;++j)
			{
				output_buffer[i++] = pos_buffer[j];
				output_buffer[i++] = sparse_buffer[j];
	/*
				printfz1("[%lu]=", pos_buffer[j]);
				print_float(sparse_buffer[j]);
				printfz1("\n");
	*/
			}

			signal Compressor.complete(COMPRESS_SIZE);
		}else{
			post compress_routine();
		}

	}

#endif

	command void Compressor.set(uint32_t* plaintext, uint16_t size)
	{
		psize = size;
		g_psize = 0;
		buffer_ptr = plaintext;
	}


	command void Compressor.compress(uint16_t pos)
	{
		printfz1("[%6lu] start compression\n", call LocalTime.get());
#ifdef ITERATIVE
		
		for(i=0;i<psize;i++)
			transform_buffer[i] = (ACC_ENTRY)buffer_ptr[i];

		j=psize;

		while(j>1)
		{
      	j/=2;
			for(i=0;i<j;i++)
			{
         	sparse_buffer[i] = (float)(transform_buffer[2*i] + transform_buffer[2*i+1])/1.414;
				sparse_buffer[i+j] = (float)(transform_buffer[2*i] - transform_buffer[2*i+1])/1.414;
			}

			for(i=0;i<(j<<1);i++)
				transform_buffer[i] = sparse_buffer[i]; 
		}

		/* position */
		for (i=0; i< psize ; ++i)
			pos_buffer[i] = i;

		/* insertion sort */
		for (i=0; i< psize; ++i)
		{
			for (j=i; j>0; --j)
			{
				/* sort in descending order */
				if ( abs(sparse_buffer[j]) > abs(sparse_buffer[j-1]) )
				{
					/* swap */
					ft = sparse_buffer[j];
					sparse_buffer[j] = sparse_buffer[j-1];
					sparse_buffer[j-1] = ft;

					ut = pos_buffer[j];
					pos_buffer[j] = pos_buffer[j-1];
					pos_buffer[j-1] = ut;
				}
			}
		}
		end_time = call LocalTime.get();
//		printfz1("Compression: %lu ms\n", end_time-start_time);
		printfz1("[%6lu] end compression\n", call LocalTime.get());
		total_time += end_time - start_time;
		total_run ++;


		/* fill in the output buffer */
		for (i=0, j=0; i< COMPRESS_SIZE;++j)
		{
			output_buffer[i++] = pos_buffer[j];
			output_buffer[i++] = sparse_buffer[j];
/*
			printfz1("[%lu]=", pos_buffer[j]);
			print_float(sparse_buffer[j]);
			printfz1("\n");
*/
		}

		signal Compressor.complete(COMPRESS_SIZE);

#else
		post compress_routine();
#endif


	}

	command void Compressor.summarize()
	{
//		printfz1("Average Time: %u/%u\n", total_time, total_run);
	}

}
