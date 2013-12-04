#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "slzw/slzw.h"
#define MAX_SIGNAL 2048

unsigned int corrupted = 0;

void fread_error(unsigned int pos)
{
	printf("Corrupted packet(s): %u\n",corrupted);
	fprintf(stderr, "Process file error at %u\n", pos);
	exit(EXIT_FAILURE);
}

// for AES
unsigned char expan[240];
unsigned char K[16] = {
		  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};



int main(int argc, char *argv[])
{
	FILE *fin, *fout;
	int buffer[1024]={0};
	unsigned long reconstruct_sig[1024];
	int ts_int[2];


	// rebuild structure
	union{
		unsigned long l;
		unsigned short us[2];
		unsigned char uc[4];
	} val;

	if (argc != 2)
	{
		fprintf(stderr, "Usage: [./rebuild] filename");
		return EXIT_FAILURE;
	}


	fin = fopen(argv[1], "r");
	if (!fin)
	{
		fprintf(stderr, "Cannot open file: %s\n", argv[1]);
		return EXIT_FAILURE;
	}

	fout = fopen("lossless_result.csv","w");
	if (!fout)
	{
		fprintf(stderr, "Cannot open file: %s\n", "lossless_result.csv");
		return EXIT_FAILURE;
	}

	// setup aes
	keyExpansion(expan,K);

	int i, j, compressed_length = 0, pad_compressed_length=0 ,decompressed_length = 0;
	char ch;

	while (!feof(fin))
	{
start:
		// get the timestamp 
		if ( fscanf(fin, "%d", &ts_int[0]) != 1)
			fread_error(ftell(fin));

		// handle the case where we have more than one lost packets
		if ( ts_int[0] == -1)
			continue;

		fscanf(fin, "%c", &ch);
		if ( fscanf(fin, "%d", &ts_int[1]) != 1)
			fread_error(ftell(fin));
		fscanf(fin, "%c", &ch);

	
		fscanf(fin, "%hu", &val.us[0]);
		fscanf(fin, "%c", &ch);
		fscanf(fin, "%hu", &val.us[1]);
		fscanf(fin, "%c", &ch);

		compressed_length = val.l;
		pad_compressed_length = (compressed_length/16 + (compressed_length%16 != 0) )*16;

#ifdef DEBUG
		printf("==============================================\n");
		printf("Timestamp: %d, %d  Compressed length: %lu bytes\n", ts_int[0], ts_int[1], val.l);
#endif
		if (val.l > MAX_SIGNAL)
		{
			corrupted ++;
			while (ch=fgetc(fin) != '\n');
			continue;
		}


		for (i=0; i< pad_compressed_length/2 && !feof(fin); ++i)
		{
			if ( !fscanf(fin, "%d", &buffer[i]) )
				fread_error(ftell(fin));
			//printf("buffer[%d] = %u\n",i,buffer[i]);
			// take away dot and newline
			fscanf(fin, "%c", &ch);

			if (buffer[i] == -1)
				goto start;
		}
	

		memset(reconstruct_sig, 0, sizeof(reconstruct_sig));

		for (i=0, j=0; i<pad_compressed_length/2; i+=2)
		{
			val.us[0] = buffer[i];
			val.us[1] = buffer[i+1];
			reconstruct_sig[ j++ ] = val.l;
		}

		// do byte-level aes decryption
		for (i=0; i<pad_compressed_length; i+=16)
		{
			decrypt(&(((unsigned char*)reconstruct_sig)[i]), expan, &(((unsigned char*)reconstruct_sig)[i]));
		//	printf("L%d: ",i);
		//	for (j=0; j<16; ++j)
		//		printf("%u ", ((unsigned char*)reconstruct_sig)[i + j] );
		//	printf("\n");
		}

		memcpy(lzw_output_file_buffer, reconstruct_sig, pad_compressed_length);

#ifdef DEBUG
		printf("start decompressing of %u...\n", compressed_length);
#endif

		decompressed_length = slzw_decompress(compressed_length);



#ifdef DEBUG
		printf("decompress len: %d\n", decompressed_length);
#endif

		if (decompressed_length != 1024)
		{
#ifdef DEBUG
		printf("Go back !\n");
#endif			
			goto start;
		}

		for (i=0, j=0;i < decompressed_length; i+=4)
		{
			val.uc[0] = write_buffer[i+0];
			val.uc[1] = write_buffer[i+1];
			val.uc[2] = write_buffer[i+2];
			val.uc[3] = write_buffer[i+3];
			reconstruct_sig[ j++ ] = val.l;
		}


		decompressed_length = j;


		fprintf(fout,"%d,%d",ts_int[0], ts_int[1]);
		for (i=0;i < decompressed_length; ++i)
			fprintf(fout,",%lu",reconstruct_sig[i]);
		fprintf(fout,"\n");

		// read until end of line
		//while (ch=fgetc(fin), ch != '\n' && ch != EOF );	
	}
	fclose(fin);
	fclose(fout);
	return 0;
}
