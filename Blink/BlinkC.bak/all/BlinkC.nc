#include "printfZ1.h"
#include "check.h"
#include "main.h"
//#include "slzw/slzw.h"


#ifdef USE_RAW
#include "raw.h"
#include "reliable/reliableradio.h"
#endif


#ifdef USE_BASELINE
#include "base_line.h"
#endif


#ifdef USE_CS
#include "compress_sensing.h"
#endif

#ifdef USE_SAES
#include "soft_aes/AES.h"
#endif



module BlinkC @safe() {
	uses interface Leds;
	uses interface Boot;
	uses interface Random;
	uses interface Compressor;
	uses interface Receive;
	uses interface SplitControl as RadioControl;
	uses interface Timer<TMilli> as Timer;
	uses interface LocalTime<TMilli>;
	uses interface SPlugControl;
	uses interface ReliableTx;
#ifdef USE_SAES
	uses interface AES;
#endif
}

implementation {

	uint32_t counter=0;
	uint16_t val2;
	ENTRY trans_buffer[COMPRESS_SIZE];

	// for AES
#ifdef USE_SAES
	uint8_t exp[240];
	uint8_t K[16] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		};
#endif



	event void Compressor.complete(uint16_t compressed_size) {
		uint16_t i;


		// type transform
		// the typecasting is only needed in CS, but for consistency we leave it here even for baseline
		for (i=0; i< compressed_size; ++i)
		{
			trans_buffer[i] = (ENTRY) output_buffer[i];
			printfz1("%d ",trans_buffer[i]);
		}
		printfz1("\n");


	

		if ( call ReliableTx.isSending() )
			printfz1("Radio in use\n");

    	call ReliableTx.cancel();
		call ReliableTx.send(&trans_buffer[0], COMPRESS_SIZE);
		printfz1("Transmission done.\n");
	}

	event void Boot.booted() {
		printfz1_init();

	 	call SPlugControl.init();
		call SPlugControl.setState(SPLUG_ON_STATE);


		call ReliableTx.init();    


		call Timer.startPeriodic(16);
   	call RadioControl.start();
	#ifdef USE_SAES
		call AES.keyExpansion(exp,K);
	#endif
  }

  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS)
      call RadioControl.start();
  }

  event void RadioControl.stopDone(error_t err) {}

	event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
		return bufPtr;
	}

	event void Timer.fired() {
		uint16_t power;
		uint8_t ready_buffer;
		uint16_t i,step;
		uint32_t start_time, end_time;

		power = counter;
//		power = call SPlugControl.read(RAENERGY);
#ifdef USE_RAW
	#ifdef CIRCULAR_BUFFER
		write_buffer[ write_ptr%SAMPLE_NUM ] = power;
		write_ptr ++;
		counter ++;

		// a new packet is ready
		if (counter >= SAMPLE_NUM)
		{
			counter = 0;	

			// if the radio is not in use
			if ( ! call ReliableTx.isSending() )
			{
				printfz1("[SENDING] r: %5lu w:%5lu\n", read_ptr, write_ptr);
				call ReliableTx.send( &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM);
				read_ptr += SAMPLE_NUM;
			}
			else
			{
				// if it's in use and write_ptr > read_ptr + SAMPLE_NUM, force next block read
				if ( write_ptr >= read_ptr + SAMPLE_NUM )
				{
					printfz1("[DISCARD] r: %5lu w:%5lu\n", read_ptr, write_ptr);
					call ReliableTx.cancel();
					read_ptr += SAMPLE_NUM;
					call ReliableTx.send( &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM);
					printfz1("[SENDING] r: %5lu w:%5lu\n", read_ptr, write_ptr);
				}
			}
		}
	#else
		write_buffer[clean_buffer][counter] = power;
		counter++;
		if (counter >= BLOCK_SIZE)
		{
			counter = 0;
			ready_buffer = clean_buffer;
			clean_buffer = !clean_buffer;

			for (i=0; i< COMPRESS_SIZE; ++i)
				trans_buffer[i] = write_buffer[ready_buffer][i];

			if ( call ReliableTx.isSending() )
	    		call ReliableTx.cancel();

			call ReliableTx.send(&trans_buffer[0], BLOCK_SIZE);
		}
	#endif
#endif

#ifdef USE_BASELINE
		write_buffer[clean_buffer][counter] = power;
		counter++;
		if (counter >= BLOCK_SIZE)
		{
			counter = 0;
			ready_buffer = clean_buffer;
			clean_buffer = !clean_buffer;
			call Compressor.set(write_buffer[ready_buffer], BLOCK_SIZE);
			call Compressor.compress(BLOCK_SIZE);
			start_time = call LocalTime.get();
			step = 16/sizeof(ENTRY);
			
			for (i=0;i<COMPRESS_SIZE; i+= step)
				call AES.encrypt((uint8_t*)(&output_buffer[i]),exp,(uint8_t*)(&output_buffer[i]));

			end_time = call LocalTime.get();
			printfz1("Encryption: %lu ms\n", end_time-start_time);
			for (i=0;i<COMPRESS_SIZE*ENTRY_SIZE; i++)
				printfz1("%02x ",((uint8_t*)&output_buffer)[i]);
			printfz1("\n");

		}
#endif

#ifdef USE_CS
		if (counter == 0)
			call Compressor.set(NULL, BLOCK_SIZE);
		call Compressor.compress(power);
		counter++;
		if (counter >= BLOCK_SIZE)
			counter = 0;
#endif


  }


	event void ReliableTx.sendDone(uint16_t dropnum) {
#ifdef USE_RAW
#ifdef CIRCULAR_BUFFER
		printfz1("[SUCCESS] r: %5lu w:%5lu\n", read_ptr, write_ptr);
		// if successfully delivered and data is ready, start next iteration
		if ( write_ptr - SAMPLE_NUM >= read_ptr )
			call ReliableTx.send( &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM);
#endif
#endif
	}

}

