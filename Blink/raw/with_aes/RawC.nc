#include "printfZ1.h"
#include "main.h"


#include "raw.h"
#include "reliable/reliableradio.h"


#include "soft_aes/AES.h"



module RawC @safe() {
	uses interface Leds;
	uses interface Boot;
	uses interface Random;
	uses interface Receive;
	uses interface SplitControl as RadioControl;
	uses interface Timer<TMilli> as Timer;
	uses interface LocalTime<TMilli>;
	uses interface SPlugControl;
	uses interface ReliableTx;
	uses interface AES;
}

implementation {

	uint32_t counter=0;
	uint16_t val2;
	ENTRY trans_buffer[COMPRESS_SIZE];
	uint32_t packet_start=0, packet_n=0, packet_sum = 0;

	// for AES
	uint8_t exp[240];
	uint8_t K[16] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		};

	// len is the number of ENTRY
	void enc(ENTRY* src, uint16_t len)
	{
		uint16_t i;
		uint32_t start_time, end_time;
		start_time = call LocalTime.get();
		for (i=0;i < len; i+= (16/sizeof(ENTRY)))
			call AES.encrypt((uint8_t*)(&src[i]),exp,(uint8_t*)(&src[i]));
		end_time = call LocalTime.get();
		printfz1("Encrypt %4d bytes for : %lu ms\n", len*sizeof(ENTRY) ,end_time-start_time);
	}


	event void Boot.booted() {
		printfz1_init();

	 	call SPlugControl.init();
		call SPlugControl.setState(SPLUG_ON_STATE);


		call ReliableTx.init();    


		call Timer.startPeriodic(SAMPLE_RATE);
   	call RadioControl.start();
		call AES.keyExpansion(exp,K);
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
		uint16_t power,i;
		uint8_t ready_buffer;

		power = 100;
	//	power = call SPlugControl.read(RAENERGY);
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
				enc(&write_buffer[read_ptr % SAMPLE_NUM], SAMPLE_NUM*RATIO);
				call ReliableTx.send((uint16_t*)  &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM*RATIO);
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
					enc(&write_buffer[read_ptr % SAMPLE_NUM], SAMPLE_NUM*RATIO);
					call ReliableTx.send( (uint16_t*) &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM*RATIO);
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

			
//			for (i=0;i<COMPRESS_SIZE; i+= (16/sizeof(ENTRY)))
//				call AES.encrypt((uint8_t*)(&write_buffer[ready_buffer][i]),exp,(uint8_t*)(&write_buffer[ready_buffer][i]));
			enc(write_buffer[ready_buffer], COMPRESS_SIZE);


/*			for (i=0;i<COMPRESS_SIZE*ENTRY_SIZE; i++)
				printfz1("%02x ",((uint8_t*)&write_buffer[ready_buffer])[i]);
			printfz1("\n");*/



			for (i=0; i< COMPRESS_SIZE; ++i)
				trans_buffer[i] = write_buffer[ready_buffer][i];

			if ( call ReliableTx.isSending() )
	    		call ReliableTx.cancel();

			call ReliableTx.send((uint16_t*) &trans_buffer[0], BLOCK_SIZE*RATIO);
			packet_start=call LocalTime.get();
		}
	#endif

  }


	event void ReliableTx.sendDone(uint16_t dropnum) {
		packet_sum += (call LocalTime.get() - packet_start);
		printfz1("Transmission: %lu/%lu \n", packet_sum, (++packet_n)*(BLOCK_SIZE*RATIO/SAMPLE_NUM));
#ifdef CIRCULAR_BUFFER
		printfz1("[SUCCESS] r: %5lu w:%5lu\n", read_ptr, write_ptr);
		// if successfully delivered and data is ready, start next iteration
		if ( write_ptr - SAMPLE_NUM >= read_ptr )
		{
			enc(&write_buffer[read_ptr % SAMPLE_NUM], SAMPLE_NUM*RATIO);
			call ReliableTx.send( (uint16_t*) &write_buffer[read_ptr % SAMPLE_NUM] , SAMPLE_NUM*RATIO);
		}
#endif
	}

}

