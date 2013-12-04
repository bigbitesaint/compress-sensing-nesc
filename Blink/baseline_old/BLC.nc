#include "printfZ1.h"
#include "main.h"
#include "reliableradio.h"



#include "base_line.h"
#include "soft_aes/AES.h"



module BLC @safe() {
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
	uses interface AES;
}

implementation {

	uint32_t counter=0;
	uint16_t val2;
	ENTRY trans_buffer[COMPRESS_SIZE];
	uint32_t packet_start = 0;
	uint32_t packet_acc_trans = 0;
	uint32_t packet_n = 0;
	uint8_t state = 0;
	uint8_t ready_buffer;

	// for AES
	uint8_t exp[240];
	uint8_t K[16] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		};



	// len is the number of ENTRY
	void enc(ACC_ENTRY* src, uint16_t len)
	{
		uint16_t i;
		uint32_t start_time, end_time;
		start_time = call LocalTime.get();
		for (i=0;i < len; i+= (16/sizeof(ACC_ENTRY)))
			call AES.encrypt((uint8_t*)(&src[i]),exp,(uint8_t*)(&src[i]));
		end_time = call LocalTime.get();
		printfz1("Encrypt %4d bytes for : %lu ms\n", len*sizeof(ACC_ENTRY) ,end_time-start_time);
	}




	event void Compressor.complete(uint16_t compressed_size) {
		uint16_t i;
		ACC_ENTRY* tmp_ptr = (ACC_ENTRY*) trans_buffer;

	//	enc( output_buffer, COMPRESS_SIZE);

		
		for (i=0; i<compressed_size; ++i)
			tmp_ptr[i] = output_buffer[i];
		//memcpy( trans_buffer, output_buffer, compressed_size*sizeof(ACC_ENTRY) );


		if ( call ReliableTx.isSending() )
			printfz1("Radio in use\n");
    	call ReliableTx.cancel();
		call ReliableTx.send((uint16_t*) &trans_buffer[0], COMPRESS_SIZE * RATIO);
		packet_start = call LocalTime.get();
		printfz1("Transmission done.\n");
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
		uint16_t power;
		uint16_t i;
		uint16_t power_data[128] = {
24,26,23,32,17,24,25,25,24,23,24,24,23,24,24,24,24,24,24,26,21,25,24,24,24,24,24,25,23,25,24,24,23,23,25,27,20,24,24,24,25,24,23,25,23,24,23,24,25,24,24,24,24,24,24,25,24,24,24,25,23,25,24,24,24,25,24,29,20,24,24,24,24,25,23,25,24,24,24,24,25,23,25,23,24,24,24,24,24,24,24,24,24,24,24,24,23,26,23,24,24,24,23,24,24,24,23,24,23,24,23,24,24,23,24,30,16,24,23,23,23,24,23,24,22,24,24,23
};
		if ( state == 1 )
			return;
		power = call SPlugControl.read(RAENERGY);
//		power = rand()%10 + 22;
//		power = power_data[counter%128];

		write_buffer[clean_buffer][counter] = power;
		counter++;
		if (counter >= BLOCK_SIZE)
		{
			call Timer.stop();
			counter = 0;
//			ready_buffer = clean_buffer;
//			clean_buffer = !clean_buffer;

			// dirty 
			write_buffer[ready_buffer][0] = write_buffer[ready_buffer][1] ;
			
			for (i=0; i< BLOCK_SIZE; ++i)
			{
				raw_buffer[i] = write_buffer[ready_buffer][i];
				printfz1("%lu,",raw_buffer[i]);
			}
			printfz1("\n");

			if ( call ReliableTx.isSending() )
				printfz1("Radio in use\n");

			call ReliableTx.cancel();
			call ReliableTx.send((uint16_t*) raw_buffer, BLOCK_SIZE*RATIO);
		
//			memcpy(raw_buffer, write_buffer[ready_buffer], sizeof(ENTRY)*BLOCK_SIZE);
	

		}



  }


	event void ReliableTx.sendDone(uint16_t dropnum) {
		// waiting for raw buffer
		if (state == 0)
		{
			state = 1;
			call Compressor.set(write_buffer[clean_buffer], BLOCK_SIZE);
			call Compressor.compress(BLOCK_SIZE);
		}else{ // waiting for compressed signal
			packet_acc_trans += call LocalTime.get() - packet_start;
			printfz1("Retry: %u\n", dropnum);
			printfz1("Transmission: %lu/%lu\n", packet_acc_trans,++packet_n);
			packet_start = call LocalTime.get();
			state = 0;
			call Timer.startPeriodic(SAMPLE_RATE);
		}


	}

}

