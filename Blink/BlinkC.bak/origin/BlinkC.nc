#include "printfZ1.h"
#include "check.h"
//#include "slzw/slzw.h"


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
#ifdef USE_SAES
	uses interface AES;
#endif
}

implementation {
  typedef nx_struct splug_data_msg {
    nx_uint16_t sensor_id;
    nx_uint8_t seq_no;
    nx_uint32_t read_val1;
    nx_uint32_t read_val2;
  } splug_data_msg_t;


	uint32_t counter=0;
	uint16_t explode_size=BLOCK_SIZE;
	uint16_t val2;

	// for AES
#ifdef USE_SAES
	uint8_t exp[240];
	uint8_t K[16] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
				0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
		};
#endif


	event void Compressor.complete(uint16_t compressed_size) {	
		return;
		printfz1("Finished. Compression Ratio %d/%d \n",compressed_size,explode_size);
		call Compressor.summarize();
	}

  event void Boot.booted() {
	printfz1_init();
	call Timer.startPeriodic(1024);
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
#ifdef USE_SAES
	 uint16_t i;
	 uint32_t start_time, end_time;
#endif
    if (len == sizeof(splug_data_msg_t)) {
      splug_data_msg_t *data = (splug_data_msg_t *) payload;
		if (data->read_val2 > 1048576) //overflow
			return bufPtr;

		printfz1("Received: %lu\n", data->read_val2);

		val2 = (ENTRY)data->read_val2;
		memcpy( &write_buffer[counter], &val2, ENTRY_SIZE );
		counter+=ENTRY_SIZE;

		if (counter >= explode_size)
		{
			call Compressor.set(write_buffer, explode_size);
			call Compressor.compress(explode_size);
#ifdef USE_SAES
			counter = 0;
			start_time = call LocalTime.get();
			
			for (i=0;i<COMPRESS_SIZE; i+= 8)
			{
				call AES.encrypt((uint8_t*)(&output_buffer[i]),exp,(uint8_t*)(&write_buffer[i]));
			}

			end_time = call LocalTime.get();
			printfz1("Encryption: %lu ms\n", end_time-start_time);
			for (i=0;i<COMPRESS_SIZE*ENTRY_SIZE; i++)
				printfz1("%02x ",((uint8_t*)write_buffer)[i]);
			printfz1("\n");
#endif
		}
    }

    return bufPtr;
  }


	event void Timer.fired() {
	}


}

