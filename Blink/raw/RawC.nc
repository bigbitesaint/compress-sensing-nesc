#include "printfZ1.h"
#include "main.h"
#include "raw.h"
#include "reliableradio.h"
#include "calib.h"
#define PTR_BLK(x) ((x%BLOCK_SIZE)/SAMPLE_NUM)


module RawC @safe() {
  uses interface Leds;
  uses interface Boot;
  uses interface Random;
  uses interface SplitControl as RadioControl;
  uses interface Timer<TMilli> as Timer;
  uses interface Alarm<TMilli, uint16_t>;
  uses interface LocalTime<TMilli>;
  uses interface SPlugControl;
  uses interface ReliableTx;
}

implementation {

	uint32_t counter=0;
	uint16_t val2;
	uint32_t start[BLOCK_SIZE/SAMPLE_NUM+1]={0}, stop[BLOCK_SIZE/SAMPLE_NUM+1]={0};
	uint8_t miss_count = 0;
    uint32_t ds_count;

  event void Boot.booted() {
    printfz1_init();

    call SPlugControl.init();
    call SPlugControl.setState(SPLUG_ON_STATE);

    call ReliableTx.init();    

	// init buffer
	memset(write_buffer, 0, sizeof(write_buffer));

    call Alarm.start(SAMPLE_RATE);
   	call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS)
      call RadioControl.start();
  }

  event void RadioControl.stopDone(error_t err) {}

  event void Timer.fired() {}


  async event void Alarm.fired() {
    uint32_t power;

    call Alarm.start(SAMPLE_RATE);

    power = 0;//convert(call SPlugControl.read(RAENERGY), TOS_NODE_ID, SAMPLE_RATE);

#ifdef DOWN_SAMPLING
    if (++ds_count % 4 != 0)
      return;
#endif

	write_buffer[ write_ptr%BLOCK_SIZE ] = (uint16_t) power;

    // Timestamp
	if (counter == 0)
	{
		start[PTR_BLK(write_ptr)] = call LocalTime.get();
		printfz1("Begin [%lu]\n", call LocalTime.get());
	}
    else if (counter == SAMPLE_NUM - 1) {
   		stop[PTR_BLK(write_ptr)] = call LocalTime.get();
		printfz1("End [%lu]\n", call LocalTime.get());

	}
    write_ptr ++;
   	counter ++;
    //call Alarm.start(SAMPLE_RATE);

    // a new packet is ready
    if (counter >= SAMPLE_NUM)
      {
        atomic counter = 0;	

        // if the radio is not in use
        if ( ! call ReliableTx.isSending() )
          {
            printfz1("[CLEAR SENDING] r: %5lu w:%5lu ts: %lu\n", read_ptr, write_ptr, call LocalTime.get());
            call ReliableTx.send((uint16_t*)  &write_buffer[ read_ptr % BLOCK_SIZE] , SAMPLE_NUM*RATIO, start[PTR_BLK(read_ptr)], stop[PTR_BLK(read_ptr)]);
            atomic read_ptr += SAMPLE_NUM;
          }
        else
          {
            // if it's in use and write_buffer is catching up, force next block read
            if ( write_ptr > (read_ptr + BLOCK_SIZE - SAMPLE_NUM)  )
            {
                printfz1("[DISCARD] r: %5lu w:%5lu\n", read_ptr, write_ptr);
                call ReliableTx.cancel();
                atomic read_ptr += SAMPLE_NUM;
                printfz1("[SENDING] r: %5lu w:%5lu\n", read_ptr, write_ptr);
                call ReliableTx.send( (uint16_t*) &write_buffer[read_ptr % BLOCK_SIZE] , SAMPLE_NUM*RATIO, start[PTR_BLK(read_ptr)], stop[PTR_BLK(read_ptr)]);
            }else
			{
				miss_count ++;
				printfz1("[WAITING] r: %5lu w:%5lu ts:%lu\n", read_ptr, write_ptr, call LocalTime.get());
				if (miss_count >= 10)
				{
					printfz1("[CANCEL] r: %5lu w:%5lu ts:%lu\n", read_ptr, write_ptr, call LocalTime.get());
					call ReliableTx.cancel();
                	atomic read_ptr += SAMPLE_NUM;
					miss_count = 0;
//                	call ReliableTx.send( (uint16_t*) &write_buffer[read_ptr % BLOCK_SIZE] , SAMPLE_NUM*RATIO, start[PTR_BLK(read_ptr)], stop[PTR_BLK(read_ptr)]);
				}

			}
          }
      }

  }


  event void ReliableTx.sendDone(uint16_t dropnum) {
	int i;
    printfz1("Retry: %u\n", dropnum);
    printfz1("[SUCCESS] r: %5lu w:%5lu ts:%lu\n", read_ptr, write_ptr, call LocalTime.get());
    // if successfully delivered and data is ready, start next iteration
    if ( write_ptr - SAMPLE_NUM >= read_ptr )
      {
        printfz1("[PUSHING] r: %5lu w:%5lu\n", read_ptr, write_ptr);

        call ReliableTx.send( (uint16_t*) &write_buffer[read_ptr % BLOCK_SIZE] , SAMPLE_NUM*RATIO, start[PTR_BLK(read_ptr)], stop[PTR_BLK(read_ptr)]);

		read_ptr += SAMPLE_NUM;
      }
  }

}

