#include "printfZ1.h"
#include "main.h"
#include "reliableradio.h"



#include "base_line.h"
#include "soft_aes/AES.h"
#include "calib.h"


module BLC @safe() {
  uses interface Leds;
  uses interface Boot;
  uses interface Random;
  uses interface Compressor;
  uses interface SplitControl as RadioControl;
  uses interface Timer<TMilli> as Timer;
  uses interface Alarm<TMilli, uint16_t>;
  uses interface LocalTime<TMilli>;
  uses interface SPlugControl;
  uses interface ReliableTx;
  uses interface AES;
}

implementation {

  uint32_t counter=0;
  uint16_t prev_val = 0;
  ENTRY trans_buffer[BLOCK_SIZE];
  uint32_t packet_start = 0;
  uint32_t packet_acc_trans = 0;
  uint32_t packet_n = 0;
  uint32_t start[2], stop[2];

  // for AES
  uint8_t exp[240];
  uint8_t K[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  };


  task void do_compress()
  {
    call Compressor.set(write_buffer[!clean_buffer], BLOCK_SIZE);
    call Compressor.compress(BLOCK_SIZE);
  }


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

    //enc( output_buffer, COMPRESS_SIZE);

    //call AES.decrypt((uint8_t*)output_buffer, exp, (uint8_t*)output_buffer);

    memcpy( trans_buffer, output_buffer, compressed_size*sizeof(ACC_ENTRY) );

    printfz1("start: \n");
    for (i=0;i<compressed_size*RATIO; ++i)
      printfz1("%u ",((uint16_t*)&trans_buffer)[i]);
    printfz1("\n");

    if ( call ReliableTx.isSending() )
      {
        printfz1("Radio in use\n");
        call ReliableTx.cancel();
      }
    call ReliableTx.send((uint16_t*) &trans_buffer[0], COMPRESS_SIZE*RATIO, start[!clean_buffer], stop[!clean_buffer]);
    packet_start = call LocalTime.get();
    //		printfz1("Transmission done.\n");

  }

  event void Timer.fired() {}

  event void Boot.booted() {
    printfz1_init();

    call SPlugControl.init();
    call SPlugControl.setState(SPLUG_ON_STATE);


    call ReliableTx.init();    


    /* call Timer.startPeriodic(SAMPLE_RATE); */
    call Alarm.start(SAMPLE_RATE);
   	call RadioControl.start();
    call AES.keyExpansion(exp,K);
  }

  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS)
      call RadioControl.start();
  }

  event void RadioControl.stopDone(error_t err) {}


  async event void Alarm.fired() {
    uint16_t power;
    uint8_t ready_buffer;


#ifdef TEST
#warning "Using TEST setting!"
    uint16_t power_data[16] = {
      5,5,5,5,5,5,5,5,5,5,6,4,6,5,5,5
    };
    call Alarm.start(SAMPLE_RATE);
    power = power_data[counter%16];
#else
    call Alarm.start(SAMPLE_RATE);
    power = convert(call SPlugControl.read(RAENERGY), TOS_NODE_ID, SAMPLE_RATE);
    /* power = call SPlugControl.read(RAENERGY); */
#endif
	
    // Timestamp
    if (counter == 0)
      start[clean_buffer] = call LocalTime.get();
    else if (counter == BLOCK_SIZE - 1)
      stop[clean_buffer] = call LocalTime.get();

    // the first reading after compression need to be replaced by the previous one
    if (counter == 0)
      {
        printfz1("Substitute [%u] with [%u]\n", power, prev_val);
        power = prev_val;
      }

    write_buffer[clean_buffer][counter] = power;
    counter++;
    if (counter >= BLOCK_SIZE)
      {
        counter = 0;
        prev_val = power;
        ready_buffer = clean_buffer;
        clean_buffer = !clean_buffer;
        post do_compress();
        //			call Compressor.set(write_buffer[ready_buffer], BLOCK_SIZE);
        //			call Compressor.compress(BLOCK_SIZE);
      }
  }

  event void ReliableTx.sendDone(uint16_t dropnum) {
    packet_acc_trans += call LocalTime.get() - packet_start;
    printfz1("Retry: %u\n", dropnum);
    printfz1("Transmission: %lu/%lu\n", packet_acc_trans,++packet_n);
    packet_start = call LocalTime.get();
  }
}

