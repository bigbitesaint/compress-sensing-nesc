#include "printfZ1.h"
#include "main.h"
#include "reliableradio.h"
#include "calib.h"


#include "lossless.h"
#include "slzw/slzw.h"



module LLC @safe() {
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
  uint16_t val2;
  ENTRY trans_buffer[COMPRESS_SIZE+1];
  uint32_t packet_start = 0;
  uint32_t packet_acc_trans = 0;
  uint32_t packet_n = 0;
  // indicates which buffer is clean to write
  uint8_t clean_buffer = 0;
  uint32_t start[2], stop[2];

  // for AES
  uint8_t exp[240];
  uint8_t K[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F,
  };


  task void do_compress() {
    call Compressor.set(double_buffer[!clean_buffer], BLOCK_SIZE);
    call Compressor.compress(BLOCK_SIZE);
  }

  // len is the number of ENTRY
  uint16_t enc(unsigned char* src, uint16_t len)
  {
    uint16_t i;
    uint32_t start_time, end_time;
    start_time = call LocalTime.get();
    for (i=0;i < len; i+= 16 )
      call AES.encrypt(&src[i],exp,&src[i]);
    end_time = call LocalTime.get();
    printfz1("Encrypt %4d bytes for : %lu ms\n", len ,end_time-start_time);
    return len;
  }




  event void Compressor.complete(uint16_t compressed_size) {
    uint16_t i;
    uint16_t pad_compressed_size = 0;

    printfz1("Compression ratio: %d/%d\n", BLOCK_SIZE*sizeof(ENTRY), compressed_size);
    call Compressor.summarize();

    memset( write_buffer, 0, sizeof(write_buffer) );
    //		slzw_decompress(compressed_size);
    printfz1("Before encryption...\n");
    for (i=0; i< compressed_size; ++i)
      printfz1("%u ",lzw_output_file_buffer[i]);
    printfz1("\n");


    compressed_size = enc( lzw_output_file_buffer, compressed_size);
    printfz1("Slze compressed size: %lu\n", compressed_size);
    pad_compressed_size = ( compressed_size/16 + (compressed_size%16 != 0) )*16;

    trans_buffer[0] = compressed_size;
    memcpy( &trans_buffer[1], lzw_output_file_buffer, pad_compressed_size );

    printfz1("0");
    for (i=0; i< pad_compressed_size/2; ++i)
      printfz1(",%u", ((uint16_t*)trans_buffer)[i]);
    printfz1("\n");


    if ( call ReliableTx.isSending() )
      printfz1("Radio in use\n");

    call ReliableTx.cancel();
    call ReliableTx.send((uint16_t*) &trans_buffer[0], RATIO + pad_compressed_size/RATIO, start[!clean_buffer], stop[!clean_buffer]);
    packet_start = call LocalTime.get();
    printfz1("Transmission done.\n");
  }

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

  event void Timer.fired() {}

  async event void Alarm.fired() {
    uint16_t power;
    uint8_t ready_buffer;

#ifdef TEST
    uint16_t power_data[] = {25,24,25,25,25,24,24,26,24,25,24,25,24,26,24,24,26,24,25,27,22,25,24,25,24,25,25,25,24,25,25,24};
    call Alarm.start(SAMPLE_RATE);    
    power = power_data[counter%32];
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

    double_buffer[clean_buffer][counter] = power;
    counter++;
    if (counter >= BLOCK_SIZE)
      {

        counter = 0;
        ready_buffer = clean_buffer;
        clean_buffer = !clean_buffer;
        post do_compress();
        /* call Compressor.set(double_buffer[ready_buffer], BLOCK_SIZE); */
        /* call Compressor.compress(BLOCK_SIZE); */
      }
  }


  event void ReliableTx.sendDone(uint16_t dropnum) {
    packet_acc_trans += call LocalTime.get() - packet_start;
    printfz1("Retry: %u\n", dropnum);
    printfz1("Transmission: %lu/%lu\n", packet_acc_trans,++packet_n);
    packet_start = call LocalTime.get();
  }

}

