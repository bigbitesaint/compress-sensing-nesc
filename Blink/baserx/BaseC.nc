#include "reliableradio.h"
#include "printfZ1.h"

module BaseC {
  uses {
    interface Boot;
    interface Leds;
    interface ReliableRx;
    interface SplitControl as RadioControl;
  }
}

implementation {

  event void Boot.booted() {
    printfz1_init();
    call RadioControl.start();
  }
  
  event void ReliableRx.receive(reliable_msg_t* dataMsg, uint8_t len, int8_t rssi, int8_t lqi) {
    uint16_t i, datalen = (len - sizeof(reliable_msg_t)) / sizeof(uint16_t);

    printfz1("%u,%lu,%lu,%u,%u,%u,%u,%d,%d", dataMsg->nodeid, dataMsg->start, dataMsg->stop, 
             dataMsg->blkno, dataMsg->cookie, dataMsg->pktno, dataMsg->dropnum, rssi, lqi);

    for (i = 0; i < datalen; i++)
      printfz1(",%u", dataMsg->data[i]);
    printfz1("\n");
  }

  event void RadioControl.startDone(error_t err) {}

  event void RadioControl.stopDone(error_t err) {}
}
