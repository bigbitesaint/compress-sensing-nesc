#include "printfZ1.h"
#include "traffic.h"
#include "reliableradio.h"

module TrafficC @safe() {
  uses interface Leds;
  uses interface Boot;
  uses interface Random;
  uses interface SplitControl as RadioControl;
  uses interface Timer<TMilli> as Timer;
  uses interface Alarm<TMilli, uint16_t>;
  uses interface LocalTime<TMilli>;
  uses interface AMSend as DataSend;
  uses interface Packet as DataPacket;
}

implementation {
  message_t packet;
  uint32_t prev;
  uint32_t curr;
  uint16_t idx;
  uint16_t buf[SAMPLE_NUM];

  void send() {
    uint16_t pktlen = SAMPLE_NUM * sizeof(uint16_t) + sizeof(reliable_msg_t);
    reliable_msg_t *dataMsg = (reliable_msg_t *) call DataPacket.getPayload(&packet, pktlen);
    uint16_t i;

    dataMsg->nodeid = TOS_NODE_ID;
    dataMsg->start = prev;
    dataMsg->stop = curr;

    for (i = 0; i < SAMPLE_NUM; i++)
      dataMsg->data[i] = buf[i];

    if (call DataSend.send(BASEID, &packet, pktlen) != SUCCESS)
      send();
  }

  event void Boot.booted() {
    printfz1_init();
    prev = call LocalTime.get();
    curr = prev;
   	call RadioControl.start();
  }

  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS)
      call RadioControl.start();
    else
      send();
  }

  event void RadioControl.stopDone(error_t err) {}

  event void Timer.fired() {}

  async event void Alarm.fired() {}

  event void DataSend.sendDone(message_t *msg, error_t error) {
    if (error == SUCCESS) {
      prev = curr;
      curr = call LocalTime.get();
      buf[idx] = (uint16_t) (curr - prev);
      idx = (idx + 1) % SAMPLE_NUM;
      call Leds.led1Toggle();
    }
    else
      call Leds.led0Toggle();
    send();
  }
}

