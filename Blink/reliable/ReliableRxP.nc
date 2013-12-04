#include "reliableradio.h"

module ReliableRxP {
  provides {
    interface ReliableRx;
  }

  uses {
    interface Leds;
    interface Receive as DataReceiver;
    interface AMSend as AckSender;
    interface Packet;
    interface CC2420Packet;
    interface CC2420Config;
  }
}

implementation {
  message_t packet;
  reliable_msg_t *dataMsg;
  ack_msg_t *ackMsg;
  int8_t rssi, lqi;

  uint8_t radioLocked = FALSE;

  event message_t* DataReceiver.receive(message_t* pkt, void* payload, uint8_t len) {
    if (radioLocked == FALSE) {
      dataMsg = (reliable_msg_t *) payload;
      ackMsg = (ack_msg_t *) call Packet.getPayload(&packet, sizeof(ack_msg_t));
      ackMsg->cookie = dataMsg->cookie;
      rssi = call CC2420Packet.getRssi(pkt);
      lqi = call CC2420Packet.getLqi(pkt);

      if (call AckSender.send(dataMsg->nodeid, &packet, sizeof(ack_msg_t)) == SUCCESS)
        atomic radioLocked = TRUE;

      signal ReliableRx.receive(dataMsg, len, rssi, lqi);
      call Leds.led1Toggle();
    }

    return pkt;
  }

  event void AckSender.sendDone(message_t *pkt, error_t error) {
    atomic radioLocked = FALSE;
  }

  event void CC2420Config.syncDone(error_t err) {}
}
