configuration ReliableRxC { 
  provides interface ReliableRx;
}

implementation {
  components ReliableRxP;
  ReliableRx = ReliableRxP;

  components LedsC;
  ReliableRxP.Leds -> LedsC;

  components new AMReceiverC(AM_DATA_MSG) as DataReceiver;
  components new AMSenderC(AM_ACK_MSG) as AckSender;
  ReliableRxP.DataReceiver -> DataReceiver;
  ReliableRxP.AckSender -> AckSender;
  ReliableRxP.Packet -> AckSender;

  components CC2420PacketC;
  ReliableRxP.CC2420Packet -> CC2420PacketC;

  components CC2420ControlC;
  ReliableRxP.CC2420Config -> CC2420ControlC;
}
