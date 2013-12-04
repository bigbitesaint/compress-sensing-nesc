configuration ReliableTxC { 
  provides interface ReliableTx;
}

implementation {
  components ReliableTxP;
  ReliableTx = ReliableTxP;

  components LedsC;
  ReliableTxP.Leds -> LedsC;

  components new AMSenderC(AM_DATA_MSG) as SPlugSender;
  components new AMReceiverC(AM_ACK_MSG) as AckReceiver;
  ReliableTxP.DataSend -> SPlugSender;
  ReliableTxP.DataPacket -> SPlugSender;
  ReliableTxP.AckReceive -> AckReceiver;

  components new TimerMilliC() as AckTimer;
  ReliableTxP.Timer -> AckTimer;

  /* components new AlarmMilli16C(); */
  /* ReliableTxP.Alarm -> AlarmMilli16C; */

  /* components RandomC; */
  /* ReliableTxP.Random -> RandomC; */
}
