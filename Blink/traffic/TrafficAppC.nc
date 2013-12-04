configuration TrafficAppC {}

implementation {
  components MainC, TrafficC, LedsC ;
  TrafficC -> MainC.Boot;
  TrafficC.Leds -> LedsC;

  components new AMSenderC(AM_DATA_MSG) as SPlugSender;
  TrafficC.DataSend -> SPlugSender;
  TrafficC.DataPacket -> SPlugSender;

  components new TimerMilliC() as Timer;
  TrafficC.Timer -> Timer;

  components new AlarmMilli16C();
  TrafficC.Alarm -> AlarmMilli16C;

  components LocalTimeMilliC;
  TrafficC.LocalTime -> LocalTimeMilliC;

  components RandomC;
  TrafficC.Random -> RandomC;

  components ActiveMessageC;
  TrafficC.RadioControl -> ActiveMessageC;
}
