configuration BLAppC {}

implementation {
  components MainC, BLC, LedsC ;
  BLC -> MainC.Boot;
  BLC.Leds -> LedsC;

  components SPlugC;
  BLC.SPlugControl -> SPlugC;

  components ReliableTxC;
  BLC.ReliableTx -> ReliableTxC;

  components new TimerMilliC() as Timer;
  BLC.Timer -> Timer;

  components new AlarmMilli16C();
  BLC.Alarm -> AlarmMilli16C;

  components LocalTimeMilliC;
  BLC.LocalTime -> LocalTimeMilliC;

  components BaseLineAppC;
  BLC.Compressor -> BaseLineAppC.Compressor;

  components RandomC;
  BLC.Random -> RandomC;

  components AESC;
  BLC.AES -> AESC;

  components ActiveMessageC;
  BLC.RadioControl -> ActiveMessageC;
}

