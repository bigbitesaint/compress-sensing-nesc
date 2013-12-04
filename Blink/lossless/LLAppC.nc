configuration LLAppC {}

implementation {
  components MainC, LLC, LedsC ;
  LLC -> MainC.Boot;
  LLC.Leds -> LedsC;

  components SPlugC;
  LLC.SPlugControl -> SPlugC;

  components ReliableTxC;
  LLC.ReliableTx -> ReliableTxC;

  components SlzwAppC;
  LLC.Compressor -> SlzwAppC.Compressor;

  components new TimerMilliC() as Timer;
  LLC.Timer -> Timer;

  components new AlarmMilli16C();
  LLC.Alarm -> AlarmMilli16C;

  components LocalTimeMilliC;
  LLC.LocalTime -> LocalTimeMilliC;

  components RandomC;
  LLC.Random -> RandomC;

  components AESC;
  LLC.AES -> AESC;

  components ActiveMessageC;
  LLC.RadioControl -> ActiveMessageC;
}

