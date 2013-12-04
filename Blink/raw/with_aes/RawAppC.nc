configuration RawAppC {}

implementation {
	components MainC, RawC, LedsC ;

	components SPlugC;
	RawC.SPlugControl -> SPlugC;

	components ReliableTxC;
	RawC.ReliableTx -> ReliableTxC;


	components new TimerMilliC() as Timer;
	components LocalTimeMilliC;
	RawC.LocalTime -> LocalTimeMilliC;

	RawC -> MainC.Boot;
	RawC.Leds -> LedsC;


	components AESC;
	RawC.AES -> AESC;

	components RandomC;
	RawC.Random -> RandomC;
	RawC.Timer -> Timer;


	components ActiveMessageC;
	RawC.RadioControl -> ActiveMessageC;
   components new AMReceiverC(124) as Receive;
   RawC.Receive -> Receive;
}

