configuration CSAppC {}

implementation {
	components MainC, CSC, LedsC ;
	CSC -> MainC.Boot;
	CSC.Leds -> LedsC;

	components SPlugC;
	CSC.SPlugControl -> SPlugC;

	components ReliableTxC;
	CSC.ReliableTx -> ReliableTxC;

	components CompressSensingAppC;
	CSC.Compressor -> CompressSensingAppC.Compressor;

	/* components new TimerMilliC() as Timer; */
	/* CSC.Timer -> Timer; */

    components new AlarmMilli16C();
    CSC.Alarm -> AlarmMilli16C;

	components LocalTimeMilliC;
	CSC.LocalTime -> LocalTimeMilliC;

	components RandomC;
	CSC.Random -> RandomC;


  	components AESC;
  	CSC.AES -> AESC;

	components ActiveMessageC;
	CSC.RadioControl -> ActiveMessageC;
}

