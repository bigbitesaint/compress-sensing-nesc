configuration BLAppC {}

implementation {
	components MainC, BLC, LedsC ;

	components SPlugC;
	BLC.SPlugControl -> SPlugC;

	components ReliableTxC;
	BLC.ReliableTx -> ReliableTxC;


	components BaseLineAppC;
	components new TimerMilliC() as Timer;
	components LocalTimeMilliC;
	BLC.LocalTime -> LocalTimeMilliC;

	BLC -> MainC.Boot;
	BLC.Leds -> LedsC;

	BLC.Compressor -> BaseLineAppC.Compressor;

	components RandomC;
	BLC.Random -> RandomC;
	BLC.Timer -> Timer;

	components AESC;
	BLC.AES -> AESC;



	components ActiveMessageC;
	BLC.RadioControl -> ActiveMessageC;
   components new AMReceiverC(124) as Receive;
   BLC.Receive -> Receive;
}

