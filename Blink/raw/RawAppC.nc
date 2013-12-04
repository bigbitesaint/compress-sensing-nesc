configuration RawAppC {}

implementation {
	components MainC, RawC, LedsC ;
	RawC -> MainC.Boot;
	RawC.Leds -> LedsC;

	components SPlugC;
	RawC.SPlugControl -> SPlugC;

	components ReliableTxC;
	RawC.ReliableTx -> ReliableTxC;

	components new TimerMilliC() as Timer;
	RawC.Timer -> Timer;

    components new AlarmMilli16C();
    RawC.Alarm -> AlarmMilli16C;

	components LocalTimeMilliC;
	RawC.LocalTime -> LocalTimeMilliC;

	components RandomC;
	RawC.Random -> RandomC;

	components ActiveMessageC;
	RawC.RadioControl -> ActiveMessageC;
}

