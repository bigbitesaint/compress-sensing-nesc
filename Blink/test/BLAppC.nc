configuration BLAppC {}

implementation {
	components MainC;

	components BLC;
	components new TimerMilliC() as Timer;
	components LocalTimeMilliC;
	BLC.LocalTime -> LocalTimeMilliC;
	BLC.Timer -> Timer;

	BLC -> MainC.Boot;
}

