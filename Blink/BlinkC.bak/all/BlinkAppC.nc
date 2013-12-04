#include "check.h"
configuration BlinkAppC {}

implementation {
	components MainC, BlinkC, LedsC ;
//	components SlzwAppC;

	components SPlugC;
	BlinkC.SPlugControl -> SPlugC;

	components ReliableTxC;
	BlinkC.ReliableTx -> ReliableTxC;


#ifdef USE_CS
	components CompressSensingAppC;
#endif

#ifdef USE_BASELINE
	components BaseLineAppC;
#endif
	components new TimerMilliC() as Timer;
	components LocalTimeMilliC;
	BlinkC.LocalTime -> LocalTimeMilliC;

	BlinkC -> MainC.Boot;
	BlinkC.Leds -> LedsC;
//	BlinkC.Compressor -> SlzwAppC.Compressor;
#ifdef USE_CS
	BlinkC.Compressor -> CompressSensingAppC.Compressor;
#endif


#ifdef USE_BASELINE
	BlinkC.Compressor -> BaseLineAppC.Compressor;
#endif

	components RandomC;
	BlinkC.Random -> RandomC;
	BlinkC.Timer -> Timer;

#ifdef USE_SAES
	components AESC;
	BlinkC.AES -> AESC;
#endif



	components ActiveMessageC;
	BlinkC.RadioControl -> ActiveMessageC;
   components new AMReceiverC(124) as Receive;
   BlinkC.Receive -> Receive;
}

