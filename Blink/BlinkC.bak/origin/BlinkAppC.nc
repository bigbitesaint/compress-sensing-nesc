#include "check.h"
configuration BlinkAppC {}

implementation {
	components MainC, BlinkC, LedsC, ;
//	components SlzwAppC;

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


  components SPlugC;
  App.SPlugControl -> SPlugC;

  components ActiveMessageC;
  App.AMControl -> ActiveMessageC;
  App.ACK -> ActiveMessageC;

  components new AMSenderC(AM_SPLUG_DATA_MSG);
  App.AMSend -> AMSenderC;
  App.Packet -> AMSenderC;

  components new AMReceiverC(AM_SPLUG_CFG_MSG);
  App.Receive -> AMReceiverC;

  components new TimerMilliC() as DataTimer;
  App.DataTimer -> DataTimer;

  components RandomC;
  App.Random -> RandomC;

  components CC2420ControlC;
  App.CC2420Config -> CC2420ControlC;

  components CC2420PacketC;
  App.CC2420Packet -> CC2420PacketC;

  components BusyWait32khzC;
  App.BusyWait -> BusyWait32khzC;


/*	components ActiveMessageC;
	BlinkC.RadioControl -> ActiveMessageC;
   components new AMReceiverC(124) as Receive;
   BlinkC.Receive -> Receive;*/
}

