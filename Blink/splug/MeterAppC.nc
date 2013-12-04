#include "main.h"

configuration MeterAppC
{
}
implementation
{
  components MainC, MeterC as App, LedsC;
  App -> MainC.Boot;
  App.Leds -> LedsC;

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
}

