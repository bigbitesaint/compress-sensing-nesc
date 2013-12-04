configuration BaseAppC {}

implementation {
  components MainC, BaseC as App, LedsC, ReliableRxC;
  App.Boot -> MainC.Boot;
  App.Leds -> LedsC;
  App.ReliableRx -> ReliableRxC;

  components ActiveMessageC;
  App.RadioControl -> ActiveMessageC;
}
