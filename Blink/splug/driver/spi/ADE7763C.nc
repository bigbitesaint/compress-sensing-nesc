configuration ADE7763C {
  provides interface ADE7763;
}

implementation {
  components ADE7763P;
  ADE7763 = ADE7763P;
  components LedsC;
  ADE7763P.Leds -> LedsC;

  components Counter32khz32C, new CounterToLocalTimeC(T32khz) as LocalTime32khzC;
  LocalTime32khzC.Counter -> Counter32khz32C;
  ADE7763P.LocalTime -> LocalTime32khzC;

  //components BusyWaitMicroC;
  //ADE7763P.BusyWait -> BusyWaitMicroC;
}
