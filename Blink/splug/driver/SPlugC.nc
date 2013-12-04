#include "main.h"

configuration SPlugC { 
	provides interface SPlugControl;
}

implementation {
	components MainC, SPlugP, LedsC;
	SPlugControl = SPlugP;
	SPlugP.Leds -> LedsC;

	components ADE7763C;
	SPlugP.Spi -> ADE7763C; 

	components HplMsp430GeneralIOC, new Msp430GpioC() as port51g;
	port51g.HplGeneralIO -> HplMsp430GeneralIOC.Port51;
	SPlugP.Power -> port51g;

    components BusyWaitMicroC;
    SPlugP.BusyWait -> BusyWaitMicroC;
}
