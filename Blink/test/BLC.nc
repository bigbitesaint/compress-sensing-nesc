#include "printfZ1.h"



module BLC @safe() {
	uses interface Boot;
	uses interface Timer<TMilli> as Timer;
	uses interface LocalTime<TMilli>;
}

implementation {

	uint16_t k=0;

	void busy()
	{
		float i=3.0;
		uint32_t j = 0;
		for (j=0;;++j)
		{
			i = sqrtf(i);
			if ( (j % 1024*1024) == 0)
				printfz1("[%u] %d\n",call LocalTime.get(), k);
		}
	}

	event void Boot.booted() {
		printfz1_init();
		call Timer.startPeriodic(128);
		busy();
  }
	async event void Timer.fired() {
		printfz1("[%6lu] Timer!\n", call LocalTime.get());
		++k;
  }


}

