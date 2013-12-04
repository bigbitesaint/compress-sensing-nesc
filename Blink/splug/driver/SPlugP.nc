#include "main.h"

module SPlugP {
  provides interface SPlugControl;

  uses interface ADE7763 as Spi;
  uses interface Leds;
  uses interface GeneralIO as Power;
  uses interface BusyWait<TMicro, uint16_t>;
}

implementation {
  uint8_t state;

  command error_t SPlugControl.init() {
    atomic {
      call Power.makeOutput();
      call Power.set();
      state = SPLUG_ON_STATE;

      call Spi.init();
      call SPlugControl.write(GAIN, 0x44);
      call SPlugControl.write(MODE, 0x600C);
      call SPlugControl.write(IRQEN, 0x28);
    }
    return SUCCESS;
  }

  //===========================================================================//w

  command void SPlugControl.setState(uint8_t s) {
    if (s == SPLUG_ON_STATE) {
      call Power.set();
      /* call Leds.led2On(); */
    }
    else if (s == SPLUG_OFF_STATE) {
      call Power.clr();
      /* call Leds.led2Off(); */
    }
    else
      return;
    state = s;
  }

  command uint8_t SPlugControl.getState() {
    return state;
  }

  //===========================================================================//

  command error_t SPlugControl.write(uint8_t address, uint32_t value) {
    if (!isWritable(address))
      return FAIL;

    call Spi.cs_low();
    call Spi.write(address, value, getRegisterSize(address));
    call Spi.cs_high();

    return SUCCESS;
  }

  command uint32_t SPlugControl.read(uint8_t address) {
    uint32_t value;
    uint32_t modeval;


    if (address == TEMP) {
      // Need to set TEMPSEL bit in MODE register and wait until that bit is cleared
      // (which indicates conversion completion) before reading from TEMP register
      modeval = call SPlugControl.read(MODE);
      modeval |= 0x20;
      call SPlugControl.write(MODE, modeval);
      do {
        modeval = call SPlugControl.read(MODE);
      } while (modeval & 0x20);
    }
    
    call Spi.cs_low();
    value = call Spi.read(address, getRegisterSize(address));
    call Spi.cs_high();

    return value;
  }
}

