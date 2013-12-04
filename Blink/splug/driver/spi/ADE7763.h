#define CURRENT_WAVEFORM 0x400C
#define VOLTAGE_WAVEFORM 0x600C
#define ACPOWER_WAVEFORM 0x000C

#define ADDRESS_SIZE 8

#define WAVEFORM 0x01
#define WAVEFORM_SIZE 24

#define AENERGY 0x02
#define AENERGY_SIZE 24

#define RAENERGY 0x03
#define RAENERGY_SIZE 24

#define LAENERGY 0x04
#define LAENERGY_SIZE 24

#define VAENERGY 0x05
#define VAENERGY_SIZE 24

#define RVAENERGY 0x06
#define RVAENERGY_SIZE 24

#define LVAENERGY 0x07
#define LVAENERGY_SIZE 24

#define RESERVED 0x08
#define RESERVED_SIZE 16

#define MODE 0x09
#define MODE_SIZE 16

#define IRQEN 0x0A
#define IRQEN_SIZE 16

#define STATUS 0x0B
#define STATUS_SIZE 16

#define RSTSTATUS 0x0C
#define RSTSTATUS_SIZE 16

#define CH1OS 0x0D
#define CH1OS_SIZE 8

#define CH2OS 0x0E
#define CH2OS_SIZE 8

#define GAIN 0x0F
#define GAIN_SIZE 8

#define PHCAL 0x10
#define PHCAL_SIZE 6

#define APOS 0x11
#define APOS_SIZE 16

#define WGAIN 0x12
#define WGAIN_SIZE 12

#define WDIV 0x13
#define WDIV_SIZE 8

#define CFNUM 0x14
#define CFNUM_SIZE 12

#define CFDEN 0x15
#define CFDEN_SIZE 12

#define IRMS 0x16
#define IRMS_SIZE 24

#define VRMS 0x17
#define VRMS_SIZE 24

#define IRMSOS 0x18
#define IRMSOS_SIZE 12

#define VRMSOS 0x19
#define VRMSOS_SIZE 12

#define VAGAIN 0x1A
#define VAGAIN_SIZE 12

#define VADIV 0x1B
#define VADIV_SIZE 8

#define LINECYC 0x1C
#define LINECYC_SIZE 16

#define ZXTOUT 0x1D
#define ZXTOUT_SIZE 12

#define SAGCYC 0x1E
#define SAGCYC_SIZE 8

#define SAGLVL 0x1F
#define SAGLVL_SIZE 8

#define IPKLVL 0x20
#define IPKLVL_SIZE 8

#define VPKLVL 0x21
#define VPKLVL_SIZE 8

#define IPEAK 0x22
#define IPEAK_SIZE 24

#define RSTIPEAK 0x23
#define RSTIPEAK_SIZE 24

#define VPEAK 0x24
#define VPEAK_SIZE 24

#define RSTVPEAK 0x25
#define RSTVPEAK_SIZE 24

#define TEMP 0x26
#define TEMP_SIZE 8

#define PERIOD 0x27
#define PERIOD_SIZE 16

#define TMODE 0x3D
#define TMODE_SIZE 8

#define CHKSUM 0x3E
#define CHKSUM_SIZE 6

#define DIEREV 0x3F
#define DIEREV_SIZE 8

uint8_t getRegisterSize(uint8_t address) {
  uint8_t length;

    switch (address) {
    case WAVEFORM: length = WAVEFORM_SIZE; break;
    case AENERGY: length = AENERGY_SIZE; break;
    case RAENERGY: length = RAENERGY_SIZE; break;
    case LAENERGY: length = LAENERGY_SIZE; break;
    case VAENERGY: length = VAENERGY_SIZE; break;
    case RVAENERGY: length = RVAENERGY_SIZE; break;
    case LVAENERGY: length = LVAENERGY_SIZE; break;
    case RESERVED: length = RESERVED_SIZE; break;
    case MODE: length = MODE_SIZE; break;
    case IRQEN: length = IRQEN_SIZE; break;
    case STATUS: length = STATUS_SIZE; break;
    case RSTSTATUS: length = RSTSTATUS_SIZE; break;
    case CH1OS: length = CH1OS_SIZE; break;
    case CH2OS: length = CH2OS_SIZE; break;
    case GAIN: length = GAIN_SIZE; break;
    case PHCAL: length = PHCAL_SIZE; break;
    case APOS: length = APOS_SIZE; break;
    case WGAIN: length = WGAIN_SIZE; break;
    case WDIV: length = WDIV_SIZE; break;
    case CFNUM: length = CFNUM_SIZE; break;
    case CFDEN: length = CFDEN_SIZE; break;
    case IRMS: length = IRMS_SIZE; break;
    case VRMS: length = VRMS_SIZE; break;
    case IRMSOS: length = IRMSOS_SIZE; break;
    case VRMSOS: length = VRMSOS_SIZE; break;
    case VAGAIN: length = VAGAIN_SIZE; break;
    case VADIV: length = VADIV_SIZE; break;
    case LINECYC: length = LINECYC_SIZE; break;
    case ZXTOUT: length = ZXTOUT_SIZE; break;
    case SAGCYC: length = SAGCYC_SIZE; break;
    case SAGLVL: length = SAGLVL_SIZE; break;
    case IPKLVL: length = IPKLVL_SIZE; break;
    case VPKLVL: length = VPKLVL_SIZE; break;
    case IPEAK: length = IPEAK_SIZE; break;
    case RSTIPEAK: length = RSTIPEAK_SIZE; break;
    case VPEAK: length = VPEAK_SIZE; break;
    case RSTVPEAK: length = RSTVPEAK_SIZE; break;
    case TEMP: length = TEMP_SIZE; break;
    case PERIOD: length = PERIOD_SIZE; break;
    case TMODE: length = TMODE_SIZE; break;
    case CHKSUM: length = CHKSUM_SIZE; break;
    case DIEREV: length = DIEREV_SIZE; break;
    default: length = 8;
    }
    return length;
}

bool isWritable(uint8_t address) {
  if (address == MODE || address == IRQEN || address == CH1OS || address == CH2OS || 
      address == GAIN || address == PHCAL || address == APOS || address == WGAIN || 
      address == WDIV || address == CFNUM || address == CFDEN || address == IRMSOS || 
      address == VRMSOS || address == VAGAIN || address == VADIV || address == LINECYC || 
      address == ZXTOUT || address == SAGCYC || address == SAGLVL || address == IPKLVL || 
      address == VPKLVL || address == TMODE)
    return TRUE;
  else
    return FALSE;
}
