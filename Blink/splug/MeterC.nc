#include "main.h"

module MeterC @safe() {
  uses interface Boot;
  uses interface Leds;

  uses interface SPlugControl;

  uses interface Receive;
  uses interface AMSend;
  uses interface Packet;
  uses interface SplitControl as AMControl;
  uses interface PacketAcknowledgements as ACK;

  uses interface Timer<TMilli> as DataTimer;

  uses interface Random;
  uses interface CC2420Config;
  uses interface CC2420Packet;

  uses interface BusyWait<T32khz,uint16_t>;
}

implementation {
  uint8_t count;
  message_t cfg_pkt;
  splug_cfg_msg_t *cfg;

  message_t data_pkt;
  splug_data_msg_t *data;

  bool lockRadio = FALSE;

  //===========================================================================//

  uint32_t randomInterval() {
    uint32_t seed = call Random.rand16();
    uint32_t time = (cfg->interval_upper - cfg->interval_lower) * seed;
    time >>= 15;
    time += cfg->interval_lower;
    return time;
  }

  void updateNewChannel(uint8_t channel) {
    call CC2420Config.setChannel(channel);
    call CC2420Config.sync(); // ~47 ticks @ 32kHz
  }

  void updateNewRfPower(uint8_t rfpower) {
    call CC2420Packet.setPower(&cfg_pkt, rfpower); // < 1 tick @ 32 kHz
    call CC2420Packet.setPower(&data_pkt, rfpower);
  }

  void sendDataMsg() {
    if (lockRadio)
      return;

    data->seq_no = (data->seq_no + 1) % 0xFF;
    data->read_val1 = call SPlugControl.read(cfg->read_reg1);
    data->read_val2 = call SPlugControl.read(cfg->read_reg2);

    if (call AMSend.send(cfg->baserx_id, &data_pkt, sizeof(splug_data_msg_t)) == SUCCESS)
      lockRadio = TRUE;
  }

  void sendCfgMsg() {
      if (lockRadio)
        return;

      if (call AMSend.send(cfg->baserx_id, &cfg_pkt, sizeof(splug_cfg_msg_t)) == SUCCESS)
        lockRadio = TRUE;
  }

  //===========================================================================//

  void setCfg(splug_cfg_msg_t *tmp) {
    // Base ID
    cfg->baserx_id = tmp->baserx_id;

    // Data update frequency
    if (tmp->interval_upper >= tmp->interval_lower && 
        tmp->interval_lower >= INTERVAL_MIN) {
      cfg->interval_lower = tmp->interval_lower;
      cfg->interval_upper = tmp->interval_upper;
    }

    // Radio channel
    if (tmp->channel >= CHANNEL_MIN && tmp->channel <= CHANNEL_MAX &&
        tmp->channel != call CC2420Config.getChannel()) {
      updateNewChannel(tmp->channel);
      cfg->channel = call CC2420Config.getChannel();
    }

    // Radio frequency power
    if (tmp->rfpower >= RFPOWER_MIN && tmp->rfpower <= RFPOWER_MAX &&
        tmp->rfpower != call CC2420Packet.getPower(&data_pkt)) {
      updateNewRfPower(tmp->rfpower);
      cfg->rfpower = call CC2420Packet.getPower(&data_pkt);
    }

    // Read register
    if (tmp->read_reg1 <= REG_MAX)
      cfg->read_reg1 = tmp->read_reg1;
    if (tmp->read_reg2 <= REG_MAX)
      cfg->read_reg2 = tmp->read_reg2;

    // Write register
    if (tmp->write_reg <= REG_MAX) {
      call SPlugControl.write(tmp->write_reg, (uint32_t)(tmp->write_val));
      cfg->write_reg = tmp->write_reg;
      cfg->write_val = tmp->write_val;
    }
    
    // Enable data msg
    if (GET(tmp, SPLUG_DATA_BIT))
      SET(cfg, SPLUG_DATA_BIT);
    else
      CLR(cfg, SPLUG_DATA_BIT);

    // Set state
    if (GET(tmp, SPLUG_STATE_BIT)) {
      call SPlugControl.setState(SPLUG_ON_STATE);
      SET(cfg, SPLUG_STATE_BIT);
    }
    else {
      call SPlugControl.setState(SPLUG_OFF_STATE);
      CLR(cfg, SPLUG_STATE_BIT);
    }
  }

  //===========================================================================//

  event void Boot.booted() {
    call SPlugControl.init();
    call SPlugControl.setState(SPLUG_ON_STATE);
    call AMControl.start();

    printfz1_init();

    cfg = (splug_cfg_msg_t*) call Packet.getPayload(&cfg_pkt, sizeof(splug_cfg_msg_t));
    initSPlugCfgMsg(cfg);
    call ACK.requestAck(&cfg_pkt);
    count = 0;

    data = (splug_data_msg_t*) call Packet.getPayload(&data_pkt, sizeof(splug_data_msg_t));
    data->sensor_id = TOS_NODE_ID;
    data->seq_no = 0;
    data->read_val1 = 0;
    data->read_val2 = 0;

    updateNewChannel(cfg->channel);
    updateNewRfPower(cfg->rfpower);
  }

  event void AMControl.startDone(error_t err) {
    if (err == SUCCESS) {
      call DataTimer.startOneShot(randomInterval());
      atomic sendCfgMsg();
    }
    else
      call AMControl.start();
  }

  event void DataTimer.fired() {
    call DataTimer.startOneShot(randomInterval());

    if (GET(cfg, SPLUG_DATA_BIT))
      atomic sendDataMsg();
  }

  event void AMSend.sendDone(message_t* msg, error_t err) {
    atomic {
      lockRadio = FALSE;
      call Leds.led1Toggle();

      if (msg != &cfg_pkt)
        return;

      // Handle config message to base station node
      if ((err == SUCCESS && call ACK.wasAcked(msg)) || ++count >= RETRANSMISSION_MAX)
        count = 0;
      else
        sendCfgMsg();
    }
  }

  event message_t* Receive.receive(message_t* bufPtr, void* payload, uint8_t len) {
    if (len == sizeof(splug_cfg_msg_t)) {
      splug_cfg_msg_t *tmp = (splug_cfg_msg_t *) payload;

      atomic {
        if (tmp->sensor_id != TOS_NODE_ID)
          return bufPtr;

        else if (GET(tmp, SPLUG_SET_BIT)) {
          if (GET(tmp, SPLUG_RESET_BIT))
            WDTCTL = 0;
          else
            setCfg(tmp);
        }
      
        sendCfgMsg();
      }
    }

    return bufPtr;
  }

  event void AMControl.stopDone(error_t err) {}

  event void CC2420Config.syncDone(error_t err) {}
}
