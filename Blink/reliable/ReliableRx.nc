interface ReliableRx {
  event void receive(reliable_msg_t* dataMsg, uint8_t len, int8_t rssi, int8_t lqi);
}
