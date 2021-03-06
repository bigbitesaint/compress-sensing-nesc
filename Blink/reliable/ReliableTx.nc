interface ReliableTx {
  command void init();
  command void send(uint16_t* data, uint16_t len, uint32_t start, uint32_t stop);
  command bool isSending();
  event void sendDone(uint16_t dropnum);
  command void cancel();
}
