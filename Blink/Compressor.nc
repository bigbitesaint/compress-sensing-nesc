interface Compressor{

	command void compress(uint16_t pos);
	command void set(uint32_t *plaintext, uint16_t size);
	command void summarize();
	event void complete(uint16_t compressed_size);
}
