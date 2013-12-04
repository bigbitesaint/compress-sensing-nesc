/*
 * Copyright (c) 2008, Shanghai Jiao Tong University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Shanghai Jiao Tong University nor the
 *   names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior
 *   written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @author Bo Zhu
 */

#include "printfZ1.h"

module AesP
{
  provides {
    interface Init;
    interface SplitControl;
    interface Encrypt;
  }
	uses {
		interface SplitControl as CC2420Control;
		interface GeneralIO as CSN;
		interface Resource as SpiResource;
		interface CC2420Register as SECCTRL0;
		interface CC2420Register as SECCTRL1;
		interface CC2420Ram as KEY0;
		interface CC2420Ram as SABUF;
		interface CC2420Strobe as SAES;
		interface CC2420Strobe as SNOP;
		interface LocalTime<TMilli>;
  }
}

implementation
{
  bool toSetKey;

  uint8_t * currentKey = NULL;
  uint8_t inverseKey[16];
  uint8_t * plain, * cipher;
  uint16_t total_time=0;
  uint16_t total_run=0;

  // prototypes
  task void setAesKey();
  task void encryptPlaintext();
  error_t acquireSpiResource();
  void releaseSpiResource();
  void invertKey();

  command error_t Init.init() {
    call CSN.makeOutput();

    return SUCCESS;
  }

  command error_t SplitControl.start() {
    return call CC2420Control.start();
  }

  command error_t SplitControl.stop() {
    return call CC2420Control.stop();
  }

  event void CC2420Control.startDone(error_t err) {
    if (SUCCESS == err) {
      signal SplitControl.startDone(SUCCESS);
    } else {
      call CC2420Control.start();
    }
  }

  event void CC2420Control.stopDone(error_t err) {
    if (SUCCESS == err) {
      signal SplitControl.stopDone(SUCCESS);
    } else {
      call CC2420Control.stop();
    }
  }

  command error_t Encrypt.setKey(uint8_t * key) {
    if (NULL != currentKey) {
      return EBUSY;
    }

    toSetKey = TRUE;
    currentKey = key;
    invertKey();

    if (SUCCESS == acquireSpiResource()) {
      post setAesKey();
    }

    return SUCCESS;
  }

  command error_t Encrypt.clrKey(uint8_t * key) {
    if (key == currentKey) {
      currentKey = NULL;

      return SUCCESS;
    }

    return FAIL;
  }

  command error_t Encrypt.putPlain(uint8_t * plaintext, uint8_t * ciphertext) {
    if (NULL == currentKey) {
      return FAIL;
    }

    toSetKey = FALSE;
    plain = plaintext;
    cipher = ciphertext;

    if (SUCCESS == acquireSpiResource()) {
      post encryptPlaintext();
    }

    return SUCCESS;
  }

  event void SpiResource.granted() {
    if (TRUE == toSetKey) {
      post setAesKey();
    } else {
      post encryptPlaintext();
    }
  }

  task void setAesKey() {
    call CSN.clr();
    call SECCTRL0.write(0x0000);
//  call CSN.set();

//  call CSN.clr();
    call SECCTRL1.write(0x0000);
//  call CSN.set();

//  call CSN.clr();
    call KEY0.write(0, inverseKey, 16);
    call CSN.set();

    signal Encrypt.setKeyDone(currentKey);
    releaseSpiResource();
  }

  task void encryptPlaintext() {
    cc2420_status_t status;
	uint32_t start_time, end_time;
	uint16_t i=0;
	
    start_time = call LocalTime.get();
	    call CSN.clr();
	    call SABUF.write(0, plain, 16);
    	call CSN.set();

	    call CSN.clr();
    	status = call SAES.strobe();
//  call CSN.set();

	    do {
//    call CSN.clr();
    	  status = call SNOP.strobe();
//    call CSN.set();
	    } while (status & CC2420_STATUS_ENC_BUSY);

//  call CSN.clr();
    	call SABUF.read(0, cipher, 16);
	    call CSN.set();

    end_time = call LocalTime.get();
	 printfz1("Elapsed: %lu\n", end_time-start_time);
	 total_time += end_time-start_time;
	 total_run ++;
	 printfz1("Average time: %u/%u\n",total_time, total_run);
    signal Encrypt.getCipher(plain, cipher);
    releaseSpiResource();
  }

  error_t acquireSpiResource() {
    error_t error = call SpiResource.immediateRequest();
    if ( error != SUCCESS ) {
      call SpiResource.request();
    }
    return error;
  }

  void releaseSpiResource() {
    call SpiResource.release();
  }

  // see https://community.ti.com/forums/t/363.aspx
  void invertKey() {
    uint8_t i;

    for (i = 0; i <= 15; i++) {
      inverseKey[i] = currentKey[15 - i];
    }
  }
}

