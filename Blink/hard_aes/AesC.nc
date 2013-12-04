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

configuration AesC
{
  provides {
    interface SplitControl;
    interface Encrypt;
  }
}

implementation
{
  components AesP;
  SplitControl = AesP.SplitControl;
  Encrypt = AesP.Encrypt;

  components MainC;
  MainC.SoftwareInit -> AesP.Init;


  components LocalTimeMilliC;
  AesP.LocalTime -> LocalTimeMilliC;

  components ActiveMessageC;
  AesP.CC2420Control -> ActiveMessageC;

  components HplCC2420PinsC as Pins;
  AesP.CSN -> Pins.CSN;

  components new AesRamP();
  AesP.KEY0 -> AesRamP.KEY0;
//AesP.KEY1 -> AesRamP.KEY1;
  AesP.SABUF -> AesRamP.SABUF;

  components new CC2420SpiC() as Spi;
/*
  AesP.KEY0 -> Spi.KEY0;
//AesP.KEY1 -> Spi.KEY1;
  AesP.SABUF -> Spi.SABUF;
*/
  AesP.SpiResource -> Spi;
  AesP.SECCTRL0 -> Spi.SECCTRL0;
  AesP.SECCTRL1 -> Spi.SECCTRL1;
  AesP.SNOP -> Spi.SNOP;
  AesP.SAES -> Spi.SAES;
}
