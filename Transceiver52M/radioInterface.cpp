/*
* Copyright 2008, 2009, 2012 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Affero Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "radioInterface.h"
#include <Logger.h>

bool started = false;

/* Device side buffers */
static short rx_buf[OUTCHUNK * 2 * 2];
static short tx_buf[INCHUNK * 2 * 2];

/* Complex float to short conversion */
static void floatToShort(short *out, float *in, int num)
{
  for (int i = 0; i < num; i++) {
    out[2 * i + 0] = (short) in[2 * i + 0];
    out[2 * i + 1] = (short) in[2 * i + 1];
  }
}

/* Complex short to float conversion */
static void shortToFloat(float *out, short *in, int num)
{
  for (int i = 0; i < num; i++) {
    out[2 * i + 0] = (float) in[2 * i + 0];
    out[2 * i + 1] = (float) in[2 * i + 1];
  }
}

RadioInterface::RadioInterface(RadioDevice *wRadio,
			       int wChanM,
			       int wReceiveOffset,
			       int wSPS,
			       GSM::Time wStartTime)
  : mChanM(wChanM), underrun(false), sendCursor(0), rcvCursor(0), mOn(false),
    mRadio(wRadio), receiveOffset(wReceiveOffset),
    samplesPerSymbol(wSPS), powerScaling(1.0),
    loadTest(false)
{
  int i;

  mClock.set(wStartTime);

  for (i = 0; i < mChanM; i++) {
    chanActive[i] = false;
  }
}

RadioInterface::~RadioInterface(void)
{
  int i;

  if (mOn) {
    mRadio->stop();
    close();
 
    delete mAlignRadioServiceLoopThread;

    for (i = 0; i < mChanM; i++) {
      if (rcvBuffer[i] != NULL)
        delete rcvBuffer[i];
      if (sendBuffer[i] != NULL)
        delete sendBuffer[i];
    }
  }
}

double RadioInterface::fullScaleInputValue(void) {
  return mRadio->fullScaleInputValue();
}

double RadioInterface::fullScaleOutputValue(void) {
  return mRadio->fullScaleOutputValue();
}


void RadioInterface::setPowerAttenuation(double atten)
{
  double rfGain, digAtten;

  rfGain = mRadio->setTxGain(mRadio->maxTxGain() - atten);
  digAtten = atten - mRadio->maxTxGain() + rfGain;

  if (digAtten < 1.0)
    powerScaling = 1.0;
  else
    powerScaling = 1.0/sqrt(pow(10, (digAtten/10.0)));
}

int RadioInterface::radioifyVector(signalVector &wVector,
				   float *retVector,
				   float scale,
				   bool zero)
{
  int i;
  signalVector::iterator itr = wVector.begin();

  if (zero) {
    memset(retVector, 0, wVector.size() * 2 * sizeof(float));
    return wVector.size();
  }

  for (i = 0; i < wVector.size(); i++) {
    retVector[2 * i + 0] = itr->real() * scale;
    retVector[2 * i + 1] = itr->imag() * scale;
    itr++;
  }

  return wVector.size();
}

int RadioInterface::unRadioifyVector(float *floatVector, int offset,
				     signalVector &newVector)
{
  int i;
  signalVector::iterator itr = newVector.begin();

  for (i = 0; i < newVector.size(); i++) {
    *itr++ = Complex<float>(floatVector[offset + 2 * i + 0],
			    floatVector[offset + 2 * i + 1]);
  }

  return newVector.size();
}

bool RadioInterface::tuneTx(double freq)
{
  return mRadio->setTxFreq(freq);
}

bool RadioInterface::tuneRx(double freq)
{
  return mRadio->setRxFreq(freq);
}


bool RadioInterface::start()
{
  int i;

  if (mOn)
    return false;

  mOn = true;
#ifdef USRP1
  mAlignRadioServiceLoopThread = new Thread(32768);
  mAlignRadioServiceLoopThread->start((void * (*)(void*))AlignRadioServiceLoopAdapter,
                                      (void*)this);
#endif
  writeTimestamp = mRadio->initialWriteTimestamp();
  readTimestamp = mRadio->initialReadTimestamp();
  for (i = 0; i < mChanM; i++) {
    sendBuffer[i] = new float[8*2*INCHUNK];
    rcvBuffer[i] = new float[8*2*OUTCHUNK];
  }

  /* Init I/O specific variables if applicable */ 
  init();

  mRadio->start(); 
  LOG(DEBUG) << "Radio started";
  mRadio->updateAlignment(writeTimestamp-10000); 
  mRadio->updateAlignment(writeTimestamp-10000);

  return true;
}

bool RadioInterface::stop()
{
  if (!mOn)
    return false;

  mOn = false;
  mRadio->stop();
}

#ifdef USRP1
void *AlignRadioServiceLoopAdapter(RadioInterface *radioInterface)
{
  while (radioInterface->on()) {
    radioInterface->alignRadio();
    pthread_testcancel();
  }
  return NULL;
}

void RadioInterface::alignRadio() {
  sleep(60);
  mRadio->updateAlignment(writeTimestamp+ (TIMESTAMP) 10000);
}
#endif

void RadioInterface::driveTransmitRadio(signalVector **radioBurst, bool *zeroBurst)
{
  int i;

  if (!mOn)
    return;

  for (i = 0; i < mChanM; i++) {
    if (chanActive[i]) {
      radioifyVector(*radioBurst[i], sendBuffer[i] + 2 * sendCursor,
                     powerScaling, zeroBurst[i]);
    }
  }

  /* 
   * All bursts should be the same size since all transceivers are
   * tied with a single clock in the radio interface.
   */
  sendCursor += radioBurst[0]->size();

  pushBuffer();
}

void shiftRxBuffers(float **buf, int offset, int len, int chanM, bool *active)
{
  int i;

  for (i = 0; i < chanM; i++) {
    if (active[i]) {
      memmove(buf[i], buf[i] + offset, sizeof(float) * len);
    }
  }
}

void RadioInterface::loadVectors(unsigned tN, int samplesPerBurst,
                                 int idx, GSM::Time rxClock)
{
  int i;

  for (i = 0; i < mChanM; i++) {
    if (chanActive[i]) {
      signalVector rxVector(samplesPerBurst);
      unRadioifyVector(rcvBuffer[i], idx * 2, rxVector);
      radioVector *rxBurst = new radioVector(rxVector, rxClock);
      mReceiveFIFO[i].write(rxBurst);
    }
  }
}

void RadioInterface::driveReceiveRadio()
{
  if (!mOn)
    return;

  if (mReceiveFIFO[0].size() > 8)
    return;

  pullBuffer();

  GSM::Time rcvClock = mClock.get();
  rcvClock.decTN(receiveOffset);
  unsigned tN = rcvClock.TN();
  int rcvSz = rcvCursor;
  int readSz = 0;
  const int symbolsPerSlot = gSlotLen + 8;
  int samplesPerBurst = (symbolsPerSlot + (tN % 4 == 0)) * samplesPerSymbol;

  // while there's enough data in receive buffer, form received 
  //    GSM bursts and pass up to Transceiver
  // Using the 157-156-156-156 symbols per timeslot format.
  while (rcvSz >= samplesPerBurst) { 
    if (rcvClock.FN() >= 0) {
      loadVectors(tN, samplesPerBurst, readSz, rcvClock);
    }

    mClock.incTN();
    rcvClock.incTN();

    readSz += samplesPerBurst;
    rcvSz -= samplesPerBurst;

    tN = rcvClock.TN();
    samplesPerBurst = (symbolsPerSlot + (tN % 4 == 0)) * samplesPerSymbol;
  }

  if (readSz > 0) {
    rcvCursor -= readSz;
    shiftRxBuffers(rcvBuffer, 2 * readSz, 2 * rcvCursor, mChanM, chanActive);
  }
}

bool RadioInterface::isUnderrun()
{
  bool retVal = underrun;
  underrun = false;

  return retVal;
}

double RadioInterface::setRxGain(double dB)
{
  if (mRadio)
    return mRadio->setRxGain(dB);
  else
    return -1;
}

double RadioInterface::getRxGain()
{
  if (mRadio)
    return mRadio->getRxGain();
  else
    return -1;
}

/* Receive a timestamped chunk from the device */ 
void RadioInterface::pullBuffer()
{
  bool local_underrun;

  /* Read samples. Fail if we don't get what we want. */
  int num_rd = mRadio->readSamples(rx_buf, OUTCHUNK, &overrun,
                                   readTimestamp, &local_underrun);

  LOG(DEBUG) << "Rx read " << num_rd << " samples from device";
  assert(num_rd == OUTCHUNK);

  underrun |= local_underrun;
  readTimestamp += (TIMESTAMP) num_rd;

  shortToFloat(rcvBuffer + 2 * rcvCursor, rx_buf, num_rd);
  rcvCursor += num_rd;
}

/* Send timestamped chunk to the device with arbitrary size */ 
void RadioInterface::pushBuffer()
{
  if (sendCursor < INCHUNK)
    return;

  floatToShort(tx_buf, sendBuffer, sendCursor);

  /* Write samples. Fail if we don't get what we want. */
  int num_smpls = mRadio->writeSamples(tx_buf,
                                       sendCursor,
                                       &underrun,
                                       writeTimestamp);
  assert(num_smpls == sendCursor);

  writeTimestamp += (TIMESTAMP) num_smpls;
  sendCursor = 0;
}
