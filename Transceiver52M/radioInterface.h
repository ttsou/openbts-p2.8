/*
* Copyright 2008, 2012 Free Software Foundation, Inc.
*
* This software is distributed under multiple licenses; see the COPYING file in the main directory for licensing information for this specific distribuion.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*/

#ifndef _RADIOINTEFACE_H_
#define _RADIOINTEFACE_H_

#include "sigProcLib.h"  
#include "GSMCommon.h"
#include "LinkedLists.h"
#include "radioDevice.h"
#include "radioVector.h"
#include "radioClock.h"

/** samples per GSM symbol */
#define SAMPSPERSYM 1
#define INCHUNK    (625)
#define OUTCHUNK   (625)
#define CHAN_MAX    2


static const unsigned gSlotLen = 148;      ///< number of symbols per slot, not counting guard periods

/** class to interface the transceiver with the USRP */
class RadioInterface {

protected:

  int mChanM;                                 ///< channelizer width
  Thread *mAlignRadioServiceLoopThread;	      ///< thread that synchronizes transmit and receive sections

  VectorFIFO mReceiveFIFO[CHAN_MAX];	      ///< FIFO that holds receive  bursts

  RadioDevice *mRadio;			      ///< the USRP object
 
  float *sendBuffer[CHAN_MAX];
  unsigned sendCursor;

  float *rcvBuffer[CHAN_MAX];
  unsigned rcvCursor;

  bool underrun;			      ///< indicates writes to USRP are too slow
  bool overrun;				      ///< indicates reads from USRP are too slow
  TIMESTAMP writeTimestamp;		      ///< sample timestamp of next packet written to USRP
  TIMESTAMP readTimestamp;		      ///< sample timestamp of next packet read from USRP

  RadioClock mClock;                          ///< the basestation clock!

  int samplesPerSymbol;			      ///< samples per GSM symbol
  int receiveOffset;                          ///< offset b/w transmit and receive GSM timestamps, in timeslots

  bool mOn;				      ///< indicates radio is on

  double powerScaling;

  bool loadTest;
  int mNumARFCNs;
  signalVector *finalVec, *finalVec9;

private:
  /** initialize I/O internals */
  bool init();

  /** format samples to USRP */ 
  int radioifyVector(signalVector &wVector,
                     float *floatVector,
                     float scale,
                     bool zero);

  /** format samples from USRP */
  int unRadioifyVector(float *floatVector, int offset, signalVector &wVector);

  /** push GSM bursts into the transmit buffer */
  virtual void pushBuffer(void);

  /** pull GSM bursts from the receive buffer */
  virtual void pullBuffer(void);

  /** load receive vectors into FIFO's */
  void loadVectors(unsigned tN, int samplesPerBurst, int index, GSM::Time rxClock);

public:

  /** start the interface */
  bool start();
  bool stop();

  bool started() { return mOn; };

  /** shutdown interface */
  void close();

  /** constructor */
  RadioInterface(RadioDevice* wRadio,
		 int wChanM = 1,
		 int wSPS = SAMPSPERSYM,
                 int receiveOffset = 3,
		 GSM::Time wStartTime = GSM::Time(0, 0));
    
  /** destructor */
  ~RadioInterface();

  void setSamplesPerSymbol(int wSamplesPerSymbol) {if (!mOn) samplesPerSymbol = wSamplesPerSymbol;}

  int getSamplesPerSymbol() { return samplesPerSymbol;}
 
  /** return the receive FIFO */
  VectorFIFO* receiveFIFO(int num) { return &mReceiveFIFO[num];}

  /** return the basestation clock */
  RadioClock* getClock(void) { return &mClock;};

  /** set transmit frequency */
  bool tuneTx(double freq, int chan = 0);

  /** set receive frequency */
  bool tuneRx(double freq, int chan = 0);

  /** set receive gain */
  double setRxGain(double dB, int chan = 0);

  /** get receive gain */
  double getRxGain(int chan = 0);

  /** drive transmission of GSM bursts */
  void driveTransmitRadio(signalVector **radioBurst, bool *zeroBurst);

  /** drive reception of GSM bursts */
  void driveReceiveRadio();

  void setPowerAttenuation(double atten, int chan = 0);

  /** returns the full-scale transmit amplitude **/
  double fullScaleInputValue();

  /** returns the full-scale receive amplitude **/
  double fullScaleOutputValue();

  /** set thread priority on current thread */
  void setPriority() { mRadio->setPriority(); }

  /** get transport window type of attached device */ 
  enum RadioDevice::TxWindowType getWindowType() { return mRadio->getWindowType(); }
};

#endif /* _RADIOINTEFACE_H_ */
