/*
* Copyright 2008, 2012 Free Software Foundation, Inc.
*
* This software is distributed under the terms of the GNU Public License.
* See the COPYING file in the main directory for details.
*
* This use of this software may be subject to additional restrictions.
* See the LEGAL file in the main directory for details.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/



/*
	Compilation switches
	TRANSMIT_LOGGING	write every burst on the given slot to a log
*/

#ifndef _DRIVELOOP_H_
#define _DRIVELOOP_H_

#include "radioInterface.h"
#include "Interthread.h"
#include "GSMCommon.h"
#include "Sockets.h"

#include <sys/types.h>
#include <sys/socket.h>

/** Define this to be the slot number to be logged. */
//#define TRANSMIT_LOGGING 1

/** The Transceiver class, responsible for physical layer of basestation */
class DriveLoop {
  
private:

  GSM::Time mTransmitLatency;     ///< latency between basestation clock and transmit deadline clock
  GSM::Time mLatencyUpdateTime;   ///< last time latency was updated

  UDPSocket mClockSocket;	  ///< socket for writing clock updates to GSM core

  VectorQueue  mTransmitPriorityQueue[CHAN_M];   ///< priority queue of transmit bursts received from GSM core

  Thread *mRadioDriveLoopThread;  ///< thread to push/pull bursts into transmit/receive FIFO

  GSM::Time mTransmitDeadlineClock;       ///< deadline for pushing bursts into transmit FIFO 
  GSM::Time mStartTime;                   ///< random start time of the radio clock

  RadioInterface *mRadioInterface;	  ///< associated radioInterface object
  double txFullScale;                     ///< full scale input to radio
  double rxFullScale;                     ///< full scale output to radio


  /** unmodulate a modulated burst */
#ifdef TRANSMIT_LOGGING
  void unModulateVector(signalVector wVector); 
#endif

  /** modulate and add a burst to the transmit queue */
  void addRadioVector(BitVector &burst, int RSSI, GSM::Time &wTime);

  /** Push modulated burst into transmit FIFO corresponding to a particular timestamp */
  void pushRadioVector(GSM::Time &nowTime);

  /** Pull and demodulate a burst from the receive FIFO */ 
  SoftVector *pullRadioVector(GSM::Time &wTime, int &RSSI, int &timingOffset);
   
  /** send messages over the clock socket */
  void writeClockInterface(void);

  signalVector *gsmPulse;              ///< the GSM shaping pulse for modulation

  int mSamplesPerSymbol;               ///< number of samples per GSM symbol

  bool mOn;			       ///< flag to indicate that transceiver is powered on
  int fillerModulus[CHAN_M][8];                ///< modulus values of all timeslots, in frames
  signalVector *fillerTable[CHAN_M][102][8];   ///< table of modulated filler waveforms for all timeslots

  signalVector *mTxBursts[CHAN_M];
  bool         mIsFiller[CHAN_M];
  bool         mIsZero[CHAN_M];

public:

  /** Transceiver constructor 
      @param wBasePort base port number of UDP sockets
      @param TRXAddress IP address of the TRX manager, as a string
      @param wSamplesPerSymbol number of samples per GSM symbol
      @param wTransmitLatency initial setting of transmit latency
      @param radioInterface associated radioInterface object
  */
  DriveLoop(int wSamplesPerSymbol,
	    GSM::Time wTransmitLatency,
	    RadioInterface *wRadioInterface);
   
  /** Destructor */
  ~DriveLoop();

  /** start the Transceiver */
  void start();

  VectorQueue *priorityQueue(int m) { return &mTransmitPriorityQueue[m]; }

  GSM::Time *deadlineClock() { return &mTransmitDeadlineClock; }

  /** Codes for burst types of received bursts*/
  typedef enum {
    OFF,               ///< timeslot is off
    TSC,	       ///< timeslot should contain a normal burst
    RACH,	       ///< timeslot should contain an access burst
    IDLE	       ///< timeslot is an idle (or dummy) burst
  } CorrType;

  /** Codes for channel combinations */
  typedef enum {
    FILL,               ///< Channel is transmitted, but unused
    I,                  ///< TCH/FS
    II,                 ///< TCH/HS, idle every other slot
    III,                ///< TCH/HS
    IV,                 ///< FCCH+SCH+CCCH+BCCH, uplink RACH
    V,                  ///< FCCH+SCH+CCCH+BCCH+SDCCH/4+SACCH/4, uplink RACH+SDCCH/4
    VI,                 ///< CCCH+BCCH, uplink RACH
    VII,                ///< SDCCH/8 + SACCH/8
    NONE,               ///< Channel is inactive, default
    LOOPBACK            ///< similar go VII, used in loopback testing
  } ChannelCombination;

  /** Set modulus for specific timeslot */
  void setModulus(int channel, int timeslot);

  /** return the expected burst type for the specified timestamp */
  CorrType expectedCorrType(int channel, GSM::Time currTime);

  void setTimeslot(int m, int timeslot, ChannelCombination comb)
  {
    mChanType[m][timeslot] = comb;
  }

  GSM::Time getStartTime() { return mStartTime; }

private:

  ChannelCombination mChanType[CHAN_M][8];     ///< channel types for all timeslots

protected:

  /** drive reception and demodulation of GSM bursts */ 
  void driveReceiveFIFO();

  /** drive transmission of GSM bursts */
  void driveTransmitFIFO();

  /** drive handling of control messages from GSM core */
  void driveControl();

  /**
    drive modulation and sorting of GSM bursts from GSM core
    @return true if a burst was transferred successfully
  */
  bool driveTransmitPriorityQueue();

  friend void *RadioDriveLoopAdapter(DriveLoop *);

  void reset();

  /** return drive loop status */
  bool on() { return mOn; }

  /** set priority on current thread */
  void setPriority() { mRadioInterface->setPriority(); }

};

/** FIFO thread loop */
void *RadioDriveLoopAdapter(DriveLoop *);

#endif /* _DRIVELOOP_H_ */
