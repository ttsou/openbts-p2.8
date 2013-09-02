/*
* Copyright 2008, 2009, 2010, 2012 Free Software Foundation, Inc.
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


#include <stdio.h>
#include "Transceiver.h"
#include <Logger.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

using namespace GSM;

#define USB_LATENCY_INTRVL		10,0

#if USE_UHD
#  define USB_LATENCY_MIN		6,7
#else
#  define USB_LATENCY_MIN		1,1
#endif

#define INIT_ENERGY_THRSHD		5.0f

Transceiver::Transceiver(int wBasePort, const char *TRXAddress,
			 DriveLoop *wDriveLoop, RadioInterface *wRadioInterface,
			 int wSPS, int wChannel, bool wPrimary)
	:mDataSocket(wBasePort+2,TRXAddress,wBasePort+102),
	 mControlSocket(wBasePort+1,TRXAddress,wBasePort+101),
	 mDriveLoop(wDriveLoop), mRadioInterface(wRadioInterface),
	 mSPS(wSPS), mTransmitPriorityQueue(NULL),
	 mChannel(wChannel), mPrimary(wPrimary)
{
  mFIFOServiceLoopThread = NULL;
  mControlServiceLoopThread = NULL;
  mTransmitPriorityQueueServiceLoopThread = NULL;
  mMaxExpectedDelay = 0;

  mTransmitPriorityQueue = mDriveLoop->priorityQueue(mChannel);
  mReceiveFIFO = mRadioInterface->receiveFIFO(mChannel);

  txFullScale = mRadioInterface->fullScaleInputValue();
  rxFullScale = mRadioInterface->fullScaleOutputValue();

  // initialize per-timeslot variables
  for (int i = 0; i < 8; i++) {
    channelResponse[i] = NULL;
    DFEForward[i] = NULL;
    DFEFeedback[i] = NULL;
    channelEstimateTime[i] = mDriveLoop->getStartTime();
  }

  mOn = false;
  mRunning = false;
  mTxFreq = 0.0;
  mRxFreq = 0.0;
  mFreqOffset = 0.0;

  mPower = -10;
  mEnergyThreshold = INIT_ENERGY_THRSHD;
  prevFalseDetectionTime = mDriveLoop->getStartTime();
}

Transceiver::~Transceiver()
{
  mTransmitPriorityQueue->clear();

  delete mFIFOServiceLoopThread;
  delete mControlServiceLoopThread;
  delete mTransmitPriorityQueueServiceLoopThread;
}
  

void Transceiver::addRadioVector(BitVector &burst,
				 int RSSI,
				 GSM::Time &wTime)
{
  // modulate and stick into queue 
  signalVector* modBurst = modulateBurst(burst,
                                         8 + (wTime.TN() % 4 == 0),
                                         mSPS);
  scaleVector(*modBurst,txFullScale * pow(10,-RSSI/10));
  radioVector *newVec = new radioVector(*modBurst,wTime);
  mTransmitPriorityQueue->write(newVec);

  delete modBurst;
}

SoftVector *Transceiver::pullRadioVector(GSM::Time &wTime,
				      int &RSSI,
				      int &timingOffset)
{
  bool needDFE = (mMaxExpectedDelay > 1);

  radioVector *rxBurst = (radioVector *) mReceiveFIFO->read();

  if (!rxBurst) return NULL;

  LOG(DEBUG) << "receiveFIFO: read radio vector at time: " << rxBurst->getTime() << ", new size: " << mReceiveFIFO->size();

  int timeslot = rxBurst->getTime().TN();

  DriveLoop::CorrType corrType = mDriveLoop->expectedCorrType(mChannel, rxBurst->getTime());

  if ((corrType == DriveLoop::OFF) || (corrType == DriveLoop::IDLE)) {
    delete rxBurst;
    return NULL;
  }
 
  // check to see if received burst has sufficient 
  signalVector *vectorBurst = rxBurst;
  complex amplitude = 0.0;
  float TOA = 0.0;
  float avgPwr = 0.0;

  if (!energyDetect(*vectorBurst,20*mSPS,mEnergyThreshold,&avgPwr)) {
     LOG(DEBUG) << "Estimated Energy: " << sqrt(avgPwr) << ", at time " << rxBurst->getTime();
     double framesElapsed = rxBurst->getTime()-prevFalseDetectionTime;
     if (framesElapsed > 50) {  // if we haven't had any false detections for a while, lower threshold
	mEnergyThreshold -= 10.0/10.0;
        if (mEnergyThreshold < 0.0)
          mEnergyThreshold = 0.0;

        prevFalseDetectionTime = rxBurst->getTime();
     }
     delete rxBurst;
     return NULL;
  }
  LOG(DEBUG) << "Estimated Energy: " << sqrt(avgPwr) << ", at time " << rxBurst->getTime();

  // run the proper correlator
  bool success = false;
  if (corrType == DriveLoop::TSC) {
    LOG(DEBUG) << "looking for TSC at time: " << rxBurst->getTime();

    signalVector *channelResp;
    double framesElapsed = rxBurst->getTime()-channelEstimateTime[timeslot];
    bool estimateChannel = false;
    if ((framesElapsed > 50) || (channelResponse[timeslot]==NULL)) {
	if (channelResponse[timeslot]) delete channelResponse[timeslot];
        if (DFEForward[timeslot]) delete DFEForward[timeslot];
        if (DFEFeedback[timeslot]) delete DFEFeedback[timeslot];
        channelResponse[timeslot] = NULL;
        DFEForward[timeslot] = NULL;
        DFEFeedback[timeslot] = NULL;
	estimateChannel = true;
    }
    if (!needDFE) estimateChannel = false;
    float chanOffset;
    success = analyzeTrafficBurst(*vectorBurst,
				  mTSC,
				  3.0,
				  mSPS,
				  &amplitude,
				  &TOA,
				  mMaxExpectedDelay, 
				  estimateChannel,
				  &channelResp,
				  &chanOffset);
    if (success) {
      LOG(DEBUG) << "FOUND TSC!!!!!! " << amplitude << " " << TOA;
      mEnergyThreshold -= 1.0F/10.0F;
      if (mEnergyThreshold < 0.0) mEnergyThreshold = 0.0;
      SNRestimate[timeslot] = amplitude.norm2()/(mEnergyThreshold*mEnergyThreshold+1.0); // this is not highly accurate
      if (estimateChannel) {
         LOG(DEBUG) << "estimating channel...";
         channelResponse[timeslot] = channelResp;
       	 chanRespOffset[timeslot] = chanOffset;
         chanRespAmplitude[timeslot] = amplitude;
	 scaleVector(*channelResp, complex(1.0,0.0)/amplitude);
         designDFE(*channelResp, SNRestimate[timeslot], 7, &DFEForward[timeslot], &DFEFeedback[timeslot]);
         channelEstimateTime[timeslot] = rxBurst->getTime();  
         LOG(DEBUG) << "SNR: " << SNRestimate[timeslot] << ", DFE forward: " << *DFEForward[timeslot] << ", DFE backward: " << *DFEFeedback[timeslot];
      }
    }
    else {
      double framesElapsed = rxBurst->getTime()-prevFalseDetectionTime; 
      LOG(DEBUG) << "wTime: " << rxBurst->getTime() << ", pTime: " << prevFalseDetectionTime << ", fElapsed: " << framesElapsed;
      mEnergyThreshold += 10.0F/10.0F*exp(-framesElapsed);
      prevFalseDetectionTime = rxBurst->getTime();
      channelResponse[timeslot] = NULL;
    }
  }
  else {
    // RACH burst
    success = detectRACHBurst(*vectorBurst,
			      5.0,  // detection threshold
			      mSPS,
			      &amplitude,
			      &TOA);
    if (success) {
      LOG(DEBUG) << "FOUND RACH!!!!!! " << amplitude << " " << TOA;
      mEnergyThreshold -= (1.0F/10.0F);
      if (mEnergyThreshold < 0.0) mEnergyThreshold = 0.0;
      channelResponse[timeslot] = NULL; 
    }
    else {
      double framesElapsed = rxBurst->getTime()-prevFalseDetectionTime;
      mEnergyThreshold += (1.0F/10.0F)*exp(-framesElapsed);
      prevFalseDetectionTime = rxBurst->getTime();
    }
  }
  LOG(DEBUG) << "energy Threshold = " << mEnergyThreshold; 

  // demodulate burst
  SoftVector *burst = NULL;
  if ((rxBurst) && (success)) {
    if ((corrType == DriveLoop::RACH) || (!needDFE)) {
      burst = demodulateBurst(*vectorBurst,
			      mSPS,
			      amplitude,TOA);
    }
    else { // TSC
      scaleVector(*vectorBurst,complex(1.0,0.0)/amplitude);
      burst = equalizeBurst(*vectorBurst,
			    TOA-chanRespOffset[timeslot],
			    mSPS,
			    *DFEForward[timeslot],
			    *DFEFeedback[timeslot]);
    }
    wTime = rxBurst->getTime();
    RSSI = (int) floor(20.0*log10(rxFullScale/amplitude.abs()));
    LOG(DEBUG) << "RSSI: " << RSSI;
    timingOffset = (int) round(TOA*256.0/mSPS);
  }

  //if (burst) LOG(DEBUG) << "burst: " << *burst << '\n';

  delete rxBurst;

  return burst;
}

void Transceiver::pullFIFO()
{
  SoftVector *rxBurst = NULL;
  int RSSI;
  int TOA;  // in 1/256 of a symbol
  GSM::Time burstTime;

  rxBurst = pullRadioVector(burstTime,RSSI,TOA);

  if (rxBurst) {
    LOG(DEBUG) << "burst parameters: "
               << " time: " << burstTime
               << " RSSI: " << RSSI
               << " TOA: "  << TOA
               << " bits: " << *rxBurst;

    char burstString[gSlotLen+10];
    burstString[0] = burstTime.TN();
    for (int i = 0; i < 4; i++) {
            burstString[1+i] = (burstTime.FN() >> ((3-i)*8)) & 0x0ff;
    }

    burstString[5] = RSSI;
    burstString[6] = (TOA >> 8) & 0x0ff;
    burstString[7] = TOA & 0x0ff;
    SoftVector::iterator burstItr = rxBurst->begin();

    for (unsigned int i = 0; i < gSlotLen; i++) {
            burstString[8+i] =(char) round((*burstItr++)*255.0);
    }

    burstString[gSlotLen+9] = '\0';
    delete rxBurst;

    mDataSocket.write(burstString,gSlotLen+10);
  }
}

void Transceiver::start()
{
  mRunning = true;
  mControlServiceLoopThread = new Thread(32768);
  mControlServiceLoopThread->start((void * (*)(void*))ControlServiceLoopAdapter,(void*) this);

  if (!mPrimary) {
    mOn = true;
    mFIFOServiceLoopThread = new Thread(32768);
    mFIFOServiceLoopThread->start((void * (*)(void*))FIFOServiceLoopAdapter,(void*) this);
 
    mTransmitPriorityQueueServiceLoopThread = new Thread(32768);
    mTransmitPriorityQueueServiceLoopThread->start((void * (*)(void*))TransmitPriorityQueueServiceLoopAdapter,(void*) this);
  }
}

void Transceiver::shutdown()
{
  mOn = false;
  mRunning = false;
}

void Transceiver::reset()
{
  mTransmitPriorityQueue->clear();
}

  
void Transceiver::driveControl()
{

  int MAX_PACKET_LENGTH = 100;

  // check control socket
  char buffer[MAX_PACKET_LENGTH];
  int msgLen = -1;
  buffer[0] = '\0';

  try { 
    msgLen = mControlSocket.read(buffer);
    if (msgLen < 1) {
      return;
    }
  } catch (...) {
    /* Ignore the read exception on shutdown */
    if (!mRunning) {
      return;
    }

    LOG(ALERT) << "Caught UHD socket exception";
    return;
  }

  char cmdcheck[4];
  char command[MAX_PACKET_LENGTH];
  char response[MAX_PACKET_LENGTH];

  sscanf(buffer,"%3s %s",cmdcheck,command);

  mDriveLoop->writeClockInterface();

  if (strcmp(cmdcheck,"CMD")!=0) {
    LOG(WARNING) << "bogus message on control interface";
    return;
  }
  LOG(INFO) << "command is " << buffer;

  if (strcmp(command,"POWEROFF")==0) {
    // turn off transmitter/demod
    sprintf(response,"RSP POWEROFF 0"); 
  }
  else if (strcmp(command,"POWERON")==0) {
    // turn on transmitter/demod
    if (!mTxFreq || !mRxFreq || (mTSC<0))
      sprintf(response,"RSP POWERON 1");
    else {
      sprintf(response,"RSP POWERON 0");
      if (mPrimary && !mOn) {
        // Prepare for thread start
        mPower = -20;
        mRadioInterface->start();
        mDriveLoop->start();

        mDriveLoop->writeClockInterface();
        generateRACHSequence(mSPS);

        // Start radio interface threads.
        mOn = true;
        mFIFOServiceLoopThread = new Thread(32768);
        mFIFOServiceLoopThread->start((void * (*)(void*))FIFOServiceLoopAdapter,(void*) this);

        mTransmitPriorityQueueServiceLoopThread = new Thread(32768);
        mTransmitPriorityQueueServiceLoopThread->start((void * (*)(void*))TransmitPriorityQueueServiceLoopAdapter,(void*) this);
      }
    }
  }
  else if (strcmp(command,"SETMAXDLY")==0) {
    //set expected maximum time-of-arrival
    int maxDelay;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&maxDelay);
    mMaxExpectedDelay = maxDelay; // 1 GSM symbol is approx. 1 km
    sprintf(response,"RSP SETMAXDLY 0 %d",maxDelay);
  }
  else if (strcmp(command,"SETRXGAIN")==0) {
    //set expected maximum time-of-arrival
    int newGain;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&newGain);
    mEnergyThreshold = INIT_ENERGY_THRSHD;
    newGain = mRadioInterface->setRxGain(newGain, mChannel);
    sprintf(response,"RSP SETRXGAIN 0 %d",newGain);
  }
  else if (strcmp(command,"NOISELEV")==0) {
    if (mOn) {
      sprintf(response,"RSP NOISELEV 0 %d",
              (int) round(20.0*log10(rxFullScale/mEnergyThreshold)));
    }
    else {
      sprintf(response,"RSP NOISELEV 1  0");
    }
  }   
  else if (strcmp(command,"SETPOWER")==0) {
    // set output power in dB
    int dbPwr;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&dbPwr);
    if (!mOn) 
      sprintf(response,"RSP SETPOWER 1 %d",dbPwr);
    else {
      mPower = dbPwr;
      mRadioInterface->setPowerAttenuation(dbPwr, mChannel);
      sprintf(response,"RSP SETPOWER 0 %d",dbPwr);
    }
  }
  else if (strcmp(command,"ADJPOWER")==0) {
    // adjust power in dB steps
    int dbStep;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&dbStep);
    if (!mOn) 
      sprintf(response,"RSP ADJPOWER 1 %d",mPower);
    else {
      mPower += dbStep;
      sprintf(response,"RSP ADJPOWER 0 %d",mPower);
    }
  }
  else if (strcmp(command,"RXTUNE")==0) {
    // tune receiver
    int freqKhz;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freqKhz);
    mRxFreq = freqKhz * 1.0e3 + mFreqOffset;
    if (!mRadioInterface->tuneRx(mRxFreq, mChannel)) {
      LOG(ALERT) << "RX failed to tune";
      sprintf(response,"RSP RXTUNE 1 %d",freqKhz);
    } else {
      sprintf(response,"RSP RXTUNE 0 %d",freqKhz);
    }
  }
  else if (strcmp(command,"TXTUNE")==0) {
    // tune txmtr
    int freqKhz;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&freqKhz);
    //freqKhz = 890e3;
    mTxFreq = freqKhz * 1.0e3 + mFreqOffset;
    if (!mRadioInterface->tuneTx(mTxFreq, mChannel)) {
      LOG(ALERT) << "TX failed to tune";
      sprintf(response,"RSP TXTUNE 1 %d",freqKhz);
    } else {
      sprintf(response,"RSP TXTUNE 0 %d",freqKhz);
    }
  }
  else if (strcmp(command,"SETTSC")==0) {
    // set TSC
    int TSC;
    sscanf(buffer,"%3s %s %d",cmdcheck,command,&TSC);
    if (mOn || (TSC<0) || (TSC>7))
      sprintf(response,"RSP SETTSC 1 %d",TSC);
    else {
      mTSC = TSC;
      generateMidamble(mSPS, TSC);
      sprintf(response,"RSP SETTSC 0 %d",TSC);
    }
  }
  else if (strcmp(command,"SETSLOT")==0) {
    // set slot type
    int  corrCode;
    int  timeslot;
    sscanf(buffer,"%3s %s %d %d",cmdcheck,command,&timeslot,&corrCode);
    if ((timeslot < 0) || (timeslot > 7)) {
      LOG(WARNING) << "bogus message on control interface";
      sprintf(response,"RSP SETSLOT 1 %d %d",timeslot,corrCode);
      return;
    }     
    mDriveLoop->setTimeslot(mChannel, timeslot, (DriveLoop::ChannelCombination) corrCode);
    mDriveLoop->setModulus(mChannel, timeslot);
    sprintf(response,"RSP SETSLOT 0 %d %d",timeslot,corrCode);

  }
  else {
    LOG(WARNING) << "bogus command " << command << " on control interface.";
    sprintf(response,"RSP ERR 1");
  }

  mControlSocket.write(response,strlen(response)+1);

}

bool Transceiver::driveTransmitPriorityQueue() 
{
  char buffer[gSlotLen+50];

  if (!mOn)
    return true;

  try { 
    size_t msgLen = mDataSocket.read(buffer);
    if (msgLen!=gSlotLen+1+4+1) {
      LOG(ERR) << "badly formatted packet on GSM->TRX interface";
      return false;
    }
  } catch (...) {
    if (!mOn) {
      /* Shutdown condition. End the thread. */
      return true;
    }

    LOG(ALERT) << "Caught UHD socket exception";
    return false;
  }

  int timeSlot = (int) buffer[0];
  uint64_t frameNum = 0;
  for (int i = 0; i < 4; i++)
    frameNum = (frameNum << 8) | (0x0ff & buffer[i+1]);
  
  // periodically update GSM core clock
  LOG(DEBUG) << "mTransmitDeadlineClock " << mDriveLoop->getDeadlineClock()
             << " mLastClockUpdateTime " << mDriveLoop->getLastClockUpdate();
  if (mDriveLoop->getDeadlineClock() > mDriveLoop->getLastClockUpdate() + GSM::Time(216,0)) {
    mDriveLoop->writeClockInterface();
  }

  LOG(DEBUG) << "rcvd. burst at: " << GSM::Time(frameNum,timeSlot);
  
  int RSSI = (int) buffer[5];
  static BitVector newBurst(gSlotLen);
  BitVector::iterator itr = newBurst.begin();
  char *bufferItr = buffer+6;
  while (itr < newBurst.end()) 
    *itr++ = *bufferItr++;
  
  GSM::Time currTime = GSM::Time(frameNum,timeSlot);
  
  addRadioVector(newBurst,RSSI,currTime);
  
  LOG(DEBUG) "added burst - time: " << currTime << ", RSSI: " << RSSI; // << ", data: " << newBurst; 

  return true;


}

void *FIFOServiceLoopAdapter(Transceiver *transceiver)
{
  while (transceiver->on()) {
    transceiver->pullFIFO();
    pthread_testcancel();
  }
  return NULL;
}

void *ControlServiceLoopAdapter(Transceiver *transceiver)
{
  while (transceiver->running()) {
    transceiver->driveControl();
    pthread_testcancel();
  }
  return NULL;
}

void *TransmitPriorityQueueServiceLoopAdapter(Transceiver *transceiver)
{
  while (transceiver->on()) {
    bool stale = false;

    // Flush the UDP packets until a successful transfer.
    while (!transceiver->driveTransmitPriorityQueue()) {
      stale = true; 
    }
    if (stale) {
      // If a packet was stale, remind the GSM stack of the clock.
      transceiver->getDriveLoop()->writeClockInterface();
    }
    pthread_testcancel();
  }
  return NULL;
}
