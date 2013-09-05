/*
* Copyright 2008, 2009, 2010 Free Software Foundation, Inc.
* Copyright 2010 Kestrel Signal Processing, Inc.
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



#include "Transceiver.h"
#include "radioDevice.h"
#include "DummyLoad.h"
#include <fstream>

#include <time.h>
#include <signal.h>

#include <GSMCommon.h>
#include <Logger.h>
#include <Configuration.h>

#define CONFIGDB            "/etc/OpenBTS/OpenBTS.db"

using namespace std;

ConfigurationTable gConfig(CONFIGDB);

volatile bool gbShutdown = false;

static void ctrlCHandler(int signo)
{
   cout << "Received shutdown signal" << endl;;
   gbShutdown = true;
}

/*
 * Attempt to open and test the database file before
 * accessing the configuration table. We do this because
 * the global table constructor cannot provide notification
 * in the event of failure.
 */
int testConfig(const char *filename)
{
  int rc, val = 9999;
  bool status;
  sqlite3 *db;
  std::string result;
  std::string test = "sadf732zdvj2";

  const char *keys[3] = {
    "Log.Level",
    "TRX.Port",
    "TRX.IP",
  };

  /* Check for file existence */
  std::ifstream file(filename);
  if (!file.good()) {
    std::cerr << "Config: File not readable \""
              << filename << "\"" << std::endl;
    return -1;
  } else {
    file.close();
  }

  /* Try to open the database  */
  rc = sqlite3_open(filename, &db);
  if (rc || !db) {
    std::cerr << "Config: Database could not be opened" << std::endl;
    return -1;
  } else {
    sqlite3_close(db);
  }

  /* Attempt to set a value in the global config */
  if (!gConfig.set(test, val)) {
    std::cerr << "Config: Failed to set test key - "
              << "permission to access the database?" << std::endl;
    return -1;
  } else {
    gConfig.remove(test);
  }

  /* Attempt to query */
  for (int i = 0; i < 3; i++) {
    try {
      result = gConfig.getStr(keys[i]); 
    } catch (...) {
      std::cerr << "Config: Failed query on " << keys[i] << std::endl;
      return -1;
    }
  }

  return 0; 
}

int main(int argc, char *argv[])
{
  int trxPort;
  std::string deviceArgs, logLevel, trxAddr, txAntenna, rxAntenna;

  RadioDevice *usrp;
  RadioInterface* radio;
  DriveLoop *drive;
  Transceiver *trx;

  if (argc == 3)
  {
    deviceArgs = std::string(argv[2]);
  }
  else
  {
    deviceArgs = "";
  }

  if ( signal( SIGINT, ctrlCHandler ) == SIG_ERR )
  {
    cerr << "Couldn't install signal handler for SIGINT" << endl;
    exit(1);
  }

  if ( signal( SIGTERM, ctrlCHandler ) == SIG_ERR )
  {
    cerr << "Couldn't install signal handler for SIGTERM" << endl;
    exit(1);
  }

  // Configure logger.
  if (testConfig(CONFIGDB) < 0) {
    std::cerr << "Config: Database failure" << std::endl;
    return EXIT_FAILURE;
  }

  logLevel = gConfig.getStr("Log.Level");
  trxPort = gConfig.getNum("TRX.Port");
  trxAddr = gConfig.getStr("TRX.IP");
  gLogInit("transceiver", logLevel.c_str(), LOG_LOCAL7);

  if (gConfig.defines("GSM.Radio.TxAntenna"))
    txAntenna = gConfig.getStr("GSM.Radio.TxAntenna").c_str();
  if (gConfig.defines("GSM.Radio.RxAntenna"))
    rxAntenna = gConfig.getStr("GSM.Radio.RxAntenna").c_str();

  if (txAntenna != "")  
    usrp->setTxAntenna(txAntenna);
  if (rxAntenna != "")  
    usrp->setRxAntenna(rxAntenna);

  LOG(INFO) << "transceiver using transmit antenna " << usrp->getRxAntenna();
  LOG(INFO) << "transceiver using receive antenna " << usrp->getTxAntenna();

  srandom(time(NULL));

  usrp = RadioDevice::make(SAMPSPERSYM);
  int radioType = usrp->open(deviceArgs);
  if (radioType < 0) {
    LOG(ALERT) << "Transceiver exiting..." << std::endl;
    return EXIT_FAILURE;
  }

  switch (radioType) {
  case RadioDevice::NORMAL:
    radio = new RadioInterface(usrp, 3, SAMPSPERSYM, false);
    break;
  case RadioDevice::RESAMP:
  default:
    LOG(ALERT) << "Unsupported configuration";
    return EXIT_FAILURE;
  }

  drive = new DriveLoop(SAMPSPERSYM, GSM::Time(3,0), radio);
  if (!drive->init()) {
    LOG(ALERT) << "Failed to initialize drive loop";
  }

  trx = new Transceiver(trxPort, trxAddr.c_str(), SAMPSPERSYM, radio, drive, 0);
  radio->activateChan(0);
  if (!trx->init()) {
    LOG(ALERT) << "Failed to initialize transceiver";
  }

/*
  signalVector *gsmPulse = generateGSMPulse(2,1);
  BitVector normalBurstSeg = "0000101010100111110010101010010110101110011000111001101010000";
  BitVector normalBurst(BitVector(normalBurstSeg,gTrainingSequence[0]),normalBurstSeg);
  signalVector *modBurst = modulateBurst(normalBurst,*gsmPulse,8,1);
  signalVector *modBurst9 = modulateBurst(normalBurst,*gsmPulse,9,1);
  signalVector *interpolationFilter = createLPF(0.6/mOversamplingRate,6*mOversamplingRate,1);
  signalVector totalBurst1(*modBurst,*modBurst9);
  signalVector totalBurst2(*modBurst,*modBurst);
  signalVector totalBurst(totalBurst1,totalBurst2);
  scaleVector(totalBurst,usrp->fullScaleInputValue());
  double beaconFreq = -1.0*(numARFCN-1)*200e3;
  signalVector finalVec(625*mOversamplingRate);
  for (int j = 0; j < numARFCN; j++) {
	signalVector *frequencyShifter = new signalVector(625*mOversamplingRate);
	frequencyShifter->fill(1.0);
	frequencyShift(frequencyShifter,frequencyShifter,2.0*M_PI*(beaconFreq+j*400e3)/(1625.0e3/6.0*mOversamplingRate));
  	signalVector *interpVec = polyphaseResampleVector(totalBurst,mOversamplingRate,1,interpolationFilter);
	multVector(*interpVec,*frequencyShifter);
	addVector(finalVec,*interpVec); 	
  }
  signalVector::iterator itr = finalVec.begin();
  short finalVecShort[2*finalVec.size()];
  short *shortItr = finalVecShort;
  while (itr < finalVec.end()) {
	*shortItr++ = (short) (itr->real());
	*shortItr++ = (short) (itr->imag());
	itr++;
  }
  usrp->loadBurst(finalVecShort,finalVec.size());
*/
  trx->start();

  while(!gbShutdown) { sleep(1); }//i++; if (i==60) break;}

  cout << "Shutting down transceiver..." << endl;
  trx->shutdown();

  delete trx;
  delete drive;
  delete radio;
}
