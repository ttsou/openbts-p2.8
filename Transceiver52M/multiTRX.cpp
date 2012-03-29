/*
 * Copyright 2012  Thomas Tsou <ttsou@vt.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * See the COPYING file in the main directory for details.
 */

#include <time.h>
#include <signal.h>

#include <GSMCommon.h>
#include <Logger.h>
#include <Configuration.h>

#include "Transceiver.h"
#include "radioDevice.h"

#define DEVICERATE (400e3 * CHAN_M)

ConfigurationTable gConfig("/etc/OpenBTS/OpenBTS.db");

volatile bool gbShutdown = false;

static void sigHandler(int signum)
{
	LOG(NOTICE) << "Received shutdown signal";
	gbShutdown = true;
}

static int setupSignals()
{
	struct sigaction action;

	action.sa_handler = sigHandler;
	sigemptyset(&action.sa_mask);
	action.sa_flags = 0;

	if (sigaction(SIGINT, &action, NULL) < 0)
		return -1;

	if (sigaction(SIGTERM, &action, NULL) < 0)
		return -1;

	return 0;
}

/*
 * Generate the channel-transceiver ordering. Channel 0 is always centered
 * at the RF tuning frequecy. Fill remaining channels alternating left and
 * right moving out from the center channel.
 */
static void genChanMap(int *chans)
{
	int i, n;

	chans[0] = 0; 

	for (i = 1, n = 1; i < CHAN_M; i++) {
		if (i % 2) {
			chans[i] = n;
		} else {
			chans[i] = CHAN_M - n;
			n++;
		}
	}
}

static void createTrx(Transceiver **trx, int *map, int num,
		      RadioInterface *radio, DriveLoop *drive)
{
	int i;

	for (i = 0; i < num; i++) {
		LOG(NOTICE) << "Creating TRX" << i
			    << " attached on channel " << map[i];

		radio->activateChan(map[i]);
		trx[i] = new Transceiver(5700 + i * 1000, "127.0.0.1",
					 SAMPSPERSYM, radio, drive, map[i]);
		trx[i]->start();
	}
}

int main(int argc, char *argv[])
{
	int i, numARFCN = 1;
	int chanMap[CHAN_M];
	RadioDevice *usrp;
	RadioInterface* radio;
	DriveLoop *drive;
	Transceiver *trx[CHAN_M];

	gLogInit("transceiver", gConfig.getStr("Log.Level").c_str(), LOG_LOCAL7);

	if (argc > 1) {
		numARFCN = atoi(argv[1]);
		if (numARFCN > CHAN_M) {
			LOG(ALERT) << numARFCN << " channels not supported with current build";
			exit(-1);
		}
	}

	if (setupSignals() < 0) {
		LOG(ERR) << "Failed to setup signal handlers, exiting...";
		exit(-1);
	}

	srandom(time(NULL));
	genChanMap(chanMap);

	usrp = RadioDevice::make(DEVICERATE);
	if (!usrp->open()) {
		LOG(ALERT) << "Failed to open device, exiting...";
		return EXIT_FAILURE;
	}

	radio = new RadioInterface(usrp, numARFCN, SAMPSPERSYM, 0, false);
	drive = new DriveLoop(SAMPSPERSYM, GSM::Time(3,0), radio);

	/* Create, attach, and activate all transceivers */
	createTrx(trx, chanMap, numARFCN, radio, drive);

	while (!gbShutdown) { 
		sleep(1);
	}

	LOG(NOTICE) << "Shutting down transceivers...";
	for (i = 0; i < numARFCN; i++) {
		trx[i]->shutdown();
	}

	/*
	 * Allow time for threads to end before we start freeing objects
	 */
	sleep(2);

	for (i = 0; i < numARFCN; i++) {
		delete trx[i];
	}

	delete drive;
	delete radio;
	delete usrp;
}
