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

int main(int argc, char *argv[])
{
	RadioDevice *usrp;
	RadioInterface* radio;
	DriveLoop *drive;
	Transceiver *trx0, *trx1, *trx2;

	gLogInit("transceiver", gConfig.getStr("Log.Level").c_str(), LOG_LOCAL7);

	if (setupSignals() < 0) {
		LOG(ERR) << "Failed to setup signal handlers, exiting...";
		exit(-1);
	}

	srandom(time(NULL));

	usrp = RadioDevice::make(DEVICERATE);
	if (!usrp->open()) {
		LOG(ALERT) << "Failed to open device, exiting...";
		return EXIT_FAILURE;
	}

	radio = new RadioInterface(usrp, 3, SAMPSPERSYM, 0, false);
	drive = new DriveLoop(SAMPSPERSYM, GSM::Time(3,0), radio);

	LOG(NOTICE) << "Creating TRX0";
	trx0 = new Transceiver(5700, "127.0.0.1", SAMPSPERSYM, radio, drive, 0);
	trx0->receiveFIFO(radio->receiveFIFO(0));
	trx0->transmitQueue(drive->priorityQueue(0));
	radio->activateChan(0);
	trx0->start();

	LOG(NOTICE) << "Creating TRX1";
	trx1 = new Transceiver(6700, "127.0.0.1", SAMPSPERSYM, radio, drive, 1);
	trx1->receiveFIFO(radio->receiveFIFO(1));
	trx1->transmitQueue(drive->priorityQueue(1));
	radio->activateChan(1);
	trx1->start();

	LOG(NOTICE) << "Creating TRX2";
	trx2 = new Transceiver(7700, "127.0.0.1", SAMPSPERSYM, radio, drive, 4);
	trx2->receiveFIFO(radio->receiveFIFO(4));
	trx2->transmitQueue(drive->priorityQueue(4));
	radio->activateChan(4);
	trx2->start();

	while (!gbShutdown) { 
		sleep(1);
	}

	LOG(NOTICE) << "Shutting down transceiver...";

	delete trx0;
	delete trx1;
	delete trx2;
	delete drive;
	delete radio;
	delete usrp;
}
