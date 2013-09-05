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

ConfigurationTable gConfig("/etc/OpenBTS/OpenBTS.db");

volatile bool gbShutdown = false;

int Transceiver::mTSC = -1;

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
	int numARFCN = 1;
	std::string deviceArgs = "";

	switch (argc) {
	case 3:
		deviceArgs = std::string(argv[2]);
	case 2:
		numARFCN = atoi(argv[1]);
		if (numARFCN > CHAN_MAX) {
			LOG(ALERT) << numARFCN  << " channels not supported "
						<< " with with current build";
			exit(-1);
		}
	case 1:
		break;
	default:
		std::cout << argv[0] << " <chans> <device args>" << std::endl;
		return -1;
	}

	gLogInit("transceiver", gConfig.getStr("Log.Level").c_str(), LOG_LOCAL7);
	srandom(time(NULL));

	if (setupSignals() < 0) {
		LOG(ERR) << "Failed to setup signal handlers, exiting...";
		exit(-1);
	}

	RadioDevice *device = RadioDevice::make(SAMPSPERSYM);
	int radioType = device->open(deviceArgs);
	if (radioType < 0) {
		LOG(ALERT) << "Failed to open device, exiting...";
		return EXIT_FAILURE;
	}

	RadioInterface *radio;
	switch (radioType) {
	case RadioDevice::NORMAL:
		radio = new RadioInterface(device, numARFCN);
		break;
	case RadioDevice::RESAMP:
	default:
		LOG(ALERT) << "Unsupported configuration";
		return EXIT_FAILURE;
	}

	DriveLoop *drive;
	drive = new DriveLoop(5700, "127.0.0.1", radio, numARFCN, 0);
	if (!drive->init()) {
		LOG(ALERT) << "Failed to initialize drive loop";
	}

	Transceiver *trx[CHAN_MAX];
	bool primary = true;
	for (int i = 0; i < numARFCN; i++) {
		trx[i] = new Transceiver(5700 + 2 * i, "127.0.0.1",
					 drive, radio, SAMPSPERSYM,
					 i, primary);
		trx[i]->start();
		primary = false;
	}

	while (!gbShutdown)
		sleep(1);

	LOG(NOTICE) << "Shutting down transceivers...";
	for (int i = 0; i < numARFCN; i++)
		trx[i]->shutdown();

	/* Allow time for threads to end before we start freeing objects */
	sleep(2);

	for (int i = 0; i < numARFCN; i++)
		delete trx[i];
	delete drive;
	delete radio;
	delete device;
}
