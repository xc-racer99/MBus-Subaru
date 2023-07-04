/***
MBus.h - Library to emulate Alpine M-Bus commands

Copyright 2012 Oliver Mueller

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***/
#include "MBus.h"

MBus::BitTrain MBus::bitTrain;
SemaphoreHandle_t MBus::bitTrainMutex;
hw_timer_t *MBus::dataTimer = NULL;

MBus::MBus(uint8_t pin)
{
	_in = pin;
	_out = pin;
	_invertedSend = true;

	pinMode(pin, (OUTPUT_OPEN_DRAIN | INPUT));
	digitalWrite(pin, HIGH);

	if (dataTimer == NULL) {
		dataTimer = timerBegin(0, 80, true);
		timerAttachInterrupt(dataTimer, &onDataTimer, true);
	}

	if (bitTrainMutex == NULL) {
		bitTrainMutex = xSemaphoreCreateMutex();
	}
}

MBus::MBus(uint8_t in, uint8_t out)
{
	_in = in;
	_out = out;
	_invertedSend = false;
	
	pinMode(_in, INPUT);
	pinMode(_out, OUTPUT);

	if (dataTimer == NULL) {
		dataTimer = timerBegin(0, 80, true);
		timerAttachInterrupt(dataTimer, &onDataTimer, true);
	}

	if (bitTrainMutex == NULL) {
		bitTrainMutex = xSemaphoreCreateMutex();
	}
}

void MBus::onDataTimer()
{
	// Don't need to hold bitTrainMutex as it should already be held by "calling" code
	if (bitTrain.bitInProgress) {
		digitalWrite(bitTrain.pinOut, bitTrain.invertedSend ? HIGH : LOW);

		bitTrain.bitsSent++;
		bitTrain.bitInProgress = false;

		if (bitTrain.bitsSent < bitTrain.bitsTotal) {
			if (bitTrain.bits[bitTrain.bitsSent - 1]) {
				timerAlarmWrite(dataTimer, 1200, false);
			} else {
				timerAlarmWrite(dataTimer, 2400, false);
			}

			timerWrite(dataTimer, 0);
			timerAlarmEnable(dataTimer);
		} else {
			timerAlarmDisable(dataTimer);
		}
	} else {
		// Send the first part of the data bit
		digitalWrite(bitTrain.pinOut, bitTrain.invertedSend ? LOW : HIGH);

		if (bitTrain.bits[bitTrain.bitsSent]) {
			timerAlarmWrite(dataTimer, 1800, false);
		} else {
			timerAlarmWrite(dataTimer, 600, false);
		}

		timerWrite(dataTimer, 0);
		timerAlarmEnable(dataTimer);

		bitTrain.bitInProgress = true;
	}
}

void MBus::writeHexBitWise(uint8_t nibble)
{
	for (int8_t i = 3; i >= 0; i--) {
		uint8_t bit = ((nibble & (1 << i) ) >> i);
		if (bit == 1) {
			bitTrain.bits[bitTrain.bitsTotal] = true;
		} else {
			bitTrain.bits[bitTrain.bitsTotal] = false;
		}
		bitTrain.bitsTotal++;
	}
}

bool MBus::checkParity(uint64_t *message)
{
	uint8_t parity = 0;
	uint8_t nibble = 0;
	for (uint8_t i = 15; i > 0; i--) {
		nibble = ((uint64_t)*message >> i * 4) & 0xf;
		parity = parity ^ nibble;
	}
	parity += 1;
	parity &= 0xf;
	
	if (parity == (*message & (uint64_t)0xf)) {
		return true;
	} else {
		return false;
	}
}

void MBus::send(uint64_t message)
{
	bool firstDataSent = 0;
	uint8_t parity = 0;

	if (xSemaphoreTake(bitTrainMutex, portMAX_DELAY)) {
		bitTrain.bitsTotal = 0;
		bitTrain.bitsSent = 0;
		bitTrain.bitInProgress = false;
		bitTrain.invertedSend = _invertedSend;
		bitTrain.pinOut = _out;

		for (int8_t i = 15; i >= 0; i--) {
			uint8_t nibble = ((uint64_t)message >> i * 4) & 0xf;
			parity = parity ^ nibble;
			if (nibble == 0 && !firstDataSent) {
				// Do nothing, first actual data bit not sent yet
			} else {
				writeHexBitWise(nibble);
				firstDataSent = true;
			}
		}
		parity += 1;
		parity &= 0xf;

		writeHexBitWise(parity);

		// Setup repeating timer for 10us, so that first bit starts soon - but slow enough that we have time to reset it
		timerAlarmWrite(dataTimer, 10, true);
		timerWrite(dataTimer, 0);
		timerAlarmEnable(dataTimer);

		// Delay to allow all bits to be transferred - 3ms a bit
		vTaskDelay(pdMS_TO_TICKS(bitTrain.bitsTotal * 3));

		// Ensure all bits are transferred before returning
		while (bitTrain.bitsTotal != bitTrain.bitsSent) {
			vTaskDelay(pdMS_TO_TICKS(1));
		}

		xSemaphoreGive(bitTrainMutex);
	}
}

bool MBus::receive(uint64_t *message)
{
	*message = 0;
	if (digitalRead(_in) == LOW) {
		unsigned long bitStartTime = micros();
		bool bitFinished = false; 
		uint8_t totalBitsRead = 0;

		while ((micros() - bitStartTime) < 4000) {
			if (digitalRead(_in) == HIGH && !bitFinished) {
				if ((micros() - bitStartTime) < 1400 && (micros() - bitStartTime) > 600) {
					// 0 is in between 600 and 1700 microseconds
					*message *= 2;
					totalBitsRead++;
					bitFinished = true; 
				} else if ((micros() - bitStartTime) > 1400) {
					// 1 is longer then 1400 microseconds
					*message *= 2;
					*message += 1;
					totalBitsRead++;
					bitFinished = true;
				}
			}
			// Transfer to next bit if we are reading low again
			if (bitFinished && digitalRead(_in) == LOW) {
				bitFinished = false;
				bitStartTime = micros();
			}
		}
		if (totalBitsRead == 0 || totalBitsRead % 4 || !checkParity(message)) {
			// Message is not ok
			*message = 0;
			return false;
		} else {
			// Discard parity nibble
			(*message) = (*message) >> 4;
			return true;
		}
	}
	return false;
}

/*
 CD-changer emulation from here on
*/
void MBus::sendPlayingTrack(uint8_t Track, uint16_t Time)
{
	uint64_t play = 0xE94000100000001ull;
	play |= (uint64_t)(Track % 10) << (10 * 4);
	play |= (uint64_t)(Track / 10) << (11 * 4);
	
	play |= (uint64_t)(Time % 10) << (4 * 4);
	play |= (uint64_t)((Time % 100) / 10) << (5 * 4);
	play |= (uint64_t)((Time / 60) % 10) << (6 * 4);
	play |= (uint64_t)(((Time / 60) % 100) / 10) << (7 * 4);
	
	send(play);
}

void MBus::sendChangedCD(uint8_t CD, uint8_t Track)
{
	uint64_t play = 0xEB900000001ull;
	play |= (uint64_t)CD << (7 * 4);
	play |= (uint64_t)(Track % 10) << (5 * 4);
	play |= (uint64_t)(Track / 10) << (6 * 4);
	send(play);
}

void MBus::sendCDStatus(uint8_t CD)
{
	uint64_t play = 0xEC001999999Full;
	play |= (uint64_t)CD << (9 * 4);
	send(play);
}
