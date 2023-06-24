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

MBus::MBus(uint8_t pin)
{
	_in = pin;
	_out = pin;
	_invertedSend = true;

	pinMode(pin, (OUTPUT_OPEN_DRAIN | INPUT));
	digitalWrite(pin, HIGH);
}

MBus::MBus(uint8_t in, uint8_t out)
{
	_in = in;
	_out = out;
	_invertedSend = false;
	
	pinMode(_in, INPUT);
	pinMode(_out,OUTPUT);
}

void MBus::sendZero()
{
	digitalWrite(_out, _invertedSend ? LOW : HIGH);
	delayMicroseconds(600);
	digitalWrite(_out, _invertedSend ? HIGH : LOW);
	delayMicroseconds(2400);
}

void MBus::sendOne()
{
	digitalWrite(_out, _invertedSend ? LOW : HIGH);
	delayMicroseconds(1800);
	digitalWrite(_out, _invertedSend ? HIGH : LOW);
	delayMicroseconds(1200);
}

void MBus::writeHexBitWise(uint8_t nibble)
{
	for (int8_t i = 3; i >= 0; i--) {
		uint8_t bit = ((nibble & (1 << i) ) >> i);
		if (bit == 1) {
			sendOne();
		} else {
			sendZero();
		}
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
	uint64_t play = 0x994000100000001ull;
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
	uint64_t play = 0x9B900000001ull;
	play |= (uint64_t)CD << (7 * 4);
	play |= (uint64_t)(Track % 10) << (5 * 4);
	play |= (uint64_t)(Track / 10) << (6 * 4);
	send(play);
}

void MBus::sendCDStatus(uint8_t CD)
{
	uint64_t play = 0x9C001999999Full;
	play |= (uint64_t)CD << (9 * 4);
	send(play);
}
