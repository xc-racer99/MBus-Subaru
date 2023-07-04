/***
MBus.cpp - Library to emulate Alpine M-Bus commands

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
#ifndef MBus_h
#define MBus_h
#include "Arduino.h"
	
class MBus
{
	public:
		MBus(uint8_t in, uint8_t out);
		MBus(uint8_t pin);
		void send(uint64_t message);
		bool receive(uint64_t *message);
		void sendPlayingTrack(uint8_t Track, uint16_t Time);
		void sendChangedCD(uint8_t CD, uint8_t Track);
		void sendCDStatus(uint8_t CD);
	private:
		void writeHexBitWise(uint8_t message);
		bool checkParity(uint64_t *message);
		static void onDataTimer();

		struct BitTrain {
			bool bits[64];
			uint16_t bitsTotal;
			uint16_t bitsSent;
			bool bitInProgress;
			bool invertedSend;
			int pinOut;
		};

		static BitTrain bitTrain;
		static SemaphoreHandle_t bitTrainMutex;
		static hw_timer_t *dataTimer;

		uint8_t _in;
		uint8_t _out;
		bool _invertedSend = true;
};
#endif	
