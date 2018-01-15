#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include "rtl-sdr.h"

rtlsdr_dev_t* dev = NULL;

unsigned char* buffData;
uint32_t buffInc = 0;
#define BUFF_SIZE 16*1024*1024
#define RECEIVE_BUFF_SIZE 16384*8

// do moving average
// compare up/down

uint32_t movingAverageValue = 0;
#define MOVING_AVERAGE_MAX 512
uint16_t movingAverageCount = 0;

uint16_t currAverage = 0;
uint16_t movingAverageValues[MOVING_AVERAGE_MAX];
uint16_t movingAverageBuffInc = 0;

void calcCurrMovingAverage(uint16_t val) {
	
	// movingAverageValue += val;

	if(movingAverageCount > MOVING_AVERAGE_MAX) {
		movingAverageValue -= movingAverageValues[movingAverageBuffInc];
	} else {
		movingAverageCount++;
	}
	movingAverageValues[movingAverageBuffInc] = val;

	movingAverageValue += movingAverageValues[movingAverageBuffInc];

	movingAverageBuffInc++;

	if(movingAverageBuffInc >= MOVING_AVERAGE_MAX) {
		movingAverageBuffInc = 0;
	}

	currAverage = movingAverageValue / movingAverageCount;
}

#define READER_BUFFER_MAX_SIZE 512

uint32_t packetInc = 0;

void parseAmData(uint16_t* amData, uint32_t len) {

	uint16_t currAmData = 0;

	uint16_t buffAvg = 0;
	int i = 0;
	while(i < (len)) {
		
		currAmData = amData[i];
		calcCurrMovingAverage(currAmData);

		// printf("Curr am: %04x %04x - %d - %d\n",(currAmData),currAverage,currAmData > (currAverage*0.25),i);

		uint8_t readerThreshold = (currAmData > (currAverage*0.25));
		uint8_t tagThreshold = i>0 && (currAmData > (amData[i-1]*1.05));

		// handle state
		if(tagThreshold == 1) {
			// printf("FOUND THRESH\n");
		} else if(readerThreshold == 0) {
			// printf("Got down threshold, trying SOF\n");
			// escape possible wrong output - make more accurate later
			i += 2;

			uint8_t lastReaderMask = 0xff;

			uint8_t bits[READER_BUFFER_MAX_SIZE*8];
			uint8_t packets[READER_BUFFER_MAX_SIZE];

			memset(packets,0,READER_BUFFER_MAX_SIZE);

			uint32_t bitCount = 0;
			uint8_t done = 0;
			while(done == 0) {

				uint8_t readerMask = 0x00;

				// i += 4;
				for(int j = 0 ; j < 4 ; j++) {

					readerThreshold = (amData[i] > (currAverage*0.25));

					readerMask <<=1;
					readerMask |= readerThreshold;
					i += 4;
				}

				// process modified miller
				switch(readerMask) {
					case 0x07:

						bits[bitCount] = 0;
						break;
					case 0x0f:

						if(lastReaderMask == 0x0d) {
							bits[bitCount] = 0;
						} else {
							done = 1;
						}

						break;
					case 0x0d:
						bits[bitCount] = 1;
						break;
					default:
						done = 1;
						break;
				}

				bitCount++;

				lastReaderMask = readerMask;

				// printf("Reader output: %02x\n",readerMask);
			}

			// process bits into packet

			uint8_t parityInc = 0;
			uint8_t bit = 0;
			uint16_t size = 0;
			for(int j = 1 ; j < bitCount ; j++) {

				if(bits[j] == 1) {
					packets[size] |= (1<<bit);
				}
				
				// printf("Bit: %d %01x - %02x\n",j,bits[j],packets[0]);

				parityInc++;
				bit++;

				if(parityInc >= 8) {
					j++;
					size++;
					parityInc = 0;
					bit = 0;
				}
			}

			if(bitCount >= 7) {

				printf("%08x %d RD: ",packetInc,currAverage);
				for(int k = 0 ; k < size ; k++) {
					printf("%02x ",packets[k]);
				}
				printf("\n");
				/*
				printf("%08x RD(B): ",packetInc);
				for(int k = 0 ; k < bitCount ; k++) {
					printf("%01x",bits[k]);
				}
				printf("\n");
				*/
				packetInc++;
			}
		}

		i++;
	}

}

uint16_t squares[256];
int abs8(int x) {
	if (x >= 127) {
		return x - 127;
	}
	return 127 - x;
}
void computeSquares() {
	int i, j;
	for (i=0; i<256; i++) {
		j = abs8(i);
		squares[i] = (uint16_t)(j*j);
	}
}

void buffCallback(unsigned char *buf, uint32_t len, void *ctx) {

	// process output
	uint16_t demodAM[RECEIVE_BUFF_SIZE];
	for(int i = 0 ; i < len ; i+= 2) {
		demodAM[i/2] = squares[buf[i]] + squares[buf[i+1]];
	}

	parseAmData(demodAM,len/2);

}

void generateAMFiles() {

	computeSquares();

	int amFile = open("analog-1-3-1",O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);

	unsigned char amBuff[4096];

	uint32_t inc = 0;

	float largestVal = 0;
	// get max buffer size
	while(inc < BUFF_SIZE) {
		float amVal = squares[buffData[inc]] + squares[buffData[inc+1]];
		
		if(amVal > largestVal) {
			largestVal = amVal;
		}

		inc+=2;
	}

	inc = 0;
	while(inc < BUFF_SIZE) {
	
		for(int j = 0 ; j < (4096/4) ; j++) {

			float amVal = squares[buffData[inc]] + squares[buffData[inc+1]];

			amVal = amVal * 20 / largestVal;
			// printf("AM Val: %f\n",amVal);

			memcpy(&amBuff[j*4],&amVal,4);

			inc += 2;
		}
		write(amFile,amBuff,4096);
	}

	close(amFile);
}

uint16_t* amData;

int main(int argc, char** argv) {

	memset(movingAverageValues,0x00,MOVING_AVERAGE_MAX);
	computeSquares();

	// sample data
	/*
	amData = malloc(BUFF_SIZE);
	int amFile = open("am_data.bin",O_RDONLY);
	read(amFile,amData,BUFF_SIZE);

	parseAmData(amData,BUFF_SIZE/2);
	close(amFile);
	return 0;
	*/
	// re-enable after file parsing is complete

	// get some rtlsdr data
	int ret = rtlsdr_open(&dev, 0);

	if(ret < 0) {
		printf("RTLSDR Open failed\n");
		return 1;
	}

	buffData = malloc(BUFF_SIZE);

	int gainCount;
	int allGains[100];

	// get highest gain
	gainCount = rtlsdr_get_tuner_gains(dev, allGains);
	rtlsdr_set_tuner_gain(dev, allGains[gainCount-1]);

	rtlsdr_set_center_freq(dev, 27.12e6);

	rtlsdr_set_sample_rate(dev, 1.695e6);

	rtlsdr_reset_buffer(dev);

	rtlsdr_read_async(dev, buffCallback, NULL,
		12,
		RECEIVE_BUFF_SIZE
	);

	while(1);

	// uint16_t amVal = squares[buffData[inc]] + squares[buffData[inc+1]];

	return 0;
}
