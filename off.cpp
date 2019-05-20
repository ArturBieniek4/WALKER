#include <wiringPi.h>
#include <mcp23017.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

#define EXPANDER_COUNT 4
#define EXPANDER_PIN_COUNT 16*EXPANDER_COUNT
#define MOTOR_COUNT 9

using namespace std;

const uint8_t pinBase=65;

const uint8_t expanderAddr[EXPANDER_COUNT] = {
	0x21,
	0x23,
	0x20
};

const uint8_t motorPin[MOTOR_COUNT][2] = { // S,K
	{65,74}, // P1 ..   Z
	{66,75}, // P2 .    Z
	{67,76}, // P3 ...  Z
	{68,77}, // P4 .... Z
	{69,78}, // P5 ...  C
	{70,79}, // P6 ..   C
	{71,80}, // P7      G
	{72,83}, // P8 .    C
	{73,84}  // P9 .... C
};

int main() {
	wiringPiSetup();
	for(uint8_t x=0;x<EXPANDER_COUNT;x++)
	{
		mcp23017Setup(pinBase+16*x, expanderAddr[x]);
	}
	for(uint8_t y=0;y<3;y++)
	{
		for(uint8_t x=0;x<MOTOR_COUNT;x++)
		{
			pinMode(motorPin[x][y], OUTPUT);
			digitalWrite(motorPin[x][y], HIGH);
		}
	}
	cout << "ALL OUTPUTS SET TO LOW[OK]" << endl;
	return 0;
}