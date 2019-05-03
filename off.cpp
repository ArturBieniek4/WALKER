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
	0x20,
	0x21,
	0x24,
	0x22
};

const uint8_t motorPin[MOTOR_COUNT][3] = {
	{104,105,119},//motor1
	{103,106,118},//motor2
	{102,107,117},//motor3
	{101,108,116},//motor4
	{124,109,115},//motor5
	{99,110,114},//motor6
	{98,111,125},//motor7
	{97,112,113},//motor8
	{122,121,123},//motor9
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
			digitalWrite(motorPin[x][y], LOW);
		}
	}
	cout << "ALL OUTPUTS SET TO LOW[OK]" << endl;
	return 0;
}