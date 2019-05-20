#include <wiringPi.h>
#include <mcp23017.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <iostream>

#define EXPANDER_COUNT 2
#define EXPANDER_PIN_COUNT 16*EXPANDER_COUNT

using namespace std;

const uint8_t pinBase=65;

const uint8_t expanderAddr[EXPANDER_COUNT] = {
	0x21,
	0x23
};

int main() {
	wiringPiSetup();
	for(unsigned int x=0;x<EXPANDER_COUNT;x++)
	{
		mcp23017Setup(pinBase+16*x, expanderAddr[x]);
	}
	for(unsigned int x=pinBase;x<pinBase+EXPANDER_PIN_COUNT;x++)
	{
		pinMode(x, OUTPUT);
		digitalWrite(x, LOW);
	}
	for(unsigned int x=pinBase;x<pinBase+EXPANDER_PIN_COUNT;x++)
	{
		cout << x << endl;
		digitalWrite(x, HIGH);
		char v;
		cin >> v;
		digitalWrite(x, LOW);
	}
	return 0;
}