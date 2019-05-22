#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <math.h>
#include "I2Cdev.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <mcp23017.h>
#include <iostream>
#include <cerrno>
#include <ctime>
#include <pthread.h>
#include <string>
#include <sstream>
#include <chrono>
#include <vector>
#include <fstream>
#include <iomanip>
#include "json.hpp"

#include "MPU6050.h"

#define MPU_COUNT 4
#define ICM_COUNT 4
#define EXPANDER_COUNT 3
#define EXPANDER_PIN_COUNT 16*EXPANDER_COUNT
#define EMERGENCY_STOP_PIN 2
#define ENDSTOP_COUNT 18

#define UDP_BUFFER_SIZE 4096
#define UDP_SERVER_PORT 11000

#define MOTOR_COUNT 9

#define MOTOR_ON LOW
#define MOTOR_OFF HIGH

using namespace std;
using json = nlohmann::json;

const float accelScale = 16384.0f;

char znak0;
string buf0 = "";
string line0 = "";
char znak1;
string buf1 = "";
string line1 = "";

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const { 
        return chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef chrono::high_resolution_clock clock_;
    typedef chrono::duration<double, ratio<1> > second_;
    chrono::time_point<clock_> beg_;
};

Timer tmr;
Timer tmr2;
Timer tmr3;

int sockfd;
char udpBuffer[UDP_BUFFER_SIZE];
struct sockaddr_in servaddr, cliaddr;

MPU6050 mpu[MPU_COUNT] {
{ 4, 0x68 },
{ 5, 0x68 },
{ 6, 0x68 },
{ 3, 0x68 }
};

#define OUTPUT_READABLE_YAWPITCHROLL

pthread_mutex_t mutex_ypr = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_full_ypr = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutex_i2c = PTHREAD_MUTEX_INITIALIZER;

bool dmpReady[MPU_COUNT];
uint8_t mpuIntStatus[MPU_COUNT];
uint8_t devStatus[MPU_COUNT];
bool readComplete[MPU_COUNT+ICM_COUNT];
uint8_t readCompleteCount;
bool stopped;


float gyroCorrectionTime = 0.01;
float gyroCorrectionDelay = 0.1;
bool verboseMode = false;

const uint8_t pinBase=65;
const uint8_t expanderAddr[EXPANDER_COUNT] = {
	0x21,
	0x23,
	0x20
};

float destinations[MOTOR_COUNT]
{
	0,//motor1
	0,//motor2
	0,//motor3
	0,//motor4
	0,//motor5
	0,//motor6
	0,//motor7
	0,//motor8
	0//motor9
};

float goToDestination[MOTOR_COUNT]
{
	false,//motor1
	false,//motor2
	false,//motor3
	false,//motor4
	false,//motor5
	false,//motor6
	false,//motor7
	false,//motor8
	false//motor9
};

const uint8_t endstopPin[ENDSTOP_COUNT]{
	81,
	82,
	97,
	98,
	99,
	100,
	101,
	102,
	103,
	104,
	105,
	106,
	107,
	108,
	109,
	110,
	111,
	112,
};

const uint8_t endstopMotor[MOTOR_COUNT][5]{ // krańcówka góra,krańcówka dół,gyro 1,oś gyro, gyro 2
	{98,112,3,2,0}, // P1 ..   Z
	{100,103,4,2,7},  // P2 .    Z
	{104,106,0,2,4},  // P3 ...  Z
	{108,81,4,0,7},  // P4 .... Z 
	{107,105,2,2,5},  // P5 ...  C
	{111,110,1,2,2},  // P6 ..   C
	{109,97,6,0,7},  // P7      G 
	{102,101,5,2,7},  // P8 .    C
	{82,99,5,0,7}   // P9 .... C
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

const int ypr_correction[MPU_COUNT + ICM_COUNT][3] = {
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{0,0,0},
	{-15,0,0},
	{30,0,0},
	{-56,0,0},
	{0,0,0}
};

const unsigned int motorDir[MOTOR_COUNT][2]
{
	{LOW,HIGH}, // P1 ..   Z
	{LOW,HIGH}, // P2 .    Z
	{LOW,HIGH}, // P3 ...  Z
	{HIGH,LOW}, // P4 .... Z
	{HIGH,LOW}, // P5 ...  C
	{HIGH,LOW}, // P6 ..   C
	{LOW,HIGH}, // P7      G
	{HIGH,LOW}, // P8 .    C
	{HIGH,LOW}  // P9 .... C
};

float ypr[MPU_COUNT+ICM_COUNT][3];
float full_ypr[MPU_COUNT+ICM_COUNT][3];

int USB0, USB1;

void emergencyStop()
{
	stopped=true;
	for(uint8_t y=0;y<2;y++)
	{
		for(uint8_t x=0;x<MOTOR_COUNT;x++)
		{
			digitalWrite(motorPin[x][y], MOTOR_OFF);
		}
	}
	cout << "EMERGENCY_STOP!!!" << endl;
}

float compareAngles(float x, float y)
{
	float difference = y - x;
    while (difference < -180) difference += 360;
    while (difference > 180) difference -= 360;
    return difference;
}

void setup() {
	wiringPiSetup();
	pinMode(EMERGENCY_STOP_PIN, INPUT);
	pullUpDnControl(EMERGENCY_STOP_PIN, PUD_UP);
	if ( wiringPiISR (EMERGENCY_STOP_PIN, INT_EDGE_RISING, &emergencyStop) < 0 ) {
      cout << "Unable to setup ISR";
      return;
	}
	if(digitalRead(EMERGENCY_STOP_PIN)==HIGH)
	{
		cout << "EMERGENCY_STOP_PIN is UNCONNECTED!!!" << endl;
		//stopped = true;
	}
	else
	{
		cout << "EMERGENCY_STOP_PIN is CONNECTED[OK]" << endl;
		stopped = false;
	}
	while(stopped){
		if(digitalRead(EMERGENCY_STOP_PIN)==LOW){
			stopped = false;
			cout << "EMERGENCY_STOP_PIN is CONNECTED AGAIN." << endl;
		}
	}
	for(uint8_t x=0;x<EXPANDER_COUNT;x++)
	{
		mcp23017Setup(pinBase+16*x, expanderAddr[x]);
	}
	for(uint8_t y=0;y<2;y++)
	{
		for(uint8_t x=0;x<MOTOR_COUNT;x++)
		{
			pinMode(motorPin[x][y], OUTPUT);
			digitalWrite(motorPin[x][y], MOTOR_OFF);
		}
	}
	for(uint8_t x=0;x<ENDSTOP_COUNT;x++)
	{
		pinMode(endstopPin[x], INPUT);
		pullUpDnControl(endstopPin[x], PUD_UP);
		cout << "ENDSTOP[" << (int)endstopPin[x] << "] = " << (int)digitalRead(endstopPin[x]) << endl;
	}
	if ((USB0 = serialOpen ("/dev/ttyUSB0", 115200)) < 0)
	{
		cout << "Unable to open serial device USB0";
		/*emergencyStop();
		exit(0);*/
	}
	if ((USB1 = serialOpen ("/dev/ttyUSB1", 115200)) < 0)
	{
		cout << "Unable to open serial device USB1";
		/*emergencyStop();
		exit(0);*/
	}
	// MPU6050 initialization
	for(uint8_t x=0;x<MPU_COUNT;x++)
	{
		mpu[x].initialize();
		if(mpu[x].testConnection())
			cout << "MPU6050[" << (int)x << "] connection successful" << endl;
		else
			cout << "MPU6050[" << (int)x <<"] connection failed" << endl;
		dmpReady[x] = true;
	}
	// UDP server initialization
	cout << "Creating UDP server socket...";
	if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		cout << "[FAILED]" << endl;
		exit(EXIT_FAILURE); 
    }
	cout << "[OK]" << endl << "Binding UDP server socket...";
	memset(&servaddr, 0, sizeof(servaddr));
	memset(&cliaddr, 0, sizeof(cliaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = INADDR_ANY;
	servaddr.sin_port = htons(UDP_SERVER_PORT);
	if ( bind(sockfd, (const struct sockaddr *)&servaddr,  
	sizeof(servaddr)) < 0 ) 
    {
        cout << "[FAILED]" << endl; 
        exit(EXIT_FAILURE); 
    }
	cout << "[OK]" << endl << "Waiting for connection...";
	socklen_t len;
	recvfrom(sockfd, (char *)udpBuffer, UDP_BUFFER_SIZE,  MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
	cout << "[OK]" << endl;
}

void *readMPU(void *) {
	while(true){
		for(uint8_t x=0;x<MPU_COUNT;x++)
		{
			readComplete[x] = false;
			readCompleteCount = 0;
		}
		while(readCompleteCount<MPU_COUNT){
			for(uint8_t x=0;x<MPU_COUNT;x++)
			{
				if(!readComplete[x]){
					int16_t accelX, accelY, accelZ;
					pthread_mutex_lock(&mutex_i2c);
					mpu[x].getAcceleration(&accelX, &accelY, &accelZ);
					pthread_mutex_unlock(&mutex_i2c);
					float accelX_scaled = accelX/accelScale;
					float accelY_scaled = accelY/accelScale;
					float accelZ_scaled = accelZ/accelScale;
					pthread_mutex_lock(&mutex_ypr);
					ypr[x][1] = atan2 (accelY_scaled ,( sqrt ((accelX_scaled * accelX_scaled) + (accelZ_scaled * accelZ_scaled)))) * 180/M_PI;
					ypr[x][2] = atan2(-accelX_scaled ,( sqrt((accelY_scaled * accelY_scaled) + (accelZ_scaled * accelZ_scaled)))) * 180/M_PI;
					pthread_mutex_unlock(&mutex_ypr);
					readComplete[x] = true;
					readCompleteCount++;
				}
			}
		}
		for(uint8_t x=0;x<MPU_COUNT;x++)
		{
			pthread_mutex_lock(&mutex_ypr);
			pthread_mutex_lock(&mutex_full_ypr);
			full_ypr[x][0] = ypr[x][0] + ypr_correction[x][0];
			full_ypr[x][1] = ypr[x][1] + ypr_correction[x][1];
			full_ypr[x][2] = ypr[x][2] + ypr_correction[x][2];
			pthread_mutex_unlock(&mutex_full_ypr);
			pthread_mutex_unlock(&mutex_ypr);
		}
	}
}

void *readUno1(void *){
	while(true){
		znak1 = serialGetchar(USB1);
		if (znak1=='$') {
			line1 = buf1;
			buf1="";
			vector <string> tokens;
			stringstream check1(line1);
			string intermediate;
			while(getline(check1, intermediate, ':')) 
			{
				tokens.push_back(intermediate);
			}
			if(tokens[0]=="#1"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+2][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+2][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO1, gyro1" << endl;
			}
			
			else if(tokens[0]=="#2"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+3][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+3][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO1, gyro2" << endl;
			}
			else if(tokens[0]=="#3"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+0][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+0][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO1, gyro3" << endl;
			}
			
			else if(tokens[0]=="#4"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+1][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+1][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO1, gyro4" << endl;
			}
			else{
				cout << line0 << endl;
			}
		}
		else{
			buf1 += znak1;
			cout << znak1;
		}
	}
}

void *readUno2(void *){
	while(true){
		znak0 = serialGetchar(USB0);
		if (znak0=='$') {
			line0 = buf0;
			buf0="";
			vector <string> tokens;
			stringstream check1(line0);
			string intermediate;
			while(getline(check1, intermediate, ':')) 
			{
				tokens.push_back(intermediate);
			}
			if(tokens[0]=="#1"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+2][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+2][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO2, gyro1" << endl;
			}
			
			else if(tokens[0]=="#2"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+3][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+3][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO2, gyro2" << endl;
			}
			else if(tokens[0]=="#3"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+0][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+0][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO2, gyro3" << endl;
			}
			
			else if(tokens[0]=="#4"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+1][i-1] = atof(tokens[i].c_str()) + ypr_correction[MPU_COUNT+1][i-1];
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			cout << "UNO2, gyro4" << endl;
			}
			else{
				cout << line0 << endl;
			}
			
		}
		else{
			buf0 += znak0;
		}
	}
}

void *consoleInput(void *) {
	while(true){
		string instr;
		cin >> instr;
		uint8_t direction = LOW;
		uint8_t motorNum;
		if(instr.length()==2)
		{
			motorNum = (instr[1]-'0')-1;
			if(instr[0]=='u'){
				direction = motorDir[motorNum][0];
			}
			else if(instr[0]=='d'){
				direction = motorDir[motorNum][1];
			}
			else
			{
				direction = motorDir[motorNum][1];
			}
			if(stopped) continue;
			if(digitalRead(endstopMotor[motorNum][direction])==HIGH) continue;
			tmr3.reset();
			try {
				if (digitalRead(endstopMotor[motorNum][direction])==LOW)
						{
							digitalWrite(motorPin[motorNum][1], direction);
							digitalWrite(motorPin[motorNum][0], MOTOR_ON);
						}
				while(stopped == false && tmr3.elapsed()<gyroCorrectionTime && digitalRead(endstopMotor[motorNum][direction])==LOW)
				{
					
				}
				digitalWrite(motorPin[motorNum][0], MOTOR_OFF);
				if(digitalRead(endstopMotor[motorNum][direction])==HIGH)
				{
					if(verboseMode) cout << "TRIGERRED FROM ENDSTOP" << endl;
				}
			}
			catch(string e){
				emergencyStop();
				cout << endl << "EXCEPTION: " << e;
			}
		}
	}
}

void *gyroAutoCorrection(void *) {
	unsigned int direction[MOTOR_COUNT];
	float gyroval[MOTOR_COUNT];
	unsigned int gyroid[MOTOR_COUNT];
	unsigned int gyroid2[MOTOR_COUNT];
	unsigned int axisid[MOTOR_COUNT];
	for (unsigned int motorNum=0; motorNum<MOTOR_COUNT; motorNum++)
	{
		gyroid[motorNum] = endstopMotor[motorNum][2];
		gyroid2[motorNum] = endstopMotor[motorNum][4];
		axisid[motorNum] = endstopMotor[motorNum][3];
	}
	while(true){
		tmr2.reset();
		for (unsigned int motorNum=0; motorNum<MOTOR_COUNT; motorNum++)
		{
			if(goToDestination[motorNum])
			{
				gyroval[motorNum] = compareAngles((full_ypr[gyroid[motorNum]][axisid[motorNum]]) , (full_ypr[gyroid2[motorNum]][axisid[motorNum]]));
				float diff = compareAngles(destinations[motorNum], gyroval[motorNum]);
				if(abs(diff)>5)
				{
					if(diff<0){
						direction[motorNum] = motorDir[motorNum][0];
						if(verboseMode) cout << "UP" << full_ypr[gyroid[motorNum]][axisid[motorNum]] << " " << full_ypr[gyroid2[motorNum]][axisid[motorNum]] << endl;
					}
					else if(diff>0){
						direction[motorNum]  = motorDir[motorNum][1];
						if(verboseMode) cout << "DOWN" << full_ypr[gyroid[motorNum]][axisid[motorNum]] << " " << full_ypr[gyroid2[motorNum]][axisid[motorNum]] << endl;
					}
					try {
						if (digitalRead(endstopMotor[motorNum][direction[motorNum]])==LOW)
						{
							digitalWrite(motorPin[motorNum][1], direction[motorNum]);
							digitalWrite(motorPin[motorNum][0], MOTOR_ON);
						}
					}
					catch(string e){
						emergencyStop();
						cout << endl << "EXCEPTION: " << e;
					}
				}
			}
		}
		tmr.reset();
		while(stopped == false && tmr.elapsed()<gyroCorrectionTime)
		{
			for (unsigned int motorNum=0; motorNum<MOTOR_COUNT; motorNum++)
			{
				if(goToDestination[motorNum])
				{
					if(digitalRead(endstopMotor[motorNum][direction[motorNum]])==LOW)
					{
						break;
					}
				}
			}
			
		}
		for (unsigned int motorNum=0; motorNum<MOTOR_COUNT; motorNum++)
		{
			if(goToDestination[motorNum])
			{
				digitalWrite(motorPin[motorNum][0], MOTOR_OFF);
				digitalWrite(motorPin[motorNum][1], MOTOR_OFF);
			}
		}
		
		usleep((gyroCorrectionDelay-tmr2.elapsed())*1000000);
	}
}

void *UDPServer(void *) {
	while(true){
		memset(&servaddr, 0, sizeof(servaddr));
		memset(&cliaddr, 0, sizeof(cliaddr));
		servaddr.sin_family = AF_INET;
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(UDP_SERVER_PORT);
		socklen_t len = sizeof(cliaddr);
		int n = recvfrom(sockfd, (char *)udpBuffer, UDP_BUFFER_SIZE,  MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);
		string s(udpBuffer);
		string jRequestString = s.substr(0,n);
		auto jRequestObj = json::parse(jRequestString);
		unsigned int type = jRequestObj["type"];
		if(type==1){
			float degs[MOTOR_COUNT];
			for(unsigned int motorNum=0;motorNum<MOTOR_COUNT;motorNum++)
			{
				unsigned int gyroid = endstopMotor[motorNum][2];
				unsigned int gyroid2 = endstopMotor[motorNum][4];
				unsigned int axisid = endstopMotor[motorNum][3];
				degs[motorNum] = compareAngles((full_ypr[gyroid][axisid]) , (full_ypr[gyroid2][axisid]));
			}
			stringstream timeBuffer;
			time_t now = time(0);
			tm timeTM = * localtime( & now );
			timeBuffer << put_time(&timeTM, "%d.%m.%Y %H:%M:%S");
			json jResponseObj;
			jResponseObj["status"]=1;
			jResponseObj["errorName"]="";
			jResponseObj["ypr"]=full_ypr;
			jResponseObj["degs"]=degs;
			jResponseObj["time"]=timeBuffer.str();
			string jResponseString = jResponseObj.dump();
			sendto(sockfd, jResponseString.c_str(), strlen(jResponseString.c_str()), MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);
		}
		else if(type==2)
		{
			goToDestination[0] = jRequestObj["goToDest1"];
			goToDestination[1] = jRequestObj["goToDest2"];
			goToDestination[2] = jRequestObj["goToDest3"];
			goToDestination[3] = jRequestObj["goToDest4"];
			goToDestination[4] = jRequestObj["goToDest5"];
			goToDestination[5] = jRequestObj["goToDest6"];
			goToDestination[6] = jRequestObj["goToDest7"];
			goToDestination[7] = jRequestObj["goToDest8"];
			goToDestination[8] = jRequestObj["goToDest9"];
			
			destinations[0] = jRequestObj["destination1"];
			destinations[1] = jRequestObj["destination2"];
			destinations[2] = jRequestObj["destination3"];
			destinations[3] = jRequestObj["destination4"];
			destinations[4] = jRequestObj["destination5"];
			destinations[5] = jRequestObj["destination6"];
			destinations[6] = jRequestObj["destination7"];
			destinations[7] = jRequestObj["destination8"];
			destinations[8] = jRequestObj["destination9"];
		}
		else if(type==4)
		{
			cout << "PROGRAM EXIT..." << endl;
			emergencyStop();
			exit(0);
		}
		else if(type==5)
		{
			gyroCorrectionTime = jRequestObj["correctionTime"];
			gyroCorrectionDelay = jRequestObj["correctionDelay"];
			verboseMode = jRequestObj["verboseMode"];
		}
	}
}

void loop() {
	usleep(60000000);
}

int main() {
	system("clear");
    setup();
	pthread_t t_gyro;
	pthread_t t_console;
	pthread_t t_autocorrection;
	pthread_t t_uno1;
	pthread_t t_uno2;
	pthread_t t_udpserver;
	pthread_create(&t_gyro, NULL, readMPU, NULL);
	cout << "MPU6050 thread started[OK]" << endl;
	pthread_create(&t_uno1, NULL, readUno1, NULL);
	cout << "Arduino Uno1 thread started[OK]" << endl;
	pthread_create(&t_uno2, NULL, readUno2, NULL);
	cout << "Arduino Uno2 thread started[OK]" << endl;
	pthread_create(&t_console, NULL, consoleInput, NULL);
	cout << "Console input thread started[OK]" << endl;
	pthread_create(&t_autocorrection, NULL, gyroAutoCorrection, NULL);
	cout << "Gyro Auto Correction thread started[OK]" << endl;
	pthread_create(&t_udpserver, NULL, UDPServer, NULL);
	cout << "UDP Server thread started[OK]" << endl;
	pthread_detach(t_uno1);
	cout << "Arduino Uno1 thread detached[OK]" << endl;
	pthread_detach(t_uno2);
	cout << "Arduino Uno2 thread detached[OK]" << endl;
	pthread_detach(t_gyro);
	cout << "MPU6050 thread detached[OK]" << endl;
	pthread_detach(t_console);
	cout << "Console input thread detached[OK]" << endl;
	pthread_detach(t_autocorrection);
	cout << "Gyro Auto Correction thread started[OK]" << endl;
	pthread_detach(t_udpserver);
	cout << "UDP Server thread detached[OK]" << endl;
	cout << "Starting the main loop..." << endl;
    while(true) {
		loop();
	}
    return 0;
}

