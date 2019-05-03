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
#include "ini.h"
#include "json.hpp"

#include "MPU6050.h"

#define MPU_COUNT 4
#define ICM_COUNT 4
#define EXPANDER_COUNT 4
#define EXPANDER_PIN_COUNT 16*EXPANDER_COUNT
#define EMERGENCY_STOP_PIN 2
#define ENDSTOP_COUNT 18


#define ENDSTOP_CZERWONA_STOPA_DOL 71
#define ENDSTOP_CZERWONA_STOPA_GORA 95

#define ENDSTOP_CZERWONA_KOLANO_DOL 67
#define ENDSTOP_CZERWONA_KOLANO_GORA 69

#define ENDSTOP_CZERWONA_NOGA_DOL 68
#define ENDSTOP_CZERWONA_NOGA_GORA 74

#define ENDSTOP_CZERWONA_UDO_DOSR 80
#define ENDSTOP_CZERWONA_UDO_ODSR 78


#define UDP_BUFFER_SIZE 4096
#define UDP_SERVER_PORT 11000

#define ENDSTOP_ZIELONA_STOPA_DOL 94
#define ENDSTOP_ZIELONA_STOPA_GORA 72

#define ENDSTOP_ZIELONA_KOLANO_DOL 65
#define ENDSTOP_ZIELONA_KOLANO_GORA 77

#define ENDSTOP_ZIELONA_NOGA_DOL 70
#define ENDSTOP_ZIELONA_NOGA_GORA 79

#define ENDSTOP_ZIELONA_UDO_DOSR 76
#define ENDSTOP_ZIELONA_UDO_ODSR 73



#define GYRO_CZERWONA_STOPA 5
#define GYRO_CZERWONA_KOLANO 6
#define GYRO_CZERWONA_NOGA 1

#define GYRO_ZIELONA_STOPA 7
#define GYRO_ZIELONA_KOLANO 4
#define GYRO_ZIELONA_NOGA 3

#define GYRO_MIEDNICA 0
#define GYRO_GORNE 2

#define MOTOR_COUNT 9

#define MOTOR_DIR_UP LOW
#define MOTOR_DIR_DOWN HIGH

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

mINI::INIFile configini("config.ini");
mINI::INIStructure iniconfig;

int sockfd;
char udpBuffer[UDP_BUFFER_SIZE];
struct sockaddr_in servaddr, cliaddr;

MPU6050 mpu[MPU_COUNT] {
{ 7, 0x68 },
{ 8, 0x68 },
{ 9, 0x68 },
{ 10, 0x68 },
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

bool programExit = false;

const uint8_t pinBase=65;
const uint8_t expanderAddr[EXPANDER_COUNT] = {
	0x20,
	0x21,
	0x24,
	0x22
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
	65,
	66,
	67,
	68,
	69,
	70,
	71,
	72,
	73,
	74,
	75,
	76,
	77,
	78,
	79,
	80,
	94,
	95
};

const uint8_t endstopMotor[MOTOR_COUNT][5]{
	{72,94,3,1,0},
	{79,70,4,1,7},
	{77,65,0,1,4},
	{76,73,4,0,7},
	{69,67,2,1,5},
	{95,71,1,1,2},
	{75,66,6,0,7},
	{74,68,5,1,7},
	{80,78,5,0,7}
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

float ypr[MPU_COUNT+ICM_COUNT][3];
float full_ypr[MPU_COUNT+ICM_COUNT][3];

int USB0, USB1;

void emergencyStop()
{
	stopped=true;
	for(uint8_t y=0;y<3;y++)
	{
		for(uint8_t x=0;x<MOTOR_COUNT;x++)
		{
			digitalWrite(motorPin[x][y], LOW);
		}
	}
	cout << "EMERGENCY_STOP!!!" << endl;
}

void setup() {
	configini.read(iniconfig);
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
	for(uint8_t x=pinBase;x<pinBase+EXPANDER_PIN_COUNT;x++)
	{
		for(uint8_t y=0;y<3;y++)
		{
			for(uint8_t x=0;x<MOTOR_COUNT;x++)
			{
				pinMode(motorPin[x][y], OUTPUT);
			}
		}
	}
	for(uint8_t x=0;x<ENDSTOP_COUNT;x++)
	{
		pinMode(endstopPin[x], INPUT);
		pullUpDnControl(endstopPin[x], PUD_UP);
		cout << "ENDSTOP[" << (int)endstopPin[x] << "] = " << (int)digitalRead(endstopPin[x]) << endl;
	}
	for(uint8_t y=0;y<3;y++)
	{
		for(uint8_t x=0;x<MOTOR_COUNT;x++)
		{
			digitalWrite(motorPin[x][y], LOW);
		}
	}
	if ((USB0 = serialOpen ("/dev/ttyUSB0", 115200)) < 0)
	{
		cout << "Unable to open serial device USB0";
		return;
	}
	if ((USB1 = serialOpen ("/dev/ttyUSB1", 115200)) < 0)
	{
		cout << "Unable to open serial device USB1";
		return;
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
			full_ypr[x][0] = ypr[x][0];
			full_ypr[x][1] = ypr[x][1];
			full_ypr[x][2] = ypr[x][2];
			pthread_mutex_unlock(&mutex_full_ypr);
			pthread_mutex_unlock(&mutex_ypr);
		}
	}
}

void *readUno(void *){
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
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+2][i-1] = atof(tokens[i].c_str());
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			}
			
			else if(tokens[0]=="#2"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+3][i-1] = atof(tokens[i].c_str());
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			}
			
		}
		else{
			buf1 += znak1;
		}
	}
}

void *readMega(void *){
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
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+0][i-1] = atof(tokens[i].c_str());
					pthread_mutex_unlock(&mutex_full_ypr);
			}
			}
			
			else if(tokens[0]=="#2"){
			for(unsigned int i = 1; i < tokens.size(); i++){
					pthread_mutex_lock(&mutex_full_ypr);
					if(tokens[i]!="nan")	full_ypr[MPU_COUNT+1][i-1] = atof(tokens[i].c_str());
					pthread_mutex_unlock(&mutex_full_ypr);
			}
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
		if(instr=="x"){
			cout << "PROGRAM EXIT...";
			emergencyStop();
			programExit=true;
		}
		if(instr.length()==2)
		{
			if(instr[0]=='u'){
				direction = MOTOR_DIR_UP;
			}
			else if(instr[0]=='d'){
				direction = MOTOR_DIR_DOWN;
			}
			else
			{
				direction = MOTOR_DIR_DOWN;
			}
			motorNum = (instr[1]-'0');
			if(stopped) continue;
			if(digitalRead(endstopMotor[motorNum][direction])==HIGH) continue;
			tmr.reset();
			try {
				if (digitalRead(endstopMotor[motorNum][direction])==LOW)
						{
							digitalWrite(motorPin[motorNum][1], direction);
							digitalWrite(motorPin[motorNum][0], HIGH);
						}
				uint8_t gyroid = endstopMotor[motorNum][2];
				uint8_t gyroid2 = endstopMotor[motorNum][4];
				uint8_t axisid = endstopMotor[motorNum][3];
				char dirid = (direction==MOTOR_DIR_DOWN) ? 'd' : 'u';
				string inireg = "motor" + to_string(motorNum) + "ypr" + to_string(axisid) + dirid;
				string inival = "0";
				while(stopped == false && tmr.elapsed()<0.01 && digitalRead(endstopMotor[motorNum][direction])==LOW)
				{
					float gyroval;
					//inival=iniconfig["gyro_endstop"][inireg];
					gyroval = (full_ypr[gyroid][axisid]) - (full_ypr[gyroid2][axisid]);
					if(inival!="")
					{
						if(abs(atof(inival.c_str())-gyroval)<5){
						cout << "TRIGERRED FROM GYRO" << endl;
						break;
						}
					}
					
				}
				digitalWrite(motorPin[motorNum][0], LOW);
				if(digitalRead(endstopMotor[motorNum][direction])==HIGH)
				{
					cout << "TRIGERRED FROM ENDSTOP" << endl;
					float gyroval;
					if(gyroid2==255){
						gyroval = full_ypr[gyroid][axisid];
					}
					else {
						gyroval = (full_ypr[gyroid][axisid] - full_ypr[gyroid2][axisid]);
					}
					cout << inireg;
					inival = to_string(gyroval);
					iniconfig["gyro_endstop"][inireg] = inival;
					configini.write(iniconfig);
				}
			}
			catch(string e){
				emergencyStop();
				cout << endl << "EXCEPTION: " << e;
			}
		}
		else if(instr.length()>=3)
		{
			string saveNum = "save" + instr.substr(1);
			if(instr[0]=='s'){
				for(uint8_t x=0;x<MPU_COUNT+ICM_COUNT;x++)
				{
					for(uint8_t y=0;y<3;y++)
					{
						string inireg = "gyro" + to_string(x) + "ypr" + to_string(y);
						string gyroVal = to_string(full_ypr[x][y]);
						iniconfig[saveNum][inireg] = gyroVal;
					}
				}
				configini.write(iniconfig);
			}
			if(instr[0]=='l'){
				if(iniconfig.has(saveNum))
				{
					
				}
				else{
					cout << "SAVE DOES NOT EXIST!" << endl;
				}
			}
			if(instr[0]=='g')
			{
				if(instr[1]=='f')
				{
					unsigned int motorNum = atoi(instr.substr(2).c_str());
					goToDestination[motorNum] = false;
				}
				else {
					unsigned int motorNum = instr[1]-'0';
					float deg = atof(instr.substr(2).c_str());
					destinations[motorNum] = deg;
					goToDestination[motorNum] = true;
				}
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
				gyroval[motorNum] = (full_ypr[gyroid[motorNum]][axisid[motorNum]]) - (full_ypr[gyroid2[motorNum]][axisid[motorNum]]);
				if(abs(destinations[motorNum]-gyroval[motorNum])>5)
				{
					if(destinations[motorNum]-gyroval[motorNum]<0){
					direction[motorNum] = MOTOR_DIR_UP;
					cout << "UP" << gyroval[motorNum] << endl;
					}
					else if(destinations[motorNum]-gyroval[motorNum]>0){
						direction[motorNum] = MOTOR_DIR_DOWN;
						cout << "DOWN" << gyroval[motorNum] << endl;
					}
					try {
						if (digitalRead(endstopMotor[motorNum][direction[motorNum]])==LOW)
						{
							digitalWrite(motorPin[motorNum][1], direction[motorNum]);
							digitalWrite(motorPin[motorNum][0], HIGH);
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
		while(stopped == false && tmr.elapsed()<0.01)
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
			digitalWrite(motorPin[motorNum][0], LOW);
			digitalWrite(motorPin[motorNum][1], LOW);
		}
		while(tmr2.elapsed()<1);
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
				degs[motorNum] = (full_ypr[gyroid][axisid]) - (full_ypr[gyroid2][axisid]);
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
		else if(type==4)
		{
			cout << "PROGRAM EXIT...";
			emergencyStop();
			programExit=true;
			exit(0);
		}
	}
}

void loop() {
	/*for(uint8_t x=0;x<MPU_COUNT+ICM_COUNT;x++)
	{
		cout << "[" << (int)x << "]" << "     " << full_ypr[x][0] << " " << full_ypr[x][1] << " " << full_ypr[x][2] << "   ";
	}
	cout << endl;
	delay(100);*/
}

int main() {
	system("clear");
    setup();
	pthread_t t_gyro;
	pthread_t t_console;
	pthread_t t_autocorrection;
	pthread_t t_uno;
	pthread_t t_mega;
	pthread_t t_udpserver;
	pthread_create(&t_gyro, NULL, readMPU, NULL);
	cout << "MPU6050 thread started[OK]" << endl;
	pthread_create(&t_uno, NULL, readUno, NULL);
	cout << "Arduino Uno thread started[OK]" << endl;
	pthread_create(&t_mega, NULL, readMega, NULL);
	cout << "Arduino Mega thread started[OK]" << endl;
	pthread_create(&t_console, NULL, consoleInput, NULL);
	cout << "Console input thread started[OK]" << endl;
	pthread_create(&t_autocorrection, NULL, gyroAutoCorrection, NULL);
	cout << "Gyro Auto Correction thread started[OK]" << endl;
	pthread_create(&t_udpserver, NULL, UDPServer, NULL);
	cout << "UDP Server thread started[OK]" << endl;
	pthread_detach(t_uno);
	cout << "MPU6050 thread detached[OK]" << endl;
	pthread_detach(t_mega);
	cout << "Arduino Uno thread detached[OK]" << endl;
	pthread_detach(t_gyro);
	cout << "Arduino Mega thread detached[OK]" << endl;
	pthread_detach(t_console);
	cout << "Console input thread detached[OK]" << endl;
	pthread_detach(t_autocorrection);
	cout << "Gyro Auto Correction thread started[OK]" << endl;
	pthread_detach(t_udpserver);
	cout << "UDP Server thread detached[OK]" << endl;
	cout << "Starting the main loop..." << endl;
    while(true) {
		if(programExit) return 0;
		loop();
	}
    return 0;
}

