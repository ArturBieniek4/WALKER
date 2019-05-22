all: main

HDRS = helper_3dmath.h I2Cdev.h MPU6050.h
CMN_OBJS = I2Cdev.o MPU6050.o
MAIN_OBJS = main.o

# Set DMP FIFO rate to 10Hz

CXXFLAGS = -Wall -g -lwiringPi -pthread

$(CMN_OBJS) $(MAIN_OBJS) : $(HDRS)

main: $(CMN_OBJS) $(MAIN_OBJS)
	$(CXX) -o $@ $^ -l wiringPi -pthread -lm

clean:
	rm -f $(CMN_OBJS) $(MAIN_OBJS) main
