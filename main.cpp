#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <cstring>
#include <unistd.h>
#include <signal.h>
#include <mutex>

#define BUFFER_SIZE 128
#define BAUDRATE B115200

int fd = -1;
auto portName = "/dev/ttyACM10";

std::mutex mutex;
std::string lastLineSent = "";

void cleanup(){

	std::cout << "Cleanup" << std::endl;

	if(fd > 0)
		close(fd);
	fd = -1;

	auto sysCom = std::string("sudo rm ") + portName;
	system(sysCom.c_str());
}

// Define the function to be called when ctrl-c (SIGINT) signal is sent to process
void sigintCallback(int signum){
	cleanup();
	exit(0);
}

void* readInThread(void* pointer) {
	int fd = *(int*)pointer;
	char inputbyte;
	std::string curLine;
	while (read(fd, &inputbyte, 1) == 1) {
		curLine += inputbyte;
		if(inputbyte == '\n'){
			if(curLine == "\n"){
				curLine = "";
				continue;
			}
//			bool printLine = true;
//			mutex.lock();
//			if(curLine == lastLineSent) printLine = false;
//			mutex.unlock();
//			if(printLine)
			std::cout << "IN: " << curLine;
			curLine = "";
		}
	}
	return 0;
}


int main(int argc, char *argv[]){

	// Register signal and signal handler
	signal(SIGINT, sigintCallback);

	fd = open("/dev/ptmx", O_RDWR | O_NOCTTY);
	if (fd == -1) {
		std::cerr << "error opening virtual port." << std::endl;
		return -1;
	}

	grantpt(fd);
	unlockpt(fd);

	char* pts_name = ptsname(fd);
	//std::cerr << "ptsname: " << pts_name << std::endl;
	std::cerr << "Port name: " << portName << std::endl;
	std::cout << "Type in text to send:" << std::endl << std::endl;

	//auto sysCom = std::string("chmod 777 ") + portName;
	auto sysCom = std::string("sudo ln -s ") + pts_name + " " + portName;
	system(sysCom.c_str());

	/* serial port parameters */
	/*
	struct termios newtio;
	memset(&newtio, 0, sizeof(newtio));
	struct termios oldtio;
	tcgetattr(fd, &oldtio);

	newtio = oldtio;
	newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR | ICRNL;
	newtio.c_oflag = 0;
	newtio.c_lflag = ICANON;
	newtio.c_cc[VMIN] = 1;
	newtio.c_cc[VTIME] = 0;
	tcflush(fd, TCIFLUSH);

	cfsetispeed(&newtio, BAUDRATE);
	cfsetospeed(&newtio, BAUDRATE);
	tcsetattr(fd, TCSANOW, &newtio);
	*/

	struct termios settings;
	tcgetattr(fd, &settings);

	cfsetispeed(&settings, BAUDRATE);
	cfsetospeed(&settings, BAUDRATE); /* baud rate */
	settings.c_cflag &= ~PARENB; /* no parity */
	settings.c_cflag &= ~CSTOPB; /* 1 stop bit */
	settings.c_cflag &= ~CSIZE;
	settings.c_cflag |= CS8 | CLOCAL; /* 8 bits */
	settings.c_lflag = ICANON; /* canonical mode */
	settings.c_oflag &= ~OPOST; /* raw output */

	tcsetattr(fd, TCSANOW, &settings); /* apply the settings */
	tcflush(fd, TCOFLUSH);

	/* start reader thread */
	pthread_t thread;
	pthread_create(&thread, 0, readInThread, (void*)&fd);

	/* read from stdin and send it to the serial port */
	std::string curLine;
	char c;
	while (true) {
		c = std::cin.get();
		if(c == '\n'){
			if(curLine == "\n"){
				curLine = "";
				continue;
			}
			curLine += c;
			mutex.lock();
			lastLineSent = curLine;
			mutex.unlock();
			write(fd, curLine.c_str(), curLine.size());
			curLine = "";
		}
		curLine += c;
	}

	cleanup();

	return 0;
}
