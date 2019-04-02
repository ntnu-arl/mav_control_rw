#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>

using namespace std;

namespace TCP{

static constexpr unsigned long MAX_PACKET_SIZE = 2048;
typedef bool (*funcPtrRecvCallback)(std::string);

class TCPServer
{
	public:
	void setup(int port);
	std::string start();
	void sendStr(std::string msg);
	void sendBuff(const char *msgBuff, int msgLen);	
	void closeServer();
	void clean();
	void addRecvCallback(funcPtrRecvCallback f);

	private:
	int sockfd, newsockfd, n, pid;
	struct sockaddr_in serverAddress;
	struct sockaddr_in clientAddress;
	pthread_t serverThread;
	char msg[ MAX_PACKET_SIZE ];
    funcPtrRecvCallback recvCallback = 0;
	//static void * Task(void * argv);
	bool isClientConnected;
};
}