#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netdb.h>
#include <vector>
#include "TCPServer.h"

using namespace std;

namespace TCP{
class TCPClient
{
  public:
    TCPClient();
    bool setup(std::string address, int port);
    bool sendStr(std::string data);
    std::string receive(int size = 4096);
    std::string read();
    void start();
    void addRecvCallback(funcPtrRecvCallback f); 
    void closeClient();

  private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;
    bool isServerConnected = false;
    char msg[ MAX_PACKET_SIZE ];
    funcPtrRecvCallback recvCallback = 0;  
};
}