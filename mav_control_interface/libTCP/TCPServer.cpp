#include "TCPServer.h" 
namespace TCP{
// void* TCPServer::Task(void *arg)
// {
// 	int n;
// 	int newsockfd=*(int *)arg;
// 	char msg[MAX_PACKET_SIZE];
// 	pthread_detach(pthread_self());
// 	std::cout << "enter TCP thread" << std::endl;	
// 	while(1)
// 	{
// 		n=recv(newsockfd,msg,MAX_PACKET_SIZE,MSG_DONTWAIT);
// 		if(n==0)
// 		{
// 			std::cout << "Connection " << newsockfd << " closed" << endl;		
// 			isClientConnected = false;   
// 			break;
// 		}
// 		msg[n]=0;
// 		//send(newsockfd,msg,n,0);
// 		//Message = string(msg);
// 		usleep(1000);
//     }
// 	return 0;
// }

void TCPServer::setup(int port)
{
	sockfd=socket(AF_INET,SOCK_STREAM,0);
	//sockfd=socket(AF_INET,SOCK_DGRAM,0);
 	memset(&serverAddress,0,sizeof(serverAddress));
	serverAddress.sin_family=AF_INET;
	serverAddress.sin_addr.s_addr=htonl(INADDR_ANY);
	serverAddress.sin_port=htons(port);
	bind(sockfd,(struct sockaddr *)&serverAddress, sizeof(serverAddress));
 	listen(sockfd,5);
}

std::string TCPServer::start()
{
	std::string str;
	pthread_detach(pthread_self());	
	while(1)
	{
		std::cout << "Wait for connection" << std::endl;
		socklen_t sosize  = sizeof(clientAddress);		
		newsockfd = accept(sockfd,(struct sockaddr*)&clientAddress,&sosize);
     	if (newsockfd < 0)
		{ 
          	std::cout << "ERROR on accept" << std::endl;
			continue;
		}
		str = inet_ntoa(clientAddress.sin_addr);
		std::cout << "Connection " << newsockfd << " established" << std::endl;
		//pthread_create(&serverThread,NULL,&Task,(void *)&newsockfd);
		isClientConnected = true;   
		while(1)
		{
			int n;
			n = recv(newsockfd,msg,MAX_PACKET_SIZE,MSG_DONTWAIT);
			if (n > 0)// receive msg from client
			{
				msg[n]=0;
				//printf("%s\n", msg);				
				if (recvCallback)
				{
					std::string recvStr;
					recvStr = msg;
					recvCallback(recvStr);
				}
			}
			else if (n == 0)
			{
				std::cout << "Connection " << newsockfd << " closed" << std::endl;		
				isClientConnected = false;   
				break;
			}
			else if (n < 0)/* error*/
			{
				switch (errno)
				{
					case EWOULDBLOCK:
						continue;
					default:
						break;	
				}
			}
			usleep(1000);
		}		
	}
	return str;
}

void TCPServer::sendStr(std::string msg)
{
	if (isClientConnected)	
		send(newsockfd,msg.c_str(),msg.length(),0);
}

void TCPServer::sendBuff(const char *msgBuff, int msgLen)
{
	if (isClientConnected)	
		send(newsockfd, msgBuff, msgLen, MSG_NOSIGNAL);	
}

void TCPServer::clean()
{
	memset(msg, 0, MAX_PACKET_SIZE);
}

void TCPServer::closeServer()
{
	close(sockfd);
	close(newsockfd);
}

void TCPServer::addRecvCallback(funcPtrRecvCallback f)
{
	recvCallback = f;
}
}