#include "TCPClient.h"
namespace TCP
{
TCPClient::TCPClient()
{
    sock = -1;
    port = 0;
    address = "";
}

bool TCPClient::setup(std::string address, int port)
{
    if (sock == -1)
    {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock == -1)
        {
            std::cout << "Could not create socket" << std::endl;
        }
    }
    if (inet_addr(address.c_str()) == -1)
    {
        struct hostent *he;
        struct in_addr **addr_list;
        if ((he = gethostbyname(address.c_str())) == NULL)
        {
            herror("gethostbyname");
            std::cout << "Failed to resolve hostname\n";
            return false;
        }
        addr_list = (struct in_addr **)he->h_addr_list;
        for (int i = 0; addr_list[i] != NULL; i++)
        {
            server.sin_addr = *addr_list[i];
            break;
        }
    }
    else
    {
        server.sin_addr.s_addr = inet_addr(address.c_str());
    }
    server.sin_family = AF_INET;
    server.sin_port = htons(port);
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        perror("connect failed. Error");
        isServerConnected = false;
        return false;
    }
    isServerConnected = true;
    return true;
}

bool TCPClient::sendStr(std::string data)
{
    if (send(sock, data.c_str(), strlen(data.c_str()), 0) < 0)
    {
        std::cout << "Send failed : " << data << std::endl;
        return false;
    }
    return true;
}

std::string TCPClient::receive(int size)
{
    char buffer[size];
    std::string reply;
    if (recv(sock, buffer, size, 0) < 0) // sizeof(buffer)
    {
        std::cout << "receive failed!" << std::endl;
    }
    buffer[size] = '\0';
    reply = buffer;
    return reply;
}

std::string TCPClient::read()
{
    char buffer[1] = {};
    std::string reply;
    while (buffer[0] != '\n')
    {
        if (recv(sock, buffer, sizeof(buffer), 0) < 0)
        {
            std::cout << "receive failed!" << std::endl;
            break;
        }
        reply += buffer[0];
    }
    return reply;
}

void TCPClient::start()
{
    std::string str;
    pthread_detach(pthread_self());

    while (1)
    {
        int n;
        n = recv(sock, msg, MAX_PACKET_SIZE, MSG_DONTWAIT);
        if (n > 0)
        {
            msg[n] = 0;
            //std::cout << msg << std::endl;
            if (recvCallback)
            {
                std::string recvStr;
                recvStr = msg;
                recvCallback(recvStr);
            }
        }
        else if (n == 0)
        {
            std::cout << "Connection " << sock << " closed" << std::endl;
            isServerConnected = false;
            break;
        }
        else if (n < 0) /* error*/
        {
            switch (errno)
            {
            case EWOULDBLOCK:
                continue;
            default:
                break;
            }
        }
        usleep(200);
    }
}

void TCPClient::closeClient()
{
    close(sock);
}

void TCPClient::addRecvCallback(funcPtrRecvCallback f)
{
    recvCallback = f;
}
} // namespace TCP