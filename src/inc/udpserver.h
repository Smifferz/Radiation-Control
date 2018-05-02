// ==============================================================
//                  ORBITER MODULE: Rcontrol
//
// UDPserver.h
//
// A plugin which enables the different thrusters of the vessels
// to be operated remotely
// ==============================================================

#ifndef UDPSERVER_H
#define UDPSERVER_H

# ifdef _WIN32
# include <Windows.h>
# include <stdio.h>
# include <tchar.h>
# endif
# ifdef linux
# include<stdio.h>
# include<arpa/inet.h>
# include<unistd.h>
# include<sys/socket.h>
# include<sys/types.h>
# endif
#include<stdio.h>
#include<string.h>
#include<string>
#include<cstring>
#include<stdlib.h>
#include "types.h"

//#pragma comment(lib,"ws2_32.lib") // Winsock library

#define BUFLEN 1024 // Max length of buffer
#define PORT 8888 // The port on which to listen to incoming data

// socklen_t is part of unistd.h so needs to be created for windows
# ifdef _WIN32
typedef int socklen_t;
# endif

// The server will change depending on the system,
// pass the server as an argument to the program
//#define SERVER "192.168.56.101"

class UDPserver
{
public:
  UDPserver(std::string server_addr, int debug_tmp);
# ifdef _WIN32
  ~UDPserver() { closesocket(socketS); }
# else
  ~UDPserver() { close(sockfd); close(newsocket); }
# endif
  bool check_ping();
  void transfer_data(std::string operation, std::string detail);
  void transfer_data(std::string operation, std::string detail, v3 *result);
  void transfer_data(std::string operation, std::string detail, int *result);
  void transfer_data(std::string operation, std::string detail, double *result);
private:
  int debug;
  void error(const char *msg) { perror(msg); exit(EXIT_FAILURE); }
  //void perform_transfer(const char *data);
  int port, sockfd, newsocket, serverlen, pid;
# ifdef _WIN32
  SOCKET socketS;
# endif
  socklen_t cli_len;
  struct sockaddr_in server, cli_addr;
  char buffer[BUFLEN];
  const char *serv_addr;
};

#endif //UDPSERVER_h
