// ==============================================================
//                  ORBITER MODULE: Rcontrol
//
// UDPserver.h
//
// A plugin which enables the different thrusters of the vessels
// to be operated remotely
// ==============================================================

#pragma once

#include <stdio.h>
#include<arpa/inet.h>
#include<unistd.h>
#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<string>
#include<cstring>
#include<stdlib.h>
#include "types.h"

//#pragma comment(lib,"ws2_32.lib") // Winsock library

#define BUFLEN 512 // Max length of buffer
#define PORT 8888 // The port on which to listen to incoming data

// The server will change depending on the system,
// pass the server as an argument to the program
//#define SERVER "192.168.56.101"

class UDPserver
{
public:
  UDPserver(std::string server_addr);
  ~UDPserver() { close(sockfd); close(newsocket); }
  void spawn_data_process(const char *data);
  void perform_transfer(int data, int info, int debug);
  void perform_transfer(int data, int info, int debug, VECTOR3 result);
  void perform_transfer(int data, int info, int debug, double result);
private:
  void error(const char *msg) { perror(msg); exit(EXIT_FAILURE); }
  //void perform_transfer(const char *data);
  int port, sockfd, newsocket, serverlen, pid;
  socklen_t cli_len;
  struct sockaddr_in server, cli_addr;
  char buffer[BUFLEN];
  const char *serv_addr;
};
