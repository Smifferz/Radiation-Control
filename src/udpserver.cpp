// ==============================================================
//                  ORBITER MODULE: Rcontrol
//
// UDPserver.cpp
//
// A plugin which enables the different thrusters of the vessels
// to be operated remotely
// ==============================================================

#include "udpserver.h"
#include <iostream>
#include <cstdio>

// Set up server connection, the server address
// will be passed to the program as an argument
UDPserver::UDPserver(std::string server_addr, int debug_tmp)
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    serverlen = sizeof(server);
    port = PORT;
#ifdef _WIN32
    socketS = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketS < 0) {
        error("ERROR: Could not create SOCKET connection");
    }
#else
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd < 0) {
        error("ERROR: Could not create SOCKET connection");
    }
#endif
  //serv_addr = server_addr.c_str();
  //printf("The address is %s\n", server_addr);
  std::cout << "The address is " << server_addr << std::endl;
  memset(&server, 0, serverlen);
  server.sin_family = AF_INET;
  server.sin_port = htons(port);
#ifdef _WIN32
  server.sin_addr.s_addr = INADDR_ANY;
#else
  server.sin_addr.s_addr = htonl(INADDR_ANY);
#endif

  //std::cout << "The char version of the address is " << serv_addr << std::endl;
#ifdef _WIN32
  /*if (bind(socketS, (sockaddr*)&server, sizeof(server))) {
      error("ERROR: Could not bind");
  }*/
  bind(socketS, (sockaddr*)&server, sizeof(server));
#else
  if (bind(sockfd, (struct sockaddr *) &server, sizeof(server)) < 0) {
      error("ERROR: Could not bind");
  }
#endif
  debug = debug_tmp;
  cli_len = sizeof(struct sockaddr);
}

bool UDPserver::check_ping()
{
  int ping;
#ifdef _WIN32
  ping = recvfrom(socketS, buffer, BUFLEN - 1, 0, (sockaddr*)&cli_addr, &cli_len);
#else
  ping = recvfrom(sockfd, buffer, BUFLEN - 1, 0, (struct sockaddr *)&cli_addr, &cli_len);
#endif
  if (ping < 0) error("ERROR reading from client");
  std::cout << "Received " << buffer << ", now returning ping..." << std::endl;
  if (debug) {
    std::cout << "The address of the client is " << inet_ntoa(cli_addr.sin_addr) << std::endl;
    std::cout << "The length of the client " << cli_len << std::endl;
  }
#ifdef _WIN32
  ping = sendto(socketS, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#else
  ping = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#endif
  if (ping < 0) error("ERROR writing to client");
  return true;
}

// for every connection made. It handles the communication
// once a connection has been established.
void UDPserver::transfer_data(std::string operation, std::string detail)
{
  int n;
  cli_len = sizeof(cli_addr);

  memset(&buffer, 0, sizeof(buffer));

  // If the size of the data is greater than zero,
  // copy into the buffer
  // Build up the JSON string
  std::string jsonbuffer = "{\"operation\":\"" + operation + "\",\"detail\":" + detail + "}";
  strcpy(buffer,jsonbuffer.c_str());
  // Request the data transaction from the client
  if (debug) {
    printf("Requesting data from client....\n");
    printf("Socket = %d\n", sockfd);
    printf("cli_addr.sin_addr.s_addr = %d\n", cli_addr.sin_addr.s_addr);
    printf("Attempting to write to socket...\n");
    std::cout << "Writing data value : " << buffer << " to client" << std::endl;
  }
#ifdef _WIN32
  n = sendto(socketS, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#else
  n = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#endif
  if (n < 0) error("ERROR writing to socket");

  // zero the buffer again
  memset(buffer, 0, BUFLEN);
  // Read the contents of the message into the buffer
#ifdef _WIN32
  n = recvfrom(socketS, buffer, BUFLEN - 1, 0, (sockaddr*)&cli_addr, &cli_len);
#else
  n = recvfrom(sockfd, buffer, BUFLEN - 1, 0, (struct sockaddr *)&cli_addr, &cli_len);
#endif
}




// There is a seperate instance of this function
// for every connection made. It handles the communication
// once a connection has been established.
void UDPserver::transfer_data(std::string operation, std::string detail, int *result)
{
  int n;
  cli_len = sizeof(cli_addr);

  memset(&buffer, 0, sizeof(buffer));

  // If the size of the data is greater than zero,
  // copy into the buffer
  // Build up the JSON string
  std::string jsonbuffer = "{\"operation\":\"" + operation + "\",\"detail\":" + detail + "}";
  strcpy(buffer,jsonbuffer.c_str());
  // Request the data transaction from the client
  if (debug) {
    printf("Requesting data from client....\n");
    printf("Socket = %d\n", sockfd);
    printf("cli_addr.sin_addr.s_addr = %d\n", cli_addr.sin_addr.s_addr);
    printf("Attempting to write to socket...\n");
    std::cout << "Writing data value : " << buffer << " to client" << std::endl;
  }
#ifdef _WIN32
  n = sendto(socketS, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#else
  n = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#endif
  if (n < 0) error("ERROR writing to socket");

  // zero the buffer again
  memset(buffer, 0, BUFLEN);
  // Read the contents of the message into the buffer
#ifdef _WIN32
  n = recvfrom(socketS, buffer, BUFLEN - 1, 0, (sockaddr*)&cli_addr, &cli_len);
#else
  n = recvfrom(sockfd, buffer, BUFLEN - 1, 0, (struct sockaddr *)&cli_addr, &cli_len);
#endif
  if (n < 0) error("ERROR reading from socket");
  if (debug)
    printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buffer);
  // Store the data into the result vector using the array data interface
  *result = atof(buffer);
}

// There is a seperate instance of this function
// for every connection made. It handles the communication
// once a connection has been established.
void UDPserver::transfer_data(std::string operation, std::string detail, double *result)
{
  int n;
  cli_len = sizeof(cli_addr);

  memset(&buffer, 0, sizeof(buffer));

  // If the size of the data is greater than zero,
  // copy into the buffer
  // Build up the JSON string
  std::string jsonbuffer = "{\"operation\":\"" + operation + "\",\"detail\":" + detail + "}";
  strcpy(buffer,jsonbuffer.c_str());
  // Request the data transaction from the client
  if (debug) {
    printf("Requesting data from client....\n");
    printf("Socket = %d\n", sockfd);
    printf("cli_addr.sin_addr.s_addr = %d\n", cli_addr.sin_addr.s_addr);
    printf("Attempting to write to socket...\n");
    std::cout << "Writing data value : " << buffer << " to client" << std::endl;
  }
#ifdef _WIN32
  n = sendto(socketS, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#else
  n = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#endif
  if (n < 0) error("ERROR writing to socket");

  // zero the buffer again
  memset(buffer, 0, BUFLEN);
  // Read the contents of the message into the buffer
#ifdef _WIN32
  n = recvfrom(socketS, buffer, BUFLEN - 1, 0, (sockaddr*)&cli_addr, &cli_len);
#else
  n = recvfrom(sockfd, buffer, BUFLEN - 1, 0, (struct sockaddr *)&cli_addr, &cli_len);
#endif
  if (n < 0) error("ERROR reading from socket");
  if (debug)
    printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buffer);
  // Store the data into the result vector using the array data interface
  *result = atof(buffer);
}




// There is a seperate instance of this function
// for every connection made. It handles the communication
// once a connection has been established.
void UDPserver::transfer_data(std::string operation, std::string detail, v3 *result)
{
  int n;
  cli_len = sizeof(cli_addr);

  // zero the buffer
  memset(&buffer, 0, sizeof(buffer));

  // If the size of the data is greater than zero,
  // copy into the buffer
  // Build up the JSON string
  std::string jsonbuffer = "{\"operation\":\"" + operation + "\",\"detail\":" + detail + "}";
  strcpy(buffer,jsonbuffer.c_str());
  // Request the data transaction from the client
  //printf("Requesting data from client....\n");
  if (debug) {
    printf("Requesting data from client....\n");
    printf("Socket = %d\n", sockfd);
    printf("cli_addr.sin_addr.s_addr = %d\n", cli_addr.sin_addr.s_addr);
    printf("Attempting to write to socket...\n");
    std::cout << "Writing data value : " << buffer << " to client" << std::endl;
  }
#ifdef _WIN32
  n = sendto(socketS, buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#else
  n = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&cli_addr, cli_len);
#endif
  if (n < 0) error("ERROR writing to socket");

  // Expect three responses from the client
  for (int i = 0; i < 3; i++) {
    // zero the buffer again
    memset(buffer, 0, BUFLEN);
    // Read the contents of the message into the buffer
#ifdef _WIN32
    n = recvfrom(socketS, buffer, BUFLEN - 1, 0, (sockaddr*)&cli_addr, &cli_len);
#else
    n = recvfrom(sockfd, buffer, BUFLEN - 1, 0, (struct sockaddr *)&cli_addr, &cli_len);
#endif
    if (n < 0) error("ERROR reading from socket");
    if (debug)
      printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buffer);
    // Store the data into the result vector using the array data interface
    result->data[i] = atof(buffer);
  }
}
