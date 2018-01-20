// ==============================================================
//                  ORBITER MODULE: Rcontrol
//
// UDPserver.cpp
//
// A plugin which enables the different thrusters of the vessels
// to be operated remotely
// ==============================================================

#include "UDPserver.h"
#include<iostream>

// Set up server connection, the server address
// will be passed to the program as an argument
UDPserver::UDPserver(std::string server_addr)
{
  serverlen = sizeof(server);
  port = PORT;
  sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sockfd < 0)
    error("ERROR: Could not create SOCKET connection");
  //serv_addr = server_addr.c_str();
  //printf("The address is %s\n", server_addr);
  std::cout << "The address is " << server_addr << std::endl;
  memset(&server, 0, serverlen);
  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  server.sin_addr.s_addr = htonl(INADDR_ANY);

  //std::cout << "The char version of the address is " << serv_addr << std::endl;
  if (bind(sockfd, (struct sockaddr *) &server,
           sizeof(server)) < 0)
    error("ERROR: Could not bind");
}

// Spawn a seperate data process for every connection
// made to the UDP server
void UDPserver::spawn_data_process(const char *data)
{
  listen(sockfd, 5);
  cli_len = sizeof(cli_addr);
  newsocket = sockfd;
  while(1) {
    pid = fork();
    if (pid < 0)
      error("ERROR on fork");
    if (pid == 0) {
      //xperform_transfer(sockfd, data);
      exit(0);
    }
    else
      close(newsocket);
    sleep(1);
  }
}

// There is a seperate instance of this function
// for every connection made. It handles the communication
// once a connection has been established.
void UDPserver::perform_transfer(const char *data, int debug)
{
  int n;
  cli_len = sizeof(cli_addr);

  // By default zero the buffer
  bzero(buffer, BUFLEN);
  memset(&buffer, 0, sizeof(buffer));
  // If the size of the data is greater than zero,
  // copy into the buffer
  //  const char* char_data = data.c_str()
   if (sizeof(data))
     strcpy(buffer, data);
  // Request the data transaction from the client
  printf("Requesting data from client....\n");
  if (debug) {
    printf("Socket = %d\n", sockfd);
    printf("cli_addr.sin_addr.s_addr = %d\n", cli_addr.sin_addr.s_addr);

  }
  n = sendto(sockfd, &buffer, sizeof(buffer), 0, (struct sockaddr *)&server, sizeof(server));
  if (n < 0) error("ERROR writing to socket");
  // zero the buffer again
  bzero(buffer, BUFLEN);
  // Read the contents of the message into the buffer
  n = recvfrom(sockfd, buffer, BUFLEN-1, 0, (struct sockaddr *)&cli_addr, &cli_len);
  if (n < 0) error("ERROR reading from socket");
  printf("Received packet from %s:%d\nData: %s\n\n", inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buffer);
}
