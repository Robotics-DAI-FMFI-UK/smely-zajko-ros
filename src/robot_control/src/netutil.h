#ifndef __NET_H__
#define __NET_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int connect_to_server(char *server, int port);
int send_packet(int socket, uint8_t *buffer, unsigned int size);
int receive_packet(int socket, uint8_t *buffer, unsigned int maxsize);
int create_server(int port);
int wait_for_client_connection(int server_fd);

#endif

