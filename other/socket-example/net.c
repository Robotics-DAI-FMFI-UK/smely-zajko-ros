#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

int connect_to_server(char *server, int port)
{
    int sock;
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    { 
        perror("socket error"); 
        return 0; 
    } 
    printf("socket created\n");

    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    int convert_result = inet_pton(AF_INET, server, &serv_addr.sin_addr);
    if (convert_result == 0)
    {
        printf("address %s not recognized\n", server);
	return 0;
    }
    else if (convert_result < 0)
    { 
        perror("server address error"); 
        return 0; 
    } 
    printf("address converted\n");
   
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        perror("connect failed"); 
        return 0; 
    } 
    printf("server connected\n");

    return sock;
}

int send_packet(int socket, uint8_t *buffer, unsigned int size)
{
    unsigned int count = size;
    uint8_t len[sizeof(unsigned int)];
    for (int i = 0; i < sizeof(unsigned int); i++)
    {
        len[i] = count & 255;
	count >>= 8;
    }

    if (send(socket, len, sizeof(unsigned int), 0) < 0)
        return 0;

    if (send(socket, buffer, size, 0) < 0)
        return 0;

    return 1;
}

int receive_packet(int socket, uint8_t *buffer, unsigned int maxsize)
{
    uint8_t len[sizeof(unsigned int)];
    int i = 0;
    int size = 0;

    while (i < sizeof(unsigned int))
    {
        int nread = recv(socket, len + i, sizeof(unsigned int) - i, 0);
        if (nread <= 0) return 0;
	i += nread;
    }
    while (i--)
    {
        size <<= 8;
	size += len[i];
    }
    if (size > maxsize) return 0;

    i = 0;
    while (i < size)
    {
        int nread = recv(socket, buffer + i, size - i, 0);
	if (nread <= 0) return 0;
	i += nread;
    }
    return 1;
}

int create_server(int port)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == 0) 
    {
        perror("socket failed");
        return 0;
    }
    printf("socket created\n");

    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
	return 0;
    }
    printf("options set\n");
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) 
    { 
        perror("bind failed"); 
	return 0;
    } 
    printf("socket bound to address\n");
    return server_fd;
}

int wait_for_client_connection(int server_fd)
{
    printf("listening...\n");
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
	return 0;
    }  
    printf("accepting...\n");
    int new_socket;
    if ((new_socket = accept(server_fd, 0, 0)) < 0)
    {
        perror("accept");
	return 0;
    }
    return new_socket;
}

