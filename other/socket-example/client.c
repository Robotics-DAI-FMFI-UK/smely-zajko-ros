#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include "net.h"

#define SERVER_IP	"127.0.0.1"
#define SERVER_PORT     1234

int main()
{
    int socket = connect_to_server(SERVER_IP, SERVER_PORT);
    if (socket == 0)
    {
        printf("connect to server failed\n");
	return 0;
    }

    char *hello = "Hello from client";
    char buffer[1024];
    strcpy(buffer, hello);
      
    do {
        if (send_packet(socket, buffer, strlen(buffer) + 1) == 0)
        {
            printf("could not send packet\n");
	    close(socket);
	    return 0;
        }

        printf("message sent\n"); 
        if (strcmp(buffer, "shutdown") == 0) break;

        if (receive_packet(socket, buffer, 1024) == 0)
	{
	    printf("could not receive packet\n");
	    close(socket);
	    return 0;
	}
	printf("response: %s\n---\n", buffer);

        printf("enter quit, shutdown, or a new message: ");
        fgets(buffer, 1024, stdin);
	buffer[strlen(buffer) - 1] = 0;

    } while (strcmp(buffer, "quit") != 0);

    close(socket);
    return 0; 
}

