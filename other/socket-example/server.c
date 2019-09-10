#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>

#include "net.h"

int main()
{
    int server_fd = create_server(1234);
    if (!server_fd)
    {
        printf("could not create server\n");
        return 0;
    }

    int shutdown = 0;
    do {
        int fd = wait_for_client_connection(server_fd);
    
        char buffer[1024] = {0};
        char *hello = "Hello from server";
    
        int msg_number = 0;
    
        do {
    
            if (!receive_packet(fd, buffer, 1024))
            {
                printf("could not receive packet\n");
	        break;
            }
            if (strcmp(buffer, "shutdown") == 0) shutdown = 1;
    
            printf("client said: %s\n", buffer);
    
            snprintf(buffer, 1024, "%s %d", hello, msg_number++);
            if (!send_packet(fd, buffer, strlen(buffer) + 1))
            {
                printf("could not send packet\n");
                close(fd);
                close(server_fd);
            }
            printf("Hello message sent\n");
    
        } while (!shutdown);
    
        close(fd);

    } while (!shutdown);
    close(server_fd);

    return 0;
}

