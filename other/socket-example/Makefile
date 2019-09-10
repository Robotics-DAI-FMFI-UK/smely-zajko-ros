all:	server client

server:	server.c net.c net.h
	gcc -o server server.c net.c

client: client.c net.c net.h
	gcc -o client client.c net.c

clean:
	rm -f server client
