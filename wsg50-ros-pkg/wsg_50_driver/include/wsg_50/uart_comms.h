#ifndef UART_COMMS_H
#define UART_COMMS_H

int client_connect(const char *server_name, int port, unsigned int speed);
int client_read(int sockfd, unsigned char* buf, int bytes);
int client_write(int sockfd, unsigned char* buf, int bytes);
int client_close(int sockfd);

#endif // UART_COMMS_H
