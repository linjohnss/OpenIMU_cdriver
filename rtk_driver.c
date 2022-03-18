#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include "driver.h"

int main(int argc, int **argv) {
    int serial_port = open("/dev/ttyUSB0", O_RDWR);

    if (serial_port <0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ISIG;
    tty.c_oflag &= ~ONLCR;
    tty.c_cc[VTIME] = 10;
    tty.c_cc[VMIN] = 0;
    cfsetispeed(&tty, B460800); // RTK bunrate must be B460800
    cfsetospeed(&tty, B460800);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    int8_t head;
    int16_t p_type;
    int8_t length = 0;
    rtkDataPointer result;
    result = (rtkDataPointer)malloc(sizeof(rtkDataPointer*));
    int n = 0;

    while(1) {
        while((n = read(serial_port, &head, sizeof(head))) > 0) {
            if(head == HEADER) {
                if(n = read(serial_port, &head, sizeof(head)) > 0) {
                    if(head == HEADER) {
                        if(n = read(serial_port, &p_type, sizeof(p_type)) > 0) {
                            if(p_type == PACKET_TYPE_RTK) {
                                if(n = read(serial_port, &length, sizeof(length)) > 0) {
                                    int16_t *buffer = (int16_t*)malloc((length/2) * sizeof(int16_t));
                                    if(n = read(serial_port, &(*buffer), (length/2) * sizeof(int16_t)) > 0) {
                                        parse_data_rtk(&(*buffer), &(*result));
                                        printf("\n");
                                    }
                                    memset(result, 0, sizeof(result));
                                }
                            } 
                        }          
                    }
                }
            }
        }
    }
    free(result);
    return 0;
}
