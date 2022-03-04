#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>


# define TRUE 1
# define FALSE 0

void parse_data(char *data)
{
    int header = 0x55;
    int packet_type = 0x7A31;
    if(data[0] == header && data[1] == header) {
        if(data[2] == packet_type && data[3] == packet_type) {
            for(int i=4;i<40;i+=2) {
                printf("%c %c", data[i], data[i+1]);
            }
        }
    }
    return;
}

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
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    char *ptr;
    char read_buf[16];
    
    char n = 0;
    while(1) {
        while((n = read(serial_port, &read_buf, sizeof(read_buf))) > 0) {
            ptr = read_buf;
            parse_data(&(*ptr));
            printf("\n");
        }
    }
    return 0;
}
