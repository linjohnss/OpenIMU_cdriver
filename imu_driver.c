#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>


# define HEADER 0x55
# define PACKET_TYPE_383 0x3153
# define PACKET_TYPE_330 0x317A

void parse_data_383(int16_t *data)
{
    printf("%f ", (float)data[0]*20/(2<<15));
    printf("%f ", (float)data[1]*20/(2<<15));
    printf("%f ", (float)data[2]*20/(2<<15));
    printf("%f ", (float)data[3]*1260/(2<<15));
    printf("%f ", (float)data[4]*1260/(2<<15));
    printf("%f ", (float)data[5]*1260/(2<<15));
    printf("%f ", (float)data[6]*200/(2<<15));
    printf("%f ", (float)data[7]*200/(2<<15));
    printf("%f ", (float)data[8]*200/(2<<15));
    printf("%f ", (float)data[9]*200/(2<<15));
    printf("%hu ", (data[10]));
    return;
}

void parse_data_330(int32_t *data)
{
    printf("%f ", (float)data[1]*20/(2<<15));
    printf("%f ", (float)data[2]*20/(2<<15));
    printf("%f ", (float)data[3]*20/(2<<15));
    printf("%f ", (float)data[4]*1260/(2<<15));
    printf("%f ", (float)data[5]*1260/(2<<15));
    printf("%f ", (float)data[6]*1260/(2<<15));
    printf("%u ", (data[0]));
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

    int8_t head;
    int16_t p_type;

    int n = 0;
    while(1) {
        while((n = read(serial_port, &head, sizeof(head))) > 0) {
            if(head == HEADER) {
                if(n = read(serial_port, &head, sizeof(head)) > 0) {
                    if(head == HEADER) {
                        if(n = read(serial_port, &p_type, sizeof(p_type)) > 0) {
                            if(p_type == PACKET_TYPE_383) {
                                int16_t *ptr;
                                int16_t buffer[12];
                                if(n = read(serial_port, &buffer, sizeof(buffer)) > 0) {
                                    ptr = buffer;
                                    parse_data_383(&(*ptr));
                                    printf("\n");
                                }
                                memset(buffer, 0, sizeof(buffer));
                            } else if(p_type == PACKET_TYPE_330) {
                                int32_t *ptr;
                                int32_t buffer[10];
                                if(n = read(serial_port, &buffer, sizeof(buffer)) > 0) {
                                    ptr = buffer;
                                    parse_data_330(&(*ptr));
                                    printf("\n");
                                }
                                memset(buffer, 0, sizeof(buffer));
                            }
                        } 
                        
                    }
                }
            }
        }
    }
    return 0;
}