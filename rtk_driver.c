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
# define PACKET_TYPE_RTK 0x3173

static u_int64_t time_count = 0;

int32_t concat(int16_t x, int16_t y)
{
    return x << 16 | (y & 0x0000ffff);
}

typedef struct rtkData *rtkDataPointer;
struct rtkData
{
    u_int16_t GPS_Week;
    u_int32_t GPS_TimeOfWeek;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

void parse_data_rtk(int16_t *data, rtkDataPointer result)
{
    int32_t temp;
    temp = concat(data[4], data[3]);
    result->accx = *((float*)((void*)(&temp)));
    temp = concat(data[6], data[5]);
    result->accy = *((float*)((void*)(&temp)));
    temp = concat(data[8], data[7]);
    result->accz = *((float*)((void*)(&temp)));
    temp = concat(data[10], data[9]);
    result->gyrox = *((float*)((void*)(&temp)));
    temp = concat(data[12], data[11]);
    result->gyroy = *((float*)((void*)(&temp))); 
    temp = concat(data[14], data[13]);
    result->gyroz = *((float*)((void*)(&temp))); 
    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);

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
    cfsetispeed(&tty, B460800);
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

    int8_t test[37];

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
