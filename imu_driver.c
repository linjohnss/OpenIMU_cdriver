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

u_int32_t reverse(u_int32_t x)
{
    u_int32_t n = x;
    n = ((n & 0xffff0000) >> 16) | ((n & 0x0000ffff) << 16);
    n = ((n & 0xff00ff00) >>  8) | ((n & 0x00ff00ff) <<  8);
    return n;
} 

typedef struct imuData *imuDataPointer;
struct imuData
{
    union time { u_int16_t t_383; u_int32_t t_330; } time;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

void parse_data_383(int16_t *data, imuDataPointer result)
{
    result->time.t_383 = data[10];
    result->accx = (float)data[0]*20/(1<<16);
    result->accy = (float)data[1]*20/(1<<16);
    result->accz = (float)data[2]*20/(1<<16);
    result->gyrox = (float)data[3]*1260/(1<<16);
    result->gyroy = (float)data[4]*1260/(1<<16); 
    result->gyroz = (float)data[5]*1260/(1<<16);    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%hu ", result->time.t_383);
    return;
}

void parse_data_330(int32_t *data, imuDataPointer result)
{
    result->time.t_330 = data[0];
    result->accx = *((float*)((void*)(&data[1])));
    result->accy = *((float*)((void*)(&data[2])));
    result->accz = *((float*)((void*)(&data[3])));
    result->gyrox = *((float*)((void*)(&data[4])));
    result->gyroy = *((float*)((void*)(&data[5]))); 
    result->gyroz = *((float*)((void*)(&data[6])));    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%u ", result->time.t_330);
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
    int8_t length = 0;
    imuDataPointer result;
    result = (imuDataPointer)malloc(sizeof(imuDataPointer*));
    
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
                                    parse_data_383(&(*ptr), &(*result));
                                    printf("\n");
                                }
                                memset(result, 0, sizeof(result));
                            } else if(p_type == PACKET_TYPE_330) {
                                if(n = read(serial_port, &length, sizeof(length)) > 0) {
                                    int32_t *buffer = (int32_t*)malloc((length/4) * sizeof(int32_t));
                                    if(n = read(serial_port, &(*buffer), (length/4) * sizeof(int32_t)) > 0) {
                                        parse_data_330(&(*buffer), &(*result));
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
