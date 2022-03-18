# ifndef DRIVER_H
# define DRIVER_H
#include <stdbool.h>

# define HEADER 0x55
# define PACKET_TYPE_383 0x3153
# define PACKET_TYPE_330 0x317A
# define PACKET_TYPE_RTK 0x3173

static u_int64_t time_count = 0;
static int serial_port;
static int8_t head;
static int16_t p_type;
static int8_t length = 0;

typedef struct imuData *imuDataPointer;
struct imuData
{
    union time { float t_383; u_int32_t t_330; } time;
    u_int32_t count;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

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

int16_t reverse(int16_t x)
{
    x = (((x & 0xff00) >> 8) | ((x & 0x00ff) << 8));
    return x;
}

int32_t concat(int16_t x, int16_t y)
{
    return x << 16 | (y & 0x0000ffff);
}

void parse_data_383(int16_t *data, imuDataPointer result)
{
    result->time.t_383 = (float)(u_int16_t)reverse(data[10])*15.259022;
    if(result->time.t_383 == 0 || result->time.t_383 == 1000000)
        time_count++;
    result->count = time_count;
    result->accx = (float)reverse(data[0])*20/(1<<16);
    result->accy = (float)reverse(data[1])*20/(1<<16);
    result->accz = (float)reverse(data[2])*20/(1<<16);
    result->gyrox = (float)reverse(data[3])*1260/(1<<16);
    result->gyroy = (float)reverse(data[4])*1260/(1<<16); 
    result->gyroz = (float)reverse(data[5])*1260/(1<<16);    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%u ", result->count);
    printf("%f ", result->time.t_383);
    printf("\n");
    return;
}

void parse_data_330(int16_t *data, imuDataPointer result)
{
    int32_t temp;
    temp = concat(data[1], data[0]);
    result->time.t_330 = temp;
    temp = concat(data[3], data[2]);
    result->accx = *((float*)((void*)(&temp)));
    temp = concat(data[5], data[4]);
    result->accy = *((float*)((void*)(&temp)));
    temp = concat(data[7], data[6]);
    result->accz = *((float*)((void*)(&temp)));
    temp = concat(data[9], data[8]);
    result->gyrox = *((float*)((void*)(&temp)));
    temp = concat(data[11], data[10]);
    result->gyroy = *((float*)((void*)(&temp))); 
    temp = concat(data[13], data[12]);
    result->gyroz = *((float*)((void*)(&temp))); 
    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%hu ", result->time.t_330);
    printf("\n");

    return;
}

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
    printf("\n");

    return;
}

int16_t* launch_driver_16(int8_t header)
{
    if((read(serial_port, &head, sizeof(head))) > 0) {
        if(head == header) {
            if(read(serial_port, &head, sizeof(head)) > 0) {
                if(head == header) {
                    if(read(serial_port, &p_type, sizeof(p_type)) > 0) {
                        if(read(serial_port, &length, sizeof(length)) > 0) {
                            int16_t *buffer= (int16_t*)malloc((length/2) * sizeof(int16_t));
                            if(read(serial_port, &(*buffer), (length/2) * sizeof(int16_t)) > 0) {
                                return buffer; 
                            }
                        }
                    }          
                }
            }
        }
    }
    return NULL;
}

int16_t* launch_driver_16_type(int8_t header, int16_t packet_type)
{
    if((read(serial_port, &head, sizeof(head))) > 0) {
        if(head == header) {
            if(read(serial_port, &head, sizeof(head)) > 0) {
                if(head == header) {
                    if(read(serial_port, &p_type, sizeof(p_type)) > 0) {
                        if(p_type == packet_type) {
                            if(read(serial_port, &length, sizeof(length)) > 0) {
                                int16_t *buffer= (int16_t*)malloc((length/2) * sizeof(int16_t));
                                if(read(serial_port, &(*buffer), (length/2) * sizeof(int16_t)) > 0) {
                                   return buffer; 
                                }
                            }
                        } 
                    }          
                }
            }
        }
    }
    return NULL;
}

int32_t* launch_driver_32(int8_t header, int16_t packet_type)
{
    if((read(serial_port, &head, sizeof(head))) > 0) {
        if(head == header) {
            if(read(serial_port, &head, sizeof(head)) > 0) {
                if(head == header) {
                    if(read(serial_port, &p_type, sizeof(p_type)) > 0) {
                        if(p_type == packet_type) {
                            if(read(serial_port, &length, sizeof(length)) > 0) {
                                int32_t *buffer= (int32_t*)malloc((length/4) * sizeof(int32_t));
                                if(read(serial_port, &(*buffer), (length/4) * sizeof(int32_t)) > 0) {
                                   return buffer; 
                                }
                            }
                     } 
                    }          
                }
            }
        }
    }
    return NULL;
}

void serial_port_bringup(int device_type)
{
    serial_port = open("/dev/ttyUSB0", O_RDWR);

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
    speed_t sp = B115200;
    if(device_type == 0)    // IMU
        sp = B115200;
    else if (device_type == 1)  //RTK
        sp = B460800;
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

#endif /* DRIVER_H */