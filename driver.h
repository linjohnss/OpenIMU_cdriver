#ifndef DRIVER_H
#define DRIVER_H
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define HEADER 0x55
#define PACKET_TYPE_383 0x3153
#define PACKET_TYPE_330 0x317A
#define PACKET_TYPE_RTK 0x3173

static uint64_t time_count = 0;
static int serial_port;
static int8_t head;
static int16_t p_type;
static int8_t length = 0;

typedef struct imuData *imuDataPointer;
struct imuData {
    union time {
        float t_383;
        uint32_t t_330;
    } time;
    uint32_t count;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

typedef struct rtkData *rtkDataPointer;
struct rtkData {
    uint16_t GPS_Week;
    float GPS_TimeOfWeek;
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

int16_t concat_16(int8_t a, int8_t b)
{
    return a << 8 | (b & 0x00ff);
}

int32_t concat_32(int8_t a, int8_t b, int8_t c, int8_t d)
{
    return concat_16(a, b) << 16 | (concat_16(c, d) & 0x0000ffff);
}

uint16_t CalculateCRC(uint8_t *buf, uint16_t length)
{
    uint16_t crc = 0x1D0F;

    for (int i = 0; i < length; i++) {
        crc ^= buf[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }

    return ((crc << 8) & 0xFF00) | ((crc >> 8) & 0xFF);
}

void parse_data_383(int16_t *data, imuDataPointer result)
{
    result->time.t_383 =
        (float) (uint16_t) reverse(concat_16(data[20], data[21])) * 15.259022;
    if (result->time.t_383 == 0 || result->time.t_383 == 1000000)
        time_count++;
    result->count = time_count;
    result->accx =
        (float) reverse(concat_16(data[1], data[0])) * 20 / (1 << 16);
    result->accy =
        (float) reverse(concat_16(data[3], data[2])) * 20 / (1 << 16);
    result->accz =
        (float) reverse(concat_16(data[5], data[4])) * 20 / (1 << 16);
    result->gyrox =
        (float) reverse(concat_16(data[7], data[6])) * 1260 / (1 << 16);
    result->gyroy =
        (float) reverse(concat_16(data[9], data[8])) * 1260 / (1 << 16);
    result->gyroz =
        (float) reverse(concat_16(data[11], data[10])) * 1260 / (1 << 16);
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

void parse_data_330(int8_t *data, imuDataPointer result)
{
    int32_t temp;
    temp = concat_32(data[3], data[2], data[1], data[0]);
    result->time.t_330 = temp;
    temp = concat_32(data[7], data[6], data[5], data[4]);
    result->accx = *((float *) ((void *) (&temp)));
    temp = concat_32(data[11], data[10], data[9], data[8]);
    result->accy = *((float *) ((void *) (&temp)));
    temp = concat_32(data[15], data[14], data[13], data[12]);
    result->accz = *((float *) ((void *) (&temp)));
    temp = concat_32(data[19], data[18], data[17], data[16]);
    result->gyrox = *((float *) ((void *) (&temp)));
    temp = concat_32(data[23], data[22], data[21], data[20]);
    result->gyroy = *((float *) ((void *) (&temp)));
    temp = concat_32(data[27], data[26], data[25], data[24]);
    result->gyroz = *((float *) ((void *) (&temp)));

    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%u ", result->time.t_330);
    printf("\n");

    return;
}

void parse_data_rtk(int8_t *data, rtkDataPointer result)
{
    int16_t temp1;
    int32_t temp2;
    temp1 = concat_16(data[1], data[0]);
    result->GPS_Week = temp1;
    temp2 = concat_32(data[5], data[4], data[3], data[2]);
    result->GPS_TimeOfWeek = (float) temp2 / 1000;
    temp2 = concat_32(data[9], data[8], data[7], data[6]);
    result->accx = *((float *) ((void *) (&temp2)));
    temp2 = concat_32(data[13], data[12], data[11], data[10]);
    result->accy = *((float *) ((void *) (&temp2)));
    temp2 = concat_32(data[17], data[16], data[15], data[14]);
    result->accz = *((float *) ((void *) (&temp2)));
    temp2 = concat_32(data[21], data[20], data[19], data[18]);
    result->gyrox = *((float *) ((void *) (&temp2)));
    temp2 = concat_32(data[25], data[24], data[23], data[22]);
    result->gyroy = *((float *) ((void *) (&temp2)));
    temp2 = concat_32(data[29], data[28], data[27], data[26]);
    result->gyroz = *((float *) ((void *) (&temp2)));

    printf("%f,", result->accx);
    printf("%f,", result->accy);
    printf("%f,", result->accz);
    printf("%f,", result->gyrox);
    printf("%f,", result->gyroy);
    printf("%f,", result->gyroz);
    printf("%d,", result->GPS_Week);
    printf("%f", result->GPS_TimeOfWeek);
    printf("\n");
    return;
}

int8_t *launch_driver_8(int8_t header, int16_t packet_type)
{
    int8_t *buffer = (int8_t *) malloc((50) * sizeof(int8_t));
    if ((read(serial_port, &buffer[0], sizeof(int8_t))) > 0) {
        if (buffer[0] == header) {
            if (read(serial_port, &buffer[0], sizeof(int8_t)) > 0) {
                if (buffer[0] == header) {
                    if (read(serial_port, &buffer[0], sizeof(int8_t)) > 0) {
                        if (buffer[0] == 0x73) {
                            if (read(serial_port, &buffer[1], sizeof(int8_t)) >
                                0) {
                                if (buffer[1] == 0x31) {
                                    if (read(serial_port, &buffer[2],
                                             sizeof(int8_t)) > 0) {
                                        int32_t count = 0;
                                        while (count < buffer[2] + 2) {
                                            if (read(serial_port,
                                                     &buffer[count + 3],
                                                     sizeof(int8_t)) > 0) {
                                                count++;
                                            }
                                        }
                                        if (CalculateCRC(buffer,
                                                         buffer[2] + 3) ==
                                            (uint16_t) concat_16(
                                                buffer[buffer[2] + 4],
                                                buffer[buffer[2] + 3]))
                                            return &buffer[3];
                                    }
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
    char *portname[4] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
                         "/dev/ttyUSB3"};
    for (int i = 0; (serial_port = open(portname[i], O_RDONLY)) < 0; i++)
        ;

    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }
    // tty.c_cflag &= ~PARENB;
    // tty.c_cflag &= ~CSTOPB;
    // tty.c_cflag &= ~CSIZE;
    // tty.c_cflag |= CS8;
    // tty.c_cflag &= ~CRTSCTS;
    // tty.c_lflag &= ICANON;
    // tty.c_lflag &= ~ISIG;
    // tty.c_oflag &= ~ONLCR;
    // tty.c_cc[VTIME] = 0;
    // tty.c_cc[VMIN] = 0;

    tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                             // communication (most common)
    tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size
    tty.c_cflag |= CS8;      // 8 bits per byte (most common)
    tty.c_cflag &=
        ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |=
        CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;    // Disable echo
    tty.c_lflag &= ~ECHOE;   // Disable erasure
    tty.c_lflag &= ~ECHONL;  // Disable new-line echo
    tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL);  // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                            // (e.g. newline chars)
    tty.c_oflag &=
        ~ONLCR;  // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT
    // PRESENT ON LINUX) tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars
    // (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds), returning as
                           // soon as any data is received.
    tty.c_cc[VMIN] = 0;

    speed_t sp = B115200;
    if (device_type == 0)  // IMU
        sp = B115200;
    else if (device_type == 1)  // RTK
        sp = B460800;
    cfsetispeed(&tty, sp);
    cfsetospeed(&tty, sp);
    tcflush(serial_port, TCIFLUSH);
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
}

#endif /* DRIVER_H */