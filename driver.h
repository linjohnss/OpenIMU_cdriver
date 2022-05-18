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
#define PACKET_TYPE_383 0x53    // s1:5331
#define PACKET_TYPE_330 0x7A    // z1:7A31
#define PACKET_TYPE_RTK 0x73    // s1:7331
#define PACKET_TYPE_RTK_INS 0x69
#define PACKET_TYPE_RTK_GNSS 0x67

#define PI 3.1415926

static uint64_t time_count = 0;
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
    uint32_t GPS_TimeOfWeek;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

typedef struct rtkGnssData *rtkGnssDataPointer;
struct rtkGnssData {
    uint16_t    gps_week;
    uint32_t    gps_millisecs;
    uint8_t     position_type;
    double      latitude;
    double      longitude;
    double      height;
    float       latitude_std_deviation;
    float       longitude_std_deviation;
    float       height_std_deviation;
    uint8_t     num_of_satellites;
    uint8_t     num_satellite_in_solution;
    float       hdop;
    float       diffage;
    float       north_vel;
    float       east_vel;
    float       up_vel;
    float       north_vel_std_deviation;    
    float       east_vel_std_deviation;   
    float       up_vel_std_deviation;
};
typedef struct rtkInsData *rtkInsDataPointer;
struct rtkInsData {
    uint16_t    gps_week;
    uint32_t    gps_millisecs;
    uint8_t     ins_status;
    uint8_t     ins_position_type;
    double      latitude;
    double      longitude;
    double      height;
    double      north_vel;
    double      east_vel;
    double      up_vel;
    double      roll;
    double      pitch;
    double      heading;
    float       latitude_std_deviation;
    float       longitude_std_deviation;
    float       height_std_deviation;
    float       north_vel_std_deviation;    
    float       east_vel_std_deviation;  
    float       up_vel_std_deviation;     
    float       roll_std_deviation; 
    float       pitch_std_deviation;   
    float       heading_std_deviation;
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

int64_t concat_64(int8_t a1, int8_t b1, int8_t c1, int8_t d1, int8_t a2, int8_t b2, int8_t c2, int8_t d2)
{
    return (int64_t)concat_32(a1, b1, c1, d1) << 32 | ((int64_t)concat_32(a2, b2, c2, d2) & 0x00000000ffffffff);
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

void parse_data_383(int8_t *data, imuDataPointer result)
{
    result->time.t_383 =
        (float) (uint16_t) (concat_16(data[20], data[21])) * 15.259022;
    if (result->time.t_383 == 0 || result->time.t_383 == 1000000)
        time_count++;
    result->count = time_count;
    result->accx =
        (float) (concat_16(data[0], data[1])) * 20 / (1 << 16);
    result->accy =
        (float) (concat_16(data[2], data[3])) * 20 / (1 << 16);
    result->accz =
        (float) (concat_16(data[4], data[5])) * 20 / (1 << 16);
    result->gyrox =
        (float) (concat_16(data[6], data[7])) * 7 * PI / (1 << 16);
    result->gyroy =
        (float) (concat_16(data[8], data[9])) * 7 * PI / (1 << 16);
    result->gyroz =
        (float) (concat_16(data[10], data[11])) * 7 * PI / (1 << 16);
    printf("%f ", result->accx);    // unit (g)
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);   // unit (rad/s)
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%u ", result->count);
    printf("%f ", result->time.t_383);  // unit (uS)
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
    result->GPS_TimeOfWeek = temp2;
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
    printf("%.3f", result->GPS_TimeOfWeek/1000.0);
    printf("\n");
    return;
}

void parse_data_rtk_gnss(int8_t *data, rtkGnssDataPointer result)
{
    void *temp;
    *((int16_t *) (&temp)) = concat_16(data[1], data[0]);
    result->gps_week = *((uint16_t *) (&temp));
    *((int32_t *) (&temp)) = concat_32(data[5], data[4], data[3], data[2]);
    result->gps_millisecs = *((uint32_t *) (&temp));
    result->position_type = data[6];
    *((int64_t *) (&temp)) = concat_64(data[14], data[13], data[12], data[11], data[10], data[9], data[8], data[7]);
    result->latitude = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[22], data[21], data[20], data[19], data[18], data[17], data[16], data[15]);
    result->longitude = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[30], data[29], data[28], data[27], data[26], data[25], data[24], data[23]);
    result->height = (*((double *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[34], data[33], data[32], data[31]);
    result->latitude_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[38], data[37], data[36], data[35]);
    result->longitude_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[42], data[41], data[40], data[39]);
    result->height_std_deviation = (*((float *) (&temp)));
    result->num_of_satellites = data[43];
    result->num_satellite_in_solution = data[44];
    *((int32_t *) (&temp)) = concat_32(data[48], data[47], data[46], data[45]);
    result->hdop = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[52], data[51], data[50], data[49]);
    result->diffage = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[56], data[55], data[54], data[53]);
    result->north_vel = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[60], data[59], data[58], data[57]);
    result->east_vel = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[64], data[63], data[62], data[61]);
    result->up_vel = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[68], data[67], data[66], data[65]);
    result->north_vel_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[72], data[71], data[70], data[69]);
    result->east_vel_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[76], data[75], data[74], data[73]);
    result->up_vel_std_deviation = (*((float *) (&temp)));

    printf("%.9lf,", result->latitude);
    printf("%.9lf,", result->longitude);
    printf("%.4lf,", result->height);
    printf("%.4lf,", result->latitude_std_deviation);
    printf("%.4lf,", result->longitude_std_deviation);
    printf("%.4lf,", result->height_std_deviation);
    printf("%d,", result->gps_week);
    printf("%.3f", result->gps_millisecs/1000.0);
    printf("\n");
    return;
}

void parse_data_rtk_ins(int8_t *data, rtkInsDataPointer result)
{
    void *temp;
    *((int16_t *) (&temp)) = concat_16(data[1], data[0]);
    result->gps_week = *((uint16_t *) (&temp));
    *((int32_t *) (&temp)) = concat_32(data[5], data[4], data[3], data[2]);
    result->gps_millisecs = *((uint32_t *) (&temp));
    result->ins_status = data[6];
    result->ins_position_type = data[7];
    *((int64_t *) (&temp)) = concat_64(data[15], data[14], data[13], data[12], data[11], data[10], data[9], data[8]);
    result->latitude = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[23], data[22], data[21], data[20], data[19], data[18], data[17], data[16]);
    result->longitude = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[31], data[30], data[29], data[28], data[27], data[26], data[25], data[24]);
    result->height = (*((double *) (&temp)));
     *((int64_t *) (&temp)) = concat_64(data[39], data[38], data[37], data[36], data[35], data[34], data[33], data[32]);
    result->north_vel = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[47], data[46], data[45], data[44], data[43], data[42], data[41], data[40]);
    result->east_vel = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[55], data[54], data[53], data[52], data[51], data[50], data[49], data[48]);
    result->roll = (*((double *) (&temp)));
      *((int64_t *) (&temp)) = concat_64(data[63], data[62], data[61], data[60], data[59], data[58], data[57], data[56]);
    result->pitch = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[71], data[70], data[69], data[68], data[67], data[66], data[65], data[64]);
    result->heading = (*((double *) (&temp)));
    *((int64_t *) (&temp)) = concat_64(data[79], data[78], data[77], data[76], data[75], data[74], data[73], data[72]);
    result->up_vel = (*((double *) (&temp)));

    *((int32_t *) (&temp)) = concat_32(data[83], data[82], data[81], data[80]);
    result->latitude_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[87], data[86], data[85], data[84]);
    result->longitude_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[91], data[90], data[89], data[88]);
    result->height_std_deviation = (*((float *) (&temp)));
     *((int32_t *) (&temp)) = concat_32(data[95], data[94], data[93], data[92]);
    result->north_vel_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[99], data[98], data[97], data[96]);
    result->east_vel_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[103], data[102], data[101], data[100]);
    result->roll_std_deviation = (*((float *) (&temp)));
      *((int32_t *) (&temp)) = concat_32(data[107], data[106], data[105], data[104]);
    result->pitch_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[111], data[110], data[109], data[108]);
    result->heading_std_deviation = (*((float *) (&temp)));
    *((int32_t *) (&temp)) = concat_32(data[115], data[114], data[113], data[112]);
    result->up_vel_std_deviation = (*((float *) (&temp)));

    printf("%14.9lf,", result->latitude);
    printf("%14.9lf,", result->longitude);
    printf("%10.4lf,", result->height);
    printf("%d,", result->gps_week);
    printf("%.3f", result->gps_millisecs/1000.0);
    printf("\n");
    return;
}

int8_t *launch_driver_8(int serial_port, int8_t header, int8_t packet_type)
{
    int8_t *buffer = (int8_t *) malloc((255) * sizeof(int8_t));
    if ((read(serial_port, &buffer[0], sizeof(int8_t))) > 0) {
        if (buffer[0] == header) {
            if (read(serial_port, &buffer[0], sizeof(int8_t)) > 0) {
                if (buffer[0] == header) {
                    if (read(serial_port, &buffer[0], sizeof(int8_t)) > 0) {
                        if (buffer[0] == packet_type) {
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
                                            return buffer;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        } 
    }
    free(buffer);
    return NULL;
}

int serial_port_bringup(int device_type, char *portname)
{
    // char *portname[4] = {"/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
    //                      "/dev/ttyUSB3"};
    // for (int i = 0; (serial_port = open(portname[i], O_RDONLY)) < 0; i++)
    //     ;
    
    int serial_port = open(portname, O_RDONLY);
    if (serial_port < 0) {
        printf("Error %i from open: %s\n", errno, strerror(errno));
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

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

    return serial_port;
}

#endif /* DRIVER_H */
