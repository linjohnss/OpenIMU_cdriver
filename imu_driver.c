#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include "driver.h"
# define HEADER 0x55
# define PACKET_TYPE_383 0x3153
# define PACKET_TYPE_330 0x317A

int main(int argc, int **argv) {
    serial_port_bringup(0);

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
                                if(n = read(serial_port, &length, sizeof(length)) > 0) {
                                    int16_t *buffer = (int16_t*)malloc((length/2) * sizeof(int16_t));
                                    if(n = read(serial_port, &(*buffer), (length/2) * sizeof(int16_t)) > 0) {
                                        parse_data_383(&(*buffer), &(*result));
                                        printf("\n");
                                    }
                                    memset(result, 0, sizeof(result));
                                }
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
