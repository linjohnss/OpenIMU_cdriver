#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include "driver.h"

int main(int argc, int **argv) {
    serial_port_bringup(1);

    rtkDataPointer result;
    result = (rtkDataPointer)malloc(sizeof(rtkDataPointer*));

    while(1) {
        int16_t *data = launch_driver_16_type(HEADER, PACKET_TYPE_RTK);
        if(data)
            parse_data_rtk(&(*data), &(*result));
        memset(result, 0, sizeof(result));
    }
    free(result);
    return 0;
}
