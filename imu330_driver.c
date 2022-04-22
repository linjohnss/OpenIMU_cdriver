#include "driver.h"

int main(int argc, int **argv)
{
    int serial_port = serial_port_bringup(0, "/dev/ttyUSB0");
    imuDataPointer result;
    result = (imuDataPointer) malloc(sizeof(imuDataPointer *));
    memset(result, 0, sizeof(result));
    uint8_t *data = NULL;
    while (1) {
        data = launch_driver_8(serial_port, HEADER, PACKET_TYPE_330);
        if (data) {
            parse_data_330(&(data[3]), &(*result));
            memset(result, 0, sizeof(result));    
        }
        free(data);
    }
    free(result);
    return 0;
}
