#include "driver.h"

int main(int argc, int **argv)
{
    serial_port_bringup(0);

    imuDataPointer result;
    result = (imuDataPointer) malloc(sizeof(imuDataPointer *));
    memset(result, 0, sizeof(result));
    uint8_t *data = NULL;
    while (1) {
        data = launch_driver_8(HEADER, PACKET_TYPE_383);
        if (data) {
            parse_data_383(&(data[3]), &(*result));
            memset(result, 0, sizeof(result));
        }
        free(data);
    }
    free(result);
    return 0;
}
