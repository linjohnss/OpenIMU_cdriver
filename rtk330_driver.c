#include "driver.h"

int main(int argc, int **argv)
{
    serial_port_bringup(1);

    rtkDataPointer result;
    result = (rtkDataPointer) malloc(sizeof(rtkDataPointer *));
    memset(result, 0, sizeof(result));
    printf("x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro,gps_week,gps_timeofweek\n");
    uint8_t *data = NULL;
    while (1) {
        data = launch_driver_8(HEADER, PACKET_TYPE_RTK);
        if (data) {
            parse_data_rtk(&(data[3]), &(*result));
            memset(result, 0, sizeof(result));    
        }
        free(data);
    }
    free(result);
    return 0;
}