#include "driver.h"

int main(int argc, int **argv)
{
    int serial_port = serial_port_bringup(1, "/dev/ttyUSB0");
    rtkGnssDataPointer result;
    result = (rtkGnssDataPointer) malloc(255 * 8);
    memset(result, 0, sizeof(result));
    // printf("x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro,gps_week,gps_timeofweek\n");
    uint8_t *data = NULL;
    while (1) {
        data = launch_driver_8(serial_port, HEADER, PACKET_TYPE_RTK_GNSS);
        if (data) {
            parse_data_rtk_gnss(&(data[3]), &(*result));
            memset(result, 0, sizeof(result));
            free(data);
        }      
    }
    free(result);
    return 0;
}
