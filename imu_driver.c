#include "driver.h"

int main(int argc, int **argv) {
  serial_port_bringup(0);

  imuDataPointer result;
  result = (imuDataPointer)malloc(sizeof(imuDataPointer *));
  while (1) {
    int8_t *data = launch_driver_8(HEADER, PACKET_TYPE_330);
    if (data && p_type == PACKET_TYPE_330)
      parse_data_330(&(*data), &(*result));
    // else if(data && p_type == PACKET_TYPE_383)
    //     parse_data_383(&(*data), &(*result));
    memset(result, 0, sizeof(result));
  }
  free(result);
  return 0;
}
