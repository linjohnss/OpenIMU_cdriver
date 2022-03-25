#include "driver.h"

int main(int argc, int **argv) {
  serial_port_bringup(1);

  rtkDataPointer result;
  result = (rtkDataPointer)malloc(sizeof(rtkDataPointer *));

  while (1) {
    int8_t *data = launch_driver_8(HEADER, PACKET_TYPE_RTK);
    if (data)
      parse_data_rtk(&(*data), &(*result));
    memset(result, 0, sizeof(result));
  }
  free(result);
  return 0;
}
