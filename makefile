CC:=gcc
imu:=imu_driver
rtk:=rtk_driver
obj_imu:=imu_driver.o
obj_rtk:=rtk_driver.o

all:$(obj_imu) $(obj_rtk)
	$(CC) -o $(imu) $(obj_imu) 
	$(CC) -o $(rtk) $(obj_rtk)

clean:
	rm -rf $(obj_imu) $(imu) $(obj_rtk) $(rtk)