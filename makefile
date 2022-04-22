CC:=gcc
imu330:=imu330_driver
imu383:=imu383_driver
rtk:=rtk330_driver
obj_imu330:=imu330_driver.o
obj_imu383:=imu383_driver.o
obj_rtk:=rtk330_driver.o

all:$(obj_imu330) $(obj_imu383) $(obj_rtk)
	$(CC) -o $(imu330) $(obj_imu330) 
	$(CC) -o $(imu383) $(obj_imu383) 
	$(CC) -o $(rtk) $(obj_rtk)

imu:$(obj_imu)
	$(CC) -o $(imu330) $(obj_imu330) 
	$(CC) -o $(imu383) $(obj_imu383) 

rtk:$(obj_rtk)
	$(CC) -o $(rtk) $(obj_rtk)

clean:
	rm -rf $(obj_imu330) $(imu330) $(obj_imu383) $(imu383) $(obj_rtk) $(rtk)