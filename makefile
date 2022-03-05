CC:=gcc
exe:=imu_driver
obj:=imu_driver.o

all:$(obj)
	$(CC) -o $(exe) $(obj)  

clean:
	rm -rf $(obj) $(exe)