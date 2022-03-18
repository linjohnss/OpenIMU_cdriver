# OpenIMU_cdriver
## Target
OpenIMU, OpenRTK driver by C.
## Devices
* [OpenIMU 383ZA](https://buildmedia.readthedocs.org/media/pdf/openimu/latest/openimu.pdf)
* [OpenIMU 330BI](https://buildmedia.readthedocs.org/media/pdf/openimu/latest/openimu.pdf)
* OpenRTK 330LI
## Get Start
### Get OpenIMU_cdriver
```shell
git clone https://github.com/linjohnss/OpenIMU_cdriver.git
```
### Level up permissions
```shell
sudo chmod 777 /dev/ttyUSB0
```

### Compile
```shell
make
```
Also can compile respectively
```shell
make imu //for imu 330 & 383
```
```shell
make rtk //for rtk
```
### Run
imu 330 & 383
```shell
./imu_driver
```
rtk 330
```shell
./rtk_driver
```
