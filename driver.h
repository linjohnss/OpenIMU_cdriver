# ifndef DRIVER_H
# define DRIVER_H

# define HEADER 0x55
# define PACKET_TYPE_RTK 0x3173
# define PACKET_TYPE_383 0x3153
# define PACKET_TYPE_330 0x317A

static u_int64_t time_count = 0;

typedef struct imuData *imuDataPointer;
struct imuData
{
    union time { float t_383; u_int32_t t_330; } time;
    u_int32_t count;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

typedef struct rtkData *rtkDataPointer;
struct rtkData
{
    u_int16_t GPS_Week;
    u_int32_t GPS_TimeOfWeek;
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
};

int16_t reverse(int16_t x)
{
    x = (((x & 0xff00) >> 8) | ((x & 0x00ff) << 8));
    return x;
}

int32_t concat(int16_t x, int16_t y)
{
    return x << 16 | (y & 0x0000ffff);
}

void parse_data_383(int16_t *data, imuDataPointer result)
{
    result->time.t_383 = (float)(u_int16_t)reverse(data[10])*15.259022;
    if(result->time.t_383 == 0 || result->time.t_383 == 1000000)
        time_count++;
    result->count = time_count;
    result->accx = (float)reverse(data[0])*20/(1<<16);
    result->accy = (float)reverse(data[1])*20/(1<<16);
    result->accz = (float)reverse(data[2])*20/(1<<16);
    result->gyrox = (float)reverse(data[3])*1260/(1<<16);
    result->gyroy = (float)reverse(data[4])*1260/(1<<16); 
    result->gyroz = (float)reverse(data[5])*1260/(1<<16);    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%u ", result->count);
    printf("%f ", result->time.t_383);
    return;
}

void parse_data_330(int32_t *data, imuDataPointer result)
{
    result->time.t_330 = data[0];
    result->accx = *((float*)((void*)(&data[1])));
    result->accy = *((float*)((void*)(&data[2])));
    result->accz = *((float*)((void*)(&data[3])));
    result->gyrox = *((float*)((void*)(&data[4])));
    result->gyroy = *((float*)((void*)(&data[5]))); 
    result->gyroz = *((float*)((void*)(&data[6]))); 
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);
    printf("%hu ", result->time.t_330);   
    return;
}

void parse_data_rtk(int16_t *data, rtkDataPointer result)
{
    int32_t temp;
    temp = concat(data[4], data[3]);
    result->accx = *((float*)((void*)(&temp)));
    temp = concat(data[6], data[5]);
    result->accy = *((float*)((void*)(&temp)));
    temp = concat(data[8], data[7]);
    result->accz = *((float*)((void*)(&temp)));
    temp = concat(data[10], data[9]);
    result->gyrox = *((float*)((void*)(&temp)));
    temp = concat(data[12], data[11]);
    result->gyroy = *((float*)((void*)(&temp))); 
    temp = concat(data[14], data[13]);
    result->gyroz = *((float*)((void*)(&temp))); 
    
    printf("%f ", result->accx);
    printf("%f ", result->accy);
    printf("%f ", result->accz);
    printf("%f ", result->gyrox);
    printf("%f ", result->gyroy);
    printf("%f ", result->gyroz);

    return;
}

#endif /* DRIVER_H */