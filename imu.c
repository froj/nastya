#include <string.h>
#include "imu.h"

const serialization_fn_table_t imu_fn_table = {
    .hash = {80, 179, 221, 132, 64, 71, 11, 94},
    .serialize = imu_serialize,
    .serialized_size = imu_serialized_size,
    .deserialize = imu_deserialize,
    .alloc_size = imu_alloc_size};



static void *alloc(int size, void **alloc_mem)
{
    void *mem = *alloc_mem;
    alloc_mem += (size-1 | (8-1)) + 1;
    return mem;
}

static uint8_t *string_serialize(char *str, uint8_t *buffer)
{
    int len = strlen(str);
    *buffer++ = (int8_t)(((int16_t)len)>>8);
    *buffer++ = (int8_t)(((int16_t)len)>>0);
    int i; // todo: use strncpy()
    for (i = 0; i < len; i++) {
        *buffer++ = *str++;
    }
    return buffer;
}

static uint8_t *string_deserialize(char **str, uint8_t *buffer, void **alloc_mem)
{
    int16_t len = (((int16_t)*buffer++)<<8);
    len += ((int16_t)*buffer++);
    *str = alloc((len+1)*sizeof(char), alloc_mem);
    strncpy(*str, (char*)buffer, len);
    (*str)[len] = '\0';
    buffer += len*sizeof(char);
    return buffer;
}


// IMU
uint8_t *imu_serialize(imu_t  *data, uint8_t *buffer)
{
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_x)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_x)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_x)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_x)>>0);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_y)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_y)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_y)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_y)>>0);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_z)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_z)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_z)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->acc_z)>>0);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_x)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_x)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_x)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_x)>>0);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_y)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_y)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_y)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_y)>>0);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_z)>>24);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_z)>>16);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_z)>>8);
    *buffer++ = (int8_t)((*(int32_t*)&data->gyro_z)>>0);
    return buffer;
}

size_t imu_serialized_size(imu_t  *data)
{
    return 24;
}

imu_t *imu_deserialize(uint8_t *buffer, void *alloc_mem)
{
    imu_t  *data = alloc(sizeof(imu_t ), &alloc_mem);
    _imu_deserialize_into(data, buffer, &alloc_mem);
    return data;
}

uint8_t *_imu_deserialize_into(imu_t  *data, uint8_t *buffer, void **alloc_mem)
{
    *(int32_t*)&data->acc_x = 0;
    *(int32_t*)&data->acc_x += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->acc_x += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->acc_x += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->acc_x += (((int32_t)*buffer++)<<0);
    *(int32_t*)&data->acc_y = 0;
    *(int32_t*)&data->acc_y += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->acc_y += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->acc_y += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->acc_y += (((int32_t)*buffer++)<<0);
    *(int32_t*)&data->acc_z = 0;
    *(int32_t*)&data->acc_z += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->acc_z += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->acc_z += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->acc_z += (((int32_t)*buffer++)<<0);
    *(int32_t*)&data->gyro_x = 0;
    *(int32_t*)&data->gyro_x += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->gyro_x += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->gyro_x += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->gyro_x += (((int32_t)*buffer++)<<0);
    *(int32_t*)&data->gyro_y = 0;
    *(int32_t*)&data->gyro_y += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->gyro_y += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->gyro_y += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->gyro_y += (((int32_t)*buffer++)<<0);
    *(int32_t*)&data->gyro_z = 0;
    *(int32_t*)&data->gyro_z += (((int32_t)*buffer++)<<24);
    *(int32_t*)&data->gyro_z += (((int32_t)*buffer++)<<16);
    *(int32_t*)&data->gyro_z += (((int32_t)*buffer++)<<8);
    *(int32_t*)&data->gyro_z += (((int32_t)*buffer++)<<0);
    return buffer;
}

size_t _imu_dynamic_alloc_size(uint8_t **buffer, uint8_t *buffend)
{
    return 0;
}

size_t imu_alloc_size(uint8_t *buffer, size_t len)
{
    return sizeof(imu_t);
}

