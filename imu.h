#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifndef __SERIALIZATION_FN_TABLE
#define __SERIALIZATION_FN_TABLE
typedef struct {
    uint8_t hash[8];
    uint8_t *(*serialize)(void  *data, uint8_t *buffer);
    size_t (*serialized_size)(void  *data);
    void *(*deserialize)(uint8_t *buffer, void *alloc_mem);
    size_t (*alloc_size)(uint8_t *buffer, size_t len);
} serialization_fn_table_t;
#endif

// type definitions

// IMU:
//
// cvra imu data
typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_t ;


// function tables

extern const serialization_fn_table_t imu_fn_table;

// prototypes

#ifdef __cplusplus
extern "C" {
#endif

// IMU
uint8_t *imu_serialize(imu_t  *data, uint8_t *buffer);
size_t imu_serialized_size(imu_t  *data);
imu_t *imu_deserialize(uint8_t *buffer, void *alloc_mem);
uint8_t *_imu_deserialize_into(imu_t  *data, uint8_t *buffer, void **alloc_mem);
size_t _imu_dynamic_alloc_size(uint8_t **buffer, uint8_t *buffend);
size_t imu_alloc_size(uint8_t *buffer, size_t len);

#ifdef __cplusplus
}
#endif

#endif
