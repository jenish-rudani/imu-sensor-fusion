/*
 * lsm6dso_imu.h
 *
 *  Created on: Dec 21, 2023
 *      Author: atom
 */

#ifndef INC_LSM6DSO_IMU_H_
#define INC_LSM6DSO_IMU_H_

struct imu_data{
    float acc_x;            //Acc X axis
    float acc_y;            //Acc Y axis
    float acc_z;            //Acc Z axis
    float acc_rms;          //
    float gyr_x;            //Gyr X axis
    float gyr_y;            //Gyr Y axis
    float gyr_z;            //Gyr Z axis
    float temp;             //Temp in chip
};

#endif /* INC_LSM6DSO_IMU_H_ */
