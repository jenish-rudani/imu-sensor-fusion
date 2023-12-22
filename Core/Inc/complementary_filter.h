/*
 * complementary_filter.h
 *
 *  Created on: Dec 21, 2023
 *      Author: atom
 */

#ifndef INC_COMPLEMENTARY_FILTER_H_
#define INC_COMPLEMENTARY_FILTER_H_

#define IMU_SAMPLE_TIME_MS  0.5
#define RAD2DEG     57.2957795131f
#define DEG2RAD     0.01745329200f
#define G_MPS       9.81000000000f
#define CF_ALPHA    0.09000000000f    //Complementary Filter Alpha


struct cf_data {
  float phi_hat_acc_rad;
  float theta_hat_acc_rad;

  float p_rps;
  float q_rps;
  float r_rps;

  float phi_dot;
  float theta_dot;

  float phiHat_deg;
  float theta_hat_rad;
};

typedef struct {

  float coeff[2];
  float out[2];

} rc_filter;

void rc_filter_init(rc_filter *filt, float cutoffFreqHz, float sampleTimeS);
float rc_filter_update(rc_filter *filt, float inp);


#endif /* INC_COMPLEMENTARY_FILTER_H_ */
