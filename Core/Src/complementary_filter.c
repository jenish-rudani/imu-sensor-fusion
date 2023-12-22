/*
 * complementary_filter.c
 *
 *  Created on: Dec 21, 2023
 *      Author: atom
 */
#include "complementary_filter.h"
#include "lsm6dso_imu.h"
#include "math.h"


void rc_filter_init(rc_filter *filt, float cutoff_freq, float sample_time) {

  /* Compute equivalent 'RC' constant from cut-off frequency */
  float RC = 1.0f / (6.28318530718f * cutoff_freq);

  /* Pre-compute filter coefficients for first-order low-pass filter */
  filt->coeff[0] = sample_time / (sample_time + RC);
  filt->coeff[1] = RC / (sample_time + RC);

  /* Clear output buffer */
  filt->out[0] = 0.0f;
  filt->out[1] = 0.0f;

}

float rc_filter_update(rc_filter *filt, float inp) {

  /* Shift output samples */
  filt->out[1] = filt->out[0];

  /* Compute new output sample */
  filt->out[0] = filt->coeff[0] * inp + filt->coeff[1] * filt->out[1];

  /* Return filtered sample */
  return (filt->out[0]);

}

void cf_filter_apply (struct imu_data *imu_df, struct cf_data *results) {
  static float phi_hat_rad = 0.0f;
  static float theta_hat_rad = 0.0f;
  // Accelerometer
  float phi_hat_acc_rad = atanf(imu_df->acc_y / imu_df->acc_z);
  float theta_hat_acc_rad = asinf(imu_df->acc_x / G_MPS);
  //TODO: NAN Handling

  // Gyroscope
  float p_rps = imu_df->gyr_x;
  float q_rps = imu_df->gyr_y;
  float r_rps = imu_df->gyr_z;

  // Convert to Euler angles
  float phi_dot = p_rps + tanf(theta_hat_acc_rad) * (sinf(phi_hat_acc_rad) * q_rps + cosf(phi_hat_acc_rad) * r_rps);
  float theta_dot = cosf(phi_hat_acc_rad) * q_rps - sinf(phi_hat_acc_rad) * r_rps;

  phi_hat_rad = CF_ALPHA * phi_hat_acc_rad + (1.0 - CF_ALPHA) * (phi_hat_rad + (IMU_SAMPLE_TIME_MS / 1000.0) * phi_dot);
  theta_hat_rad = CF_ALPHA * theta_hat_acc_rad + (1.0 - CF_ALPHA) * (theta_hat_rad + (IMU_SAMPLE_TIME_MS / 1000.0) * theta_dot);

  results->phi_hat_acc_rad = phi_hat_acc_rad;
  results->theta_hat_acc_rad = theta_hat_acc_rad;
  results->p_rps = p_rps;
  results->q_rps = q_rps;
  results->r_rps = r_rps;
  results->phi_dot = phi_dot;
  results->theta_dot = theta_dot;
  results->phiHat_deg = phi_hat_rad * RAD2DEG;
  results->theta_hat_rad = theta_hat_rad * RAD2DEG;
}
