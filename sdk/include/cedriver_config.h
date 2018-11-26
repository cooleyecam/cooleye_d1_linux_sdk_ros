#ifndef CEDRIVER_CONFIG_H
#define CEDRIVER_CONFIG_H

void ce_config_load_settings(const char* settings_file);

std::string ce_config_get_cf_dev_name();
std::string ce_config_get_cf_dev_version();
std::string ce_config_get_cf_imu_uart();

int ce_config_get_cf_imu_icm20689_acc_bias_X();
int ce_config_get_cf_imu_icm20689_acc_bias_Y();
int ce_config_get_cf_imu_icm20689_acc_bias_Z();
int ce_config_get_cf_imu_icm20689_gyro_bias_X();
int ce_config_get_cf_imu_icm20689_gyro_bias_Y();
int ce_config_get_cf_imu_icm20689_gyro_bias_Z();
void ce_config_get_cf_imu_icm20689_acc_T(float **mat);
int ce_config_get_cf_imu_icm20689_sample_rate();

int ce_config_get_cf_cam_mode();
int ce_config_get_cf_cam_resolution();
int ce_config_get_cf_cam_FPS();
int ce_config_get_cf_cam_EG_mode();
int ce_config_get_cf_cam_man_exp();
int ce_config_get_cf_cam_man_gain();
int ce_config_get_cf_cam_auto_EG_top();
int ce_config_get_cf_cam_auto_EG_bottom();
int ce_config_get_cf_cam_auto_EG_des();
int ce_config_get_cf_cam_auto_E_man_G_Etop();
int ce_config_get_cf_cam_auto_E_man_G_Ebottom();
int ce_config_get_cf_cam_auto_E_man_G();
int ce_config_get_cf_cam_agc_aec_skip_frame();
int ce_config_get_cf_cam_rectify();
int ce_config_get_cf_cam_rectify_force_on();
int ce_config_get_cf_cam_rectify_force_off();

std::string ce_config_get_cf_cam_intrinsics();
std::string ce_config_get_cf_cam_extrinsics();


int ce_config_get_cf_img_width();
int ce_config_get_cf_img_height();
int ce_config_get_cf_img_size();
int ce_config_get_cf_img_buff_size();
int ce_config_get_cf_img_HB();
int ce_config_get_cf_img_VB();
double ce_config_get_cf_img_time_offset();

int ce_config_get_cf_ros_showimage();
int ce_config_get_cf_ste_algorithm();

void  ce_config_rst_imu_offset(void);
void  ce_config_rewrite_imu_offset(std::string filepath, float *gyro_offs, float *accel_offs, float **accel_T);



#endif
