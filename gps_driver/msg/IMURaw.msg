# GPCHC数据
std_msgs/Header header # 时间戳

float64 raw_gyo_x # 角速率
float64 raw_gyo_y
float64 raw_gyo_z

float64 raw_acc_x # 线加速度
float64 raw_acc_y
float64 raw_acc_z

int32 sys_state # system state
int32 gps_num_stats_used # 主天线使用的卫星数
int32 sate_stuts # 卫星状态
int32 gps_num_stats2_used # 辅天线使用的卫星数
float64 gps_age # 差分延时
int32 gps_num_sats # 主天线搜星数
int32 gps_num_sats2 # 辅天线搜星数

float64 latitude # 原始经纬度和大地高
float64 lontitude
float64 altitude

float64 pose_E_sigma # 东北天 位置 sigma值 标准差?
float64 pose_N_sigma
float64 pose_U_sigma

float64 vel_E # 东北天速度
float64 vel_N
float64 vel_U
float64 vel_body # 车体速度

float64 vel_E_sigma # 东北天 速度 sigma值 标准差?
float64 vel_N_sigma
float64 vel_U_sigma
float64 vel_body_sigma

float64 acc_X # 车辆坐标系加速度
float64 acc_Y
float64 acc_Z

float64 angle_heading # 车辆姿态 unit: rad
float64 angle_pitch
float64 angle_roll

float64 angle_heading_sigma # 车辆姿态 unit: rad
float64 angle_pitch_sigma
float64 angle_roll_sigma

float64 ang_rate_X # 车辆坐标系角速度 unit: rad/s
float64 ang_rate_Y
float64 ang_rate_Z