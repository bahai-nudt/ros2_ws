

# 时间戳（展开 builtin_interfaces/Time）
float64   stamp

# 位置 xyz
float64[3] position        # [x, y, z]

# 姿态四元数
float64[4] orientation     # [qw, qx, qy, qz]

# 线速度 (m/s)
float64[3] linear_velocity   # [vx, vy, vz]

# 角速度 (rad/s)
float64[3] angular_velocity  # [wx, wy, wz]

# 线加速度 (m/s^2)
float64[3] linear_acceleration   # [ax, ay, az]


# 状态量
int32 status_code   # 0: 无效, 1: 有效, 2: 修正等

