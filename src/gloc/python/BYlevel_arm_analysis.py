import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import pymap3d as pm


def lla2enu(lla, init_lla):
    e, n, u = pm.geodetic2enu(lla[1], lla[0], lla[2], init_lla[0], init_lla[1], init_lla[2])
    return np.array([e, n, u])


def euler2angle(roll, pitch, yaw):
    euler_angles = [roll, pitch, yaw]

    # 'xyz' 表示绕 X-Y-Z 旋转
    rotation = R.from_euler('xyz', euler_angles)
    R_matrix = rotation.as_matrix()  # 3x3 NumPy array

    return R_matrix


def main():

    # 1773712717.094511032

    lever_arm = np.array([-0.51, 1.65, 1.80])

    gps_heading = 301.389508431 / 180 * math.pi + math.pi / 2 - math.pi * 2 #32.270580292 / 180 * math.pi
    bynavx_azimuth = 301.389508431 / 180 * math.pi


    imu_t = lla2enu(np.array([118.219097534,41.141938617,703.764633235]), np.array([41.141938617,118.219097534,703.764633235]))
    gps_t = lla2enu(np.array([118.219077811,41.141938524,706.085410926]), np.array([41.141938617,118.219097534,703.764633235]))

    gps_heading = (gps_heading - math.pi / 2.0)
    R_heading = euler2angle(0, 0, -gps_heading)
    residual = gps_t - (imu_t + R_heading @ lever_arm)

    print(imu_t)
    print(gps_t)
    print(residual)



if __name__ == "__main__":
    main()
