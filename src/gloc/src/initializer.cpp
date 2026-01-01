    #include "gloc/initializer.h"
    #include "gloc/message.h"
    #include "gloc/gloc_data_buffer.h"

    #include "base_utils/coordinate.h"
    

    bool Initializer::init_by_gps(const GpsData& gps_data, State& state) {

        if (SingletonDataBuffer::getInstance()._imu_buffer.cache.size() < 80 || 
            SingletonDataBuffer::getInstance()._heading_buffer.cache.size() < 1 || 
            SingletonDataBuffer::getInstance()._gps_buffer.cache.size() < 1) {
            RCLCPP_WARN(logger, "imubuffer size, %d", SingletonDataBuffer::getInstance()._imu_buffer.cache.size());
            RCLCPP_WARN(logger, "heading buffer size, %d", SingletonDataBuffer::getInstance()._heading_buffer.cache.size());
            RCLCPP_WARN(logger, "gps buffer size, %d", SingletonDataBuffer::getInstance()._gps_buffer.cache.size());

            return false;
        }

        Heading heading = SingletonDataBuffer::getInstance()._heading_buffer.cache.back();
        GpsData last_gps = SingletonDataBuffer::getInstance()._gps_buffer.cache.back();
        Ins ins = SingletonDataBuffer::getInstance()._ins_buffer.cache.back();


        if (fabs(heading._timestamp - gps_data._timestamp) > 1.0) {
            RCLCPP_WARN(logger, "heading timestamp is far away from gos timestamp:%d : %d", heading._timestamp, gps_data._timestamp);
            return false;
        }

        state._init_lla = gps_data._lla;
        // 检查gps状态是否为4 TODO

        std::vector<double> last_enu = Coordinate::lla2enu(state._init_lla(0), state._init_lla(1), state._init_lla(2), last_gps._lla(0), last_gps._lla(1), last_gps._lla(2));

        Eigen::Vector3d last_enu_vec(last_enu[0], last_enu[1], last_enu[2]);
        Eigen::Vector3d cur_enu_vec(0.0, 0.0, 0.0);

        double delta_t = gps_data._timestamp - last_gps._timestamp;


        // 北云设备， heading与azimuth定义一样，正北为0,顺时针为正


        heading._heading = (heading._heading - M_PI / 2.0);
        std::cout << "ins azimuth: " << ins._azimuth << std::endl;

        Eigen::AngleAxisd rotation_vector(-heading._heading, Eigen::Vector3d(0, 0, 1));

        state._timestamp = gps_data._timestamp;
        state._position = -rotation_vector.toRotationMatrix() * _lever_arm;
        state._velocity = (Eigen::Vector3d(0, 0, 0) - last_enu_vec) / delta_t;//Eigen::Vector3d(0, 0, 0);
        state._rotation = rotation_vector.toRotationMatrix();
        state._bias_accel = Eigen::Vector3d(0.0793282, -0.0240076, -0.0114863);
        state._bias_gyro = Eigen::Vector3d(0.00127599, 0.00106483, 0.0011411);

        state._cov = Eigen::Matrix<double, 15, 15>::Identity();
        state._cov.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 0.3;
        state._cov.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity() * 0.2;
        state._cov.block(6, 6, 3, 3) = Eigen::Matrix3d::Identity() * 0.05;
        state._cov.block(9, 9, 3, 3) = Eigen::Matrix3d::Identity() * 0.001;
        state._cov.block(12, 12, 3, 3) = Eigen::Matrix3d::Identity() * 0.0001;

        state._timestamp = gps_data._timestamp;

        Eigen::Vector3d euler_angles = rotation_vector.toRotationMatrix().eulerAngles(2, 0, 1);

        std::cout << "初始化角度： " << euler_angles(0) * 180 / 3.1415926<< std::endl;

        std::cout << "初始化成功，初始化速度: " << state._velocity(0) << "," << state._velocity(1) << "," << state._velocity(2) << std::endl;
        return true;
    }