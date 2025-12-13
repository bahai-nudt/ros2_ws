    #include "gloc/initializer.h"
    #include "gloc/message.h"
    #include "gloc/gloc_data_buffer.h"

    #include "base_utils/coordinate.h"

    bool Initializer::init_by_gps(const GpsData& gps_data, State& state) {

        if (SingletonDataBuffer::getInstance()._imu_buffer.cache.size() < 80 || SingletonDataBuffer::getInstance()._heading_buffer.cache.size() < 1) {
            RCLCPP_WARN(logger, "imubuffer size, %d", SingletonDataBuffer::getInstance()._imu_buffer.cache.size());
            RCLCPP_WARN(logger, "heading buffer size, %d", SingletonDataBuffer::getInstance()._heading_buffer.cache.size());

            return false;
        }

        Heading heading = SingletonDataBuffer::getInstance()._heading_buffer.cache.back();

        if (fabs(heading._timestamp - gps_data._timestamp) > 1.0) {
            RCLCPP_WARN(logger, "heading timestamp is far away from gos timestamp:%d : %d", heading._timestamp, gps_data._timestamp);
            return false;
        }

        state._init_lla = gps_data._lla;
        // 检查gps状态是否为4 TODO

        // 北云设备， heading与azimuth定义一样，正北为0,顺时针为正
        Eigen::AngleAxisd rotation_vector(heading._heading , Eigen::Vector3d(0, 0, 1));

        state._timestamp = gps_data._timestamp;
        state._position = Eigen::Vector3d(0, 0, 0);
        state._velocity = Eigen::Vector3d(0, 0, 0);
        state._rotation = rotation_vector.toRotationMatrix();
        state._bias_accel = Eigen::Vector3d(0, 0, 0);
        state._bias_gyro = Eigen::Vector3d(0, 0, 0);

        state._cov = Eigen::Matrix<double, 15, 15>::Zero();

        return true;
    }