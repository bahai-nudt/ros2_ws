#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "config/global_config.h"
#include "data_reader.h"
#include "math/constants.h"

namespace {

bool Near(double actual, double expected, double tolerance = 1e-9) {
    return std::abs(actual - expected) <= tolerance;
}

std::filesystem::path MakeTempDir(const std::string& name) {
    const auto dir = std::filesystem::temp_directory_path() / name;
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    return dir;
}

bool TestLoadGnssPos() {
    const auto dir = MakeTempDir("msf_reader_gnsspos");
    const auto path = dir / "gnsspos.txt";
    {
        std::ofstream file(path);
        file << "ts,sol,type,lat,lon,h,std_lat,std_lon,std_h\n";
        file << "100.0,0,50,30.0,120.0,100.0,0.1,0.2,0.3\n";
    }

    msf::DataReader reader;
    if (!reader.LoadGnssPos(path.string())) {
        return false;
    }
    const auto& rows = reader.GetGnssPosData();
    if (rows.size() != 1) {
        return false;
    }
    return Near(rows[0]._timestamp, 100.0) &&
           rows[0]._pos_type == 50 &&
           Near(rows[0]._blh(0), 30.0 * msf::constants::_D2R) &&
           Near(rows[0]._blh(1), 120.0 * msf::constants::_D2R) &&
           Near(rows[0]._blh(2), 100.0) &&
           Near(rows[0]._blh_std(0), 0.1) &&
           Near(rows[0]._blh_std(1), 0.2) &&
           Near(rows[0]._blh_std(2), 0.3);
}

bool TestLoadGnssVel() {
    const auto dir = MakeTempDir("msf_reader_gnssvel");
    const auto path = dir / "gnssvel.txt";
    {
        std::ofstream file(path);
        file << "ts,type,speed,trk,ver\n";
        file << "200.0,50,5.5,370.0,-0.2\n";  // 370 deg -> 10 deg
    }

    msf::DataReader reader;
    if (!reader.LoadGnssVel(path.string())) {
        return false;
    }
    const auto& rows = reader.GetGnssVelData();
    if (rows.size() != 1) {
        return false;
    }
    return Near(rows[0]._timestamp, 200.0) &&
           rows[0]._vel_type == 50 &&
           Near(rows[0]._hor_speed, 5.5) &&
           Near(rows[0]._trk_gnd, 10.0 * msf::constants::_D2R) &&
           Near(rows[0]._ver_speed, -0.2);
}

bool TestLoadHeading() {
    const auto dir = MakeTempDir("msf_reader_heading");
    const auto path = dir / "heading.txt";
    {
        std::ofstream file(path);
        file << "ts,sol,type,base,heading,std\n";
        file << "300.0,0,50,2.0,190.0,0.5\n";  // offset 180 -> 10 deg
    }

    msf::DataReader reader;
    if (!reader.LoadHeading(path.string(), 180.0)) {
        return false;
    }
    const auto& rows = reader.GetHeadingData();
    if (rows.size() != 1) {
        return false;
    }
    return Near(rows[0]._timestamp, 300.0) &&
           rows[0]._pos_type == 50 &&
           Near(rows[0]._baseline_length, 2.0) &&
           Near(rows[0]._heading, 10.0 * msf::constants::_D2R) &&
           Near(rows[0]._heading_std, 0.5 * msf::constants::_D2R);
}

bool TestLoadImuBadpFormat() {
    const auto dir = MakeTempDir("msf_reader_imu_badp");
    const auto path = dir / "bynav.imu.txt";
    {
        std::ofstream file(path);
        file << "# timestamp qx qy qz qw gx gy gz ax ay az\n";
        file << "0 0 0 0 1 0.1 -0.2 0.3 1.0 2.0 9.8\n";
        file << "10000 0 0 0 1 0.1 -0.2 0.3 1.0 2.0 9.8\n";
    }

    msf::DataReader reader;
    if (!reader.LoadImu(path.string(), false)) {
        return false;
    }
    const auto& rows = reader.GetImuData();
    if (rows.size() != 2) {
        return false;
    }
    return Near(rows[0]._timestamp, 0.0) &&
           Near(rows[0]._gyro(0), 0.1) &&
           Near(rows[0]._gyro(1), -0.2) &&
           Near(rows[0]._gyro(2), 0.3) &&
           Near(rows[0]._accel(0), 1.0) &&
           Near(rows[0]._accel(1), 2.0) &&
           Near(rows[0]._accel(2), 9.8) &&
           Near(rows[1]._timestamp, 0.01) &&
           Near(rows[1]._dt, 0.01);
}

bool TestLoadDataGnssMode() {
    const auto dir = MakeTempDir("msf_reader_all");
    {
        std::ofstream file(dir / "bynav.imu.txt");
        file << "# timestamp qx qy qz qw gx gy gz ax ay az\n";
        file << "0 0 0 0 1 0 0 0 0 0 9.8\n";
        file << "10000 0 0 0 1 0 0 0 0 0 9.8\n";
    }
    {
        std::ofstream file(dir / "gnsspos.txt");
        file << "ts,sol,type,lat,lon,h,std_lat,std_lon,std_h\n";
        file << "0.02,0,50,30.0,120.0,100.0,0.1,0.2,0.3\n";
    }
    {
        std::ofstream file(dir / "gnssvel.txt");
        file << "ts,type,speed,trk,ver\n";
        file << "0.02,50,1.0,90.0,0.0\n";
    }
    {
        std::ofstream file(dir / "heading.txt");
        file << "ts,sol,type,base,heading,std\n";
        file << "0.02,0,50,2.0,90.0,0.5\n";
    }

    msf::GlobalConfig config;
    config._file._input_data_dir = dir.string();

    msf::DataReader reader;
    if (!reader.LoadData(config, msf::LoadOptions{true, false})) {
        return false;
    }
    return reader.GetImuCount() == 2 &&
           reader.GetGnssPosCount() == 1 &&
           reader.GetGnssVelCount() == 1 &&
           reader.GetHeadingCount() == 1 &&
           Near(reader.GetStartTime(), 0.0) &&
           Near(reader.GetEndTime(), 0.02);
}

bool TestLoadDataImuOnlySkipsGnss() {
    const auto dir = MakeTempDir("msf_reader_imu_only");
    {
        std::ofstream file(dir / "bynav.imu.txt");
        file << "# timestamp qx qy qz qw gx gy gz ax ay az\n";
        file << "0 0 0 0 1 0 0 0 0 0 9.8\n";
    }

    msf::GlobalConfig config;
    config._file._input_data_dir = dir.string();

    msf::DataReader reader;
    if (!reader.LoadData(config, msf::LoadOptions{false, false})) {
        return false;
    }
    return reader.GetImuCount() == 1 &&
           reader.GetGnssPosCount() == 0 &&
           reader.GetGnssVelCount() == 0 &&
           reader.GetHeadingCount() == 0;
}

}  // namespace

int main() {
    struct TestCase {
        const char* name;
        bool (*fn)();
    };

    const TestCase tests[] = {
        {"load_gnss_pos", TestLoadGnssPos},
        {"load_gnss_vel", TestLoadGnssVel},
        {"load_heading", TestLoadHeading},
        {"load_imu_badp_format", TestLoadImuBadpFormat},
        {"load_data_gnss_mode", TestLoadDataGnssMode},
        {"load_data_imu_only_skips_gnss", TestLoadDataImuOnlySkipsGnss},
    };

    int failed = 0;
    for (const auto& test : tests) {
        if (!test.fn()) {
            std::cerr << "[FAIL] " << test.name << std::endl;
            ++failed;
        } else {
            std::cout << "[PASS] " << test.name << std::endl;
        }
    }

    return failed == 0 ? 0 : 1;
}
