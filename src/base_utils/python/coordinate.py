from pyproj import CRS, Transformer

# 定义 WGS84 的经纬度和ECEF坐标系统
lla_crs = CRS("EPSG:4326")  # WGS84 经纬度 (LLA)
ecef_crs = CRS("EPSG:4978")  # ECEF坐标系

# 创建一个转换器
transformer = Transformer.from_crs(lla_crs, ecef_crs, always_xy=True)

def lla_to_ecef(latitude, longitude, altitude):
    # 将经纬度（LLA）转换为ECEF坐标
    x, y, z = transformer.transform(longitude, latitude, altitude)
    return x, y, z

# 输入经纬度和高度
latitude = 39.9042  # 纬度 (单位：度)
longitude = 116.4074  # 经度 (单位：度)
altitude = 50.0  # 高度 (单位：米)

# 转换为 ECEF 坐标
x, y, z = lla_to_ecef(latitude, longitude, altitude)

# 输出结果
print(f"ECEF坐标: x = {x}, y = {y}, z = {z}")

