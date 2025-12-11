import numpy as np
import geographiclib.geodesic as geodesic

# 参考点的经纬度和高度
lat_ref = 39.9042   # 参考点纬度
lon_ref = 116.4074  # 参考点经度
alt_ref = 50.0      # 参考点高度

# 目标点的经纬度和高度
lat = 39.9142       # 目标点纬度
lon = 116.4075      # 目标点经度
alt = 50.5          # 目标点高度

# 使用 GeographicLib 库计算两个点的差值
geod = geodesic.Geodesic.WGS84

# 获取参考点和目标点的 ECEF 坐标差值
result = geod.Inverse(lat_ref, lon_ref, lat, lon)

# 计算纬度和经度的差异
d_lat = np.radians(lat - lat_ref)
d_lon = np.radians(lon - lon_ref)

# 计算 ECEF 坐标差值
dX = result['s12'] * np.cos(d_lat) * np.cos(d_lon)
dY = result['s12'] * np.cos(d_lat) * np.sin(d_lon)
dZ = result['s12'] * np.sin(d_lat)

# 计算 ENU 坐标差值（旋转矩阵）
lat_ref_rad = np.radians(lat_ref)
lon_ref_rad = np.radians(lon_ref)

E = -np.sin(lon_ref_rad) * dX + np.cos(lon_ref_rad) * dY
N = -np.sin(lat_ref_rad) * np.cos(lon_ref_rad) * dX - np.sin(lat_ref_rad) * np.sin(lon_ref_rad) * dY + np.cos(lat_ref_rad) * dZ
U = np.cos(lat_ref_rad) * np.cos(lon_ref_rad) * dX + np.cos(lat_ref_rad) * np.sin(lon_ref_rad) * dY + np.sin(lat_ref_rad) * dZ

# 输出 ENU 坐标
print(f"ENU坐标: E = {E}, N = {N}, U = {U}")
