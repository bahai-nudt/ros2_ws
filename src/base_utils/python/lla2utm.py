import pyproj

def lla_to_utm(lat, lon):
    # 定义 WGS84 投影（经纬度）
    wgs84 = pyproj.CRS('EPSG:4326')
    
    # 定义 UTM 投影（根据经度确定带区，这里是使用区域 33N 作为例子）
    # 如果知道具体的带区，可以更改为适当的 UTM 区域，如 'EPSG:32633' 等
    utm = pyproj.CRS('EPSG:32650')
    
    # 创建转换对象
    transformer = pyproj.Transformer.from_crs(wgs84, utm, always_xy=True)
    
    # 转换经纬度到 UTM 坐标
    easting, northing = transformer.transform(lon, lat)
    
    return easting, northing

# 示例: 北京的经纬度
lat = 39.9042
lon = 116.4074

double lat = 39.9042;   // 纬度 (北京)
double lon = 116.4074;  // 经度





easting, northing = lla_to_utm(lat, lon)

print(f"UTM 坐标: Easting = {easting}, Northing = {northing}")

