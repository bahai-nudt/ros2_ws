import numpy as np
import matplotlib.pyplot as plt

# 1. 读文件
data = np.loadtxt("/home/zhouwang/program/bahai-nudt/ros2_ws/delta_imu_t.txt")   # 自动忽略 # 注释

x = data[0:500]

# 2. 画图
plt.figure()
plt.plot(x, label="x")

plt.show()
