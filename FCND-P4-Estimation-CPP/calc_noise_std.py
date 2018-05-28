import pandas as pd
import numpy as np

gps_x  = pd.read_csv('./config/log/Graph1.txt')
imu_ax = pd.read_csv('./config/log/Graph2.txt')

gps_x_std  = np.std(gps_x[' Quad.GPS.X'])
imu_ax_std = np.std(imu_ax[' Quad.IMU.AX'])

print('GPS position std is {0:.2f}'.format(gps_x_std))
print('IMU accelerometer position std is {0:.2f}'.format(imu_ax_std))