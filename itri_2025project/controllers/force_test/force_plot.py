import pandas as pd
import matplotlib.pyplot as plt

# 讀資料
df = pd.read_csv(r"C:\Users\vivian\OneDrive - NTHU\桌面\ITRI\webot_practice_2025ver\controllers\force_test\force_data.csv")

# 畫 X、Y、Z 三條曲線
# plt.plot(df['time'], df['fx'], label='Fx')
# plt.plot(df['time'], df['fy'], label='Fy')
plt.plot(df['time'], df['fz'], label='Fz')

plt.xlabel('Time (s)')
plt.ylabel('Force (N)')
plt.title('Force Sensor Readings Over Time')
plt.legend()
plt.grid(True)
plt.show()
