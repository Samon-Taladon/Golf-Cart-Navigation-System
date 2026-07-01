import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("path_smoothlog6_resampled_1m.csv")

plt.plot(df['x'], df['y'])
plt.axis('equal')
plt.grid()
plt.show()