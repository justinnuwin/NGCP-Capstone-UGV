import matplotlib.pyplot as plt
import numpy
import pandas as pd
import sys

data = pd.read_csv(sys.argv[1])
data['rad']  = data['theta'] * 3.14 / 180
#plt.polar(data['rad'], data['dist'], linewidth=1)
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
c = ax.scatter(data['rad'], data['dist'], s = 1)
plt.show()
