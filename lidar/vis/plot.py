import matplotlib.pyplot as plt
import numpy
import pandas as pd

data = pd.read_csv("data.csv")
data['rad']  = data['theta'] * 3.14 / 180
#plt.polar(data['rad'], data['dist'], linewidth=1)
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')
c = ax.scatter(data['rad'], data['dist'], s = 1)
plt.show()
