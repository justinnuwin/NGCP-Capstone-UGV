import matplotlib.pyplot as plt
import numpy
import pandas as pd

data = pd.read_csv("data.csv")
data['rad']  = data['theta'] * 3.14 / 180
plt.polar(data['rad'], data['dist'], linewidth=1)
plt.show()
