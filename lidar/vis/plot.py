import matplotlib.pyplot as plt
import numpy
import pandas as pd

data = pd.read_csv("data1.csv")
print(data)
indices = [0]
oldTheta = -1
for i, j in data.iterrows():
    if(j["theta"] < oldTheta):
        indices.append(i)
    oldTheta = j["theta"]
indices.append(i+1)

a = 0
for i in range(0, len(indices) - 1):
    tempData = data.loc[indices[a] : indices[a + 1] - 1]
    tempData['rad']  = tempData['theta'] * 3.14 / 180
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar')
    c = ax.scatter(tempData['rad'], tempData['dist'], s = 1)
    a += 1
plt.show()
