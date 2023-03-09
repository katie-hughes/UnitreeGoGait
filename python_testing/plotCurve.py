import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

curvex = []
curvey = []

with open("myCurve.txt") as f:
  lines = f.readlines()
  for l in lines:
    splt = l.split()
    # print(splt)
    # x in place 6 y in place 7
    x = splt[6]
    y = splt[7]
    x = x.strip('(')
    x = x.strip(',')
    y = y.strip(')')
    # print(x, y)
    x_int = float(x)
    y_int = float(y)
    print(x_int, y_int)
    curvex.append(x_int)
    curvey.append(y_int)


colors = cm.rainbow(np.linspace(0, 1, len(curvex)))

for i in range(0, len(curvex)):
  plt.scatter(curvex[i], curvey[i], color = colors[i])

plt.savefig("plots/myCurve.png")
plt.show()