import matplotlib.pyplot as plt
import numpy as np

# https://javascript.info/bezier-curve

points_x = [0,1,2]
points_y = [0,1,0]
points = [(points_x[i],points_y[i]) for i in range(len(points_x))]
print(points)

step = 0.05

# doing it like this bc c++ has no built in factorial program

def fact(n):
  if (n==1 or n==0):
    return 1
  else:
    return fact(n-1)*n


def choose(n,k):
  return fact(n)/(fact(k)*fact(n-k))


def pi(i,order,t,point):
  return choose(order,i)*((1-t)**(order-i))*(t**i)*point


def bez(points, step):
  # points: 1D array
  # Step: step size for parameterization t as it goes from 0 to 1
  order = len(points)
  curve = []
  ts = np.arange(0,1,step)
  for t in ts:
    current = 0
    for i in range(0,order):
      current += pi(i,order,t,points[i])
    curve.append(current)
  return curve

curve_x = bez(points_x, step)
curve_y = bez(points_y, step)

plt.plot(curve_x,curve_y, color='k')
for n,p in enumerate(points):
  plt.scatter(p[0], p[1], label='Control Point #'+str(n))
plt.legend()
plt.title("Bezier Curve Generation")
plt.show()



