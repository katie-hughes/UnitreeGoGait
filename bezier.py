import matplotlib.pyplot as plt
import numpy as np

# https://javascript.info/bezier-curve

points_x = [-200.0, -280.5, -300.0, -300.0, -300.0,   0.0,   0.0,   0.0, 303.2, 303.2, 282.6, 200.0]
points_y = [ 500.0,  500.0,  361.1,  361.1,  361.1, 361.1, 361.1, 321.4, 321.4, 321.4, 500.0, 500.0]
points = [(points_x[i],points_y[i]) for i in range(len(points_x))]
print(points)

step = 0.01

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
  # had to do this otherwise would crop before I reach 1
  ts = np.arange(0,1.0000001,step)
  for t in ts:
    current = 0
    for i in range(0,order):
      current += pi(i,order-1,t,points[i])
    curve.append(current)
  return curve

curve_x = bez(points_x, step)
curve_y = bez(points_y, step)

order = len(points)-1



prev=None
for n,p in enumerate(points):
  plt.scatter(p[0], p[1], label='control '+str(n))
  if prev is not None:
    plt.plot((prev[0],p[0]),(prev[1],p[1]),linestyle='dashed',color='gray')
  prev = p
plt.plot(curve_x,curve_y, color='k')
plt.legend()
plt.title("Bezier Curve Generation: Order "+str(order))
plt.xlabel("X")
plt.ylabel("Y")

plt.savefig("plots/bezier_"+str(order)+".png")
plt.show()



