import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches

# https://javascript.info/bezier-curve

# leg length
l = 0.213

calf_lo = -2.82
calf_hi = -0.89

thigh_lo = -0.69
thigh_hi =  4.50

hip_lo = -0.86
hip_hi =  0.86


lspan = 0.10
dl = 0.025
ddl = 0.025
stand_floor = -1.5*l
swing_height = 0.05
dswing_height = 0.025
points_x = [-1.0*lspan,
            -1.0*lspan - dl,
            -1.0*lspan - dl - ddl,
            -1.0*lspan - dl - ddl,
            -1.0*lspan - dl - ddl,
            0.0,
            0.0,
            0.0,
            lspan + dl + ddl,
            lspan + dl + ddl,
            lspan + dl,
            lspan]
points_y = [stand_floor,
            stand_floor,
            stand_floor + swing_height,
            stand_floor + swing_height,
            stand_floor + swing_height,
            stand_floor + swing_height,
            stand_floor + swing_height,
            stand_floor + swing_height + dswing_height,
            stand_floor + swing_height + dswing_height,
            stand_floor + swing_height + dswing_height,
            stand_floor,
            stand_floor]

# # points_x = [-0.2, -0.2805, -0.300, -0.300, -0.300,   0.0,   0.0,   0.0, 0.3032, 0.3032, 0.2826, 0.200] 
# points_x = [-0.18, -0.2605, -0.280, -0.280, -0.280,   0.0,   0.0,   0.0, 0.2832, 0.2832, 0.2626, 0.180] 
# # points_y = [-0.5, -0.5, -0.3611, -0.3611, -0.3611, -0.3611, -0.3611, -0.3214, -0.3214, -0.3214, -0.5, -0.5]
# points_y = [-0.3, -0.3, -0.1611, -0.1611, -0.1611, -0.1611, -0.1611, -0.1214, -0.1214, -0.1214, -0.3, -0.3]
points = [(points_x[i],points_y[i]) for i in range(len(points_x))]
print(points)

print("Points x")
for px in points_x:
  print(px, end=', ')

print()

print("Points y")
for py in points_y:
  print(round(py+0.05, 6), end=', ')
print()

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


fig = plt.figure()
ax = fig.add_subplot()
ws_circle = patches.Circle((0,0), radius=2*l, facecolor=(0, 1, 0, 0.5), label='Workspace')
ax.add_patch(ws_circle)
prev=None
for n,p in enumerate(points):
  ax.scatter(p[0], p[1], label='control '+str(n))
  if prev is not None:
    ax.plot((prev[0],p[0]),(prev[1],p[1]),linestyle='dashed',color='gray')
  prev = p
ax.plot(curve_x,curve_y, color='k')
ax.legend(bbox_to_anchor=(1.1, 1.05))
ax.set_title("Bezier Curve Generation: Order "+str(order))
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_aspect('equal', 'box')
fig.tight_layout()
plt.savefig("plots/ws+bezier_"+str(order)+".png")
plt.show()



