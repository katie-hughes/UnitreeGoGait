import numpy as np
import matplotlib.pyplot as plt
import math

M_PI = np.pi

calf_lo = -2.82
calf_hi = -0.89

thigh_lo = -0.69
thigh_hi =  4.50

hip_lo = -0.86
hip_hi =  0.86

print("TESTING FK\n\n")
l = 0.213

def fx(theta_t, theta_c):
  return -l*math.sin(theta_t) - l*math.sin(theta_t+theta_c)

def fy(theta_t, theta_c):
  return -l*math.cos(theta_t) - l*math.cos(theta_t+theta_c)

print("0,0:", fx(0,0), fy(0,0))

print("pi/2,0:", fx(math.pi/2,0), fy(math.pi/2,0))

print("0, pi/2", fx(0,math.pi/2), fy(0, math.pi/2))

print("pi/2, -pi/2", fx(math.pi/2,-math.pi/2), fy(math.pi/2,-math.pi/2))

print("\n\nTESTING IK\n\n")

def get_oc(ot, x):
  return math.asin(-x/l - math.sin(ot)) - ot

def ik(x,y):
  alpha = math.acos(math.sqrt(x*x + y*y)/(2*l))
  gamma = math.atan(x/y)
  print("gamma, alpha", gamma, alpha)
  ot_left = gamma + alpha
  oc_left = get_oc(ot_left, x)

  ot_right = gamma - alpha
  oc_right = get_oc(ot_right, x)
  return (ot_left, oc_left), (ot_right, oc_right)


left, right = ik(0, -2*l)

print("Left:", left)
print("Right:", right)
print("Should give 0, -2l =", -2*l)
print(fx(*left), fy(*left))
print(fx(*right), fy(*right))



print("\nTRYING NON DEGENERATE")

left, right = ik(-l, -l)

print("Left:", left)
print("Right:", right)
print("Should give -l, -l = ", -l, -l)
print(fx(*left), fy(*left))
print(fx(*right), fy(*right))

print("\nTRYING ANOTHER NON DEGENERATE")

left, right = ik(l, -l)

print("Left:", left)
print("Right:", right)
print("Should give l, -l = ", l, -l)
print(fx(*left), fy(*left))
print(fx(*right), fy(*right))


def in_workspace(x, y):
  return (x*x + y*y)<2*l


numpoints = 1000

desired_x = np.linspace(-l, l, numpoints)
desired_y = np.linspace(-l, -l, numpoints)

# plt.plot(desired_x, desired_y)
# plt.xlabel("Desired x")
# plt.ylabel("Desired Y")
# plt.show()

ot_left = []
ot_right = []
oc_left = []
oc_right = []

for i in range(0,numpoints):
  x = desired_x[i]
  y = desired_y[i]
  if in_workspace(x, y):
    left, right = ik(x, y)
    ot_left.append(left[0])
    oc_left.append(left[1])
    ot_right.append(right[0])
    oc_right.append(right[1])
  else:
    print("Too far out!!")
    break

plt.plot(ot_left, label='Thigh Joint', color='b')
plt.plot(oc_left, label='Calf Joint', color='r')
plt.axhline(thigh_lo, color='b')
plt.axhline(thigh_hi, color='b')
plt.axhline(calf_lo, color='r')
plt.axhline(calf_hi, color='r')
plt.title("Left")
plt.xlabel("Timestep")
plt.ylabel("angle (radians)")
plt.legend()
plt.savefig("left.png")
plt.show()

plt.plot(ot_right, label='Thigh Joint', color='b')
plt.plot(oc_right, label='Calf Joint', color='r')
plt.axhline(thigh_lo, color='b')
plt.axhline(thigh_hi, color='b')
plt.axhline(calf_lo, color='r')
plt.axhline(calf_hi, color='r')
plt.title("Right")
plt.xlabel("Timestep")
plt.ylabel("angle (radians)")
plt.legend()
plt.savefig("right.png")
plt.show()