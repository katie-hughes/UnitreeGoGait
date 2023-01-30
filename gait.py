import numpy as np
import matplotlib.pyplot as plt
import math

M_PI = np.pi

motiontime = np.linspace(0,10000,10000)

calf_1 = -M_PI / 2 + 0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
calf_2 = -M_PI / 2 - 0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
calf_lo = -2.82
calf_hi = -0.89

thigh_1 =  0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
thigh_2 = -0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
thigh_lo = -0.69
thigh_hi =  4.50

hip_1 =  0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
hip_2 = -0.5 * np.sin(2 * M_PI / 5.0 * motiontime * 1e-3)
hip_lo = -0.86
hip_hi =  0.86

plt.plot(motiontime, calf_1, color='r', label='Calf 1')
plt.plot(motiontime, calf_2, color='r', label='Calf 2')
plt.axhline(calf_lo, color='r')
plt.axhline(calf_hi, color='r', label='Calf joint limits')

plt.plot(motiontime, thigh_1, color='b', label='thigh 1')
plt.plot(motiontime, thigh_2, color='b', label='thigh 2')
plt.axhline(thigh_lo, color='b')
plt.axhline(thigh_hi, color='b', label='Joint limits')
plt.ylim(-1.5*M_PI, 1.5*M_PI)

plt.legend()
plt.title("Joint trajectories: 10s")
plt.ylabel("Joint angle (radians)")
plt.xlabel("Ticks")


plt.savefig('thigh_calf.png')
plt.close()


print("TESTING FK\n\n")
# desired foot position
l = 0.213
x = 0
y = -2*l


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