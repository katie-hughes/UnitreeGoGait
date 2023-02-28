import numpy as np
import matplotlib.pyplot as plt


# while Kd = 5, standup time = 1.5

kp = [35, 40, 45, 50, 55, 60, 65]
err = [0.318, 0.285, 0.248, 0.224, 0.208, 0.187, 0.175]


plt.plot(kp, err)
plt.ylim(0, 0.5)
plt.show()