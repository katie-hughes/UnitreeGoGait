import numpy as np
import matplotlib.pyplot as plt



qs = []
dqs = []
ddqs = []
tau_ests = []

for i in range(0, 21):
  qs.append([])
  dqs.append([])
  ddqs.append([])
  tau_ests.append([])

with open('high_stand_sit.txt') as f:
  r = f.read()
  spl = r.split('---')
  print("# of messages", len(spl))
  for msg in spl:
    chop = msg[msg.find('motor_state'):msg.find('bms')]
    chop = chop[chop.find('- mode'):]
    # print(chop)
    modes = chop.split('- mode')
    print(len(modes))
    print("MODE\n\n")
    for i,m in enumerate(modes):
      m = m[m.find('q:'):m.find('q_raw')]
      values = m.split()
      print(i, values)
      if (len(values) == 8):
        qs[i].append(float(values[1]))
        dqs[i].append(float(values[3]))
        ddqs[i].append(float(values[5]))
        tau_ests[i].append(float(values[7]))


def convert_name(num):
  match num:
      case 1:
        return "FR_HIP"
      case 2:
        return "FR_THIGH"
      case 3:
        return "FR_CALF"
      case 4:
        return "FL_HIP"
      case 5:
        return "FL_THIGH"
      case 6:
        return "FL_CALF"
      case 7:
        return "RR_HIP"
      case 8:
        return "RR_THIGH"
      case 9:
        return "RR_CALF"
      case 10:
        return "RL_HIP"
      case 11:
        return "RL_THIGH"
      case 12:
        return "RL_CALF"
      case _:
        return ""


for n,i in enumerate(qs):
  if (1<=n<=12):
    plt.plot(i, label=convert_name(n))
plt.title("Q")
plt.legend()
plt.show()

for n,i in enumerate(dqs):
  if (1<=n<=12):
    plt.plot(i, label=convert_name(n))
plt.title("DQ")
plt.legend()
plt.show()

for n,i in enumerate(ddqs):
  if (1<=n<=12):
    plt.plot(i, label=convert_name(n))
plt.title("DDQ")
plt.legend()
plt.show()

for n,i in enumerate(tau_ests):
  if (1<=n<=12):
    plt.plot(i, label=convert_name(n))
plt.title("Tau_est")
plt.legend()
plt.show()
