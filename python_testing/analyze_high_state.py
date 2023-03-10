import numpy as np
import matplotlib.pyplot as plt



qs = []
dqs = []
ddqs = []
tau_ests = []

low_qs = []

for i in range(0, 21):
  qs.append([])
  dqs.append([])
  ddqs.append([])
  tau_ests.append([])
  low_qs.append([])

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

with open('custom_stand_sit.txt') as f:
  r = f.read()
  spl = r.split('---')
  print("# of messages", len(spl))
  for msg in spl:
    chop = msg[msg.find('motor_cmd'):msg.find('bms')]
    chop = chop[chop.find('- mode'):]
    # print(chop)
    modes = chop.split('- mode')
    print(len(modes))
    print("MODE\n\n")
    for i,m in enumerate(modes):
      m = m[m.find('q:'):m.find('kp')]
      values = m.split()
      print(i, values)
      if (len(values) == 6):
        low_qs[i].append(float(values[1]))



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

def plotarray(arr, name, lab=None):
  for n,i in enumerate(arr):
    if (1<=n<=12):
      mylabel = convert_name(n)
      if lab is not None:
        if lab in mylabel:
          plt.plot(i, label=mylabel)
      else:
        plt.plot(i, label=mylabel)
  plottitle = name
  if lab is not None:
    plottitle += '_'+lab
  plt.title(plottitle)
  plt.legend()
  plt.savefig('plots/'+plottitle+'.png')
  plt.show()


def plotall(arr, title):
  plotarray(arr, title, lab="CALF")
  plotarray(arr, title, lab="HIP")
  plotarray(arr, title, lab="THIGH")


plotall(qs, "Q_HIGH")
plotall(low_qs, "Q_LOW")

plotall(dqs, "DQ_HIGH")
plotall(ddqs, "DDQ_HIGH")
plotall(tau_ests, "TAU_EST_HIGH")

# plotarray(tau_ests, "Tau", lab="CALF")
# plotarray(tau_ests, "Tau", lab="HIP")
# plotarray(tau_ests, "Tau", lab="THIGH")